#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion()
    {
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("tde_lio/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("tde_lio/imu/path", 1);
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // static tf
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;
    ros::Publisher pubExtDisturbance;   // external disturbance published as sensor_msgs/Imu
    ros::Publisher pubImuCorrected;     // imuConverter'd IMU minus bias minus disturbance (for deskew)
    ros::Subscriber subObsBias;         // optional: IMU bias from the mapOptimization geometric observer
    gtsam::Vector3 obsBiasGyro_  = gtsam::Vector3::Zero();
    gtsam::Vector3 obsBiasAccel_ = gtsam::Vector3::Zero();
    bool useObserverBias_ = false;      // tde_lio/useObserverBias: subtract observer bias instead of factor-graph bias
    ros::Subscriber subTdeCorrected;    // /imu_eskf/tde_corrected feedback from the ESKF
    // ESKF-corrected disturbance. The handler sets a TARGET; the value actually subtracted
    // (tdeGyroCorr_/tdeAccelCorr_) is slewed toward the target by a first-order low-pass at
    // IMU rate (Eq 24-26). This removes the 10 Hz step that a held LiDAR-rate correction
    // injects into the high-rate IMU odometry, and band-limits the feedback to the slowly
    // varying, LiDAR-observable part of the disturbance (the only part LiDAR can see).
    gtsam::Vector3 tdeGyroCorr_    = gtsam::Vector3::Zero();   // eps^w applied (smoothed)
    gtsam::Vector3 tdeAccelCorr_   = gtsam::Vector3::Zero();   // eps^a applied (smoothed)
    gtsam::Vector3 tdeGyroTarget_  = gtsam::Vector3::Zero();   // eps^w target from the ESKF
    gtsam::Vector3 tdeAccelTarget_ = gtsam::Vector3::Zero();   // eps^a target from the ESKF
    // SAFE DEFAULTS: feedback OFF unless explicitly enabled. The accel channel is the
    // dangerous one (gravity-projection leak + double-differentiated LiDAR noise), so it
    // has its own flag and is OFF even when the master switch is on.
    bool   useTdeFeedback_ = false;   // master switch (tde_lio/useTdeFeedback)
    bool   useTdeAccelFb_  = false;   // also feed back the accel disturbance (tde_lio/useTdeAccelFeedback)
    double epsGyroMax_     = 1.0;     // |eps^w| per-axis clamp [rad/s]   (tde_lio/epsGyroMax)
    double epsAccelMax_    = 3.0;     // |eps^a| per-axis clamp [m/s^2]   (tde_lio/epsAccelMax)
    double tdeTau_         = 0.30;    // feedback low-pass time constant [s] (tde_lio/tdeTau)

    // ---- per-node history for the TDE external-disturbance prediction ----
    // One record is stored at every factor-graph node (= lidar keyframe time t).
    struct NodeRecord
    {
        double         time;        // node timestamp [s]
        double         dt;          // interval ending at this node (t_k - t_{k-1}) [s]
        gtsam::Vector3 gyroRaw;     // raw IMU gyro  at the node  (omega tilde)
        gtsam::Vector3 accelRaw;    // raw IMU accel at the node  (a tilde)
        gtsam::Vector3 biasGyro;    // optimized gyro  bias at the node
        gtsam::Vector3 biasAccel;   // optimized accel bias at the node
        gtsam::Rot3    R;           // optimized orientation (body->world), lidar-corrected
        gtsam::Vector3 vel;         // optimized world-frame velocity, lidar-corrected
        gtsam::Vector3 distGyro;    // estimated gyro  disturbance at the node
        gtsam::Vector3 distAccel;   // estimated accel disturbance at the node
    };
    std::deque<NodeRecord> nodeHist;
    // last valid disturbance estimate, republished to keep the topic continuous
    // through bias/velocity resets and the bootstrap window.
    gtsam::Vector3 lastDistGyro  = gtsam::Vector3::Zero();
    gtsam::Vector3 lastDistAccel = gtsam::Vector3::Zero();

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;
    
    // T_bl: tramsform points from lidar frame to imu frame 
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration()
    {
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("tde_lio/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subTdeCorrected = nh.subscribe<sensor_msgs::Imu>("/imu_eskf/tde_corrected", 50, &IMUPreintegration::tdeCorrectedHandler, this, ros::TransportHints().tcpNoDelay());
        nh.param<bool>  ("tde_lio/useTdeFeedback",      useTdeFeedback_, false);
        nh.param<bool>  ("tde_lio/useTdeAccelFeedback", useTdeAccelFb_,  false);
        nh.param<double>("tde_lio/epsGyroMax",          epsGyroMax_,     1.0);
        nh.param<double>("tde_lio/epsAccelMax",         epsAccelMax_,    3.0);
        nh.param<double>("tde_lio/tdeTau",              tdeTau_,         0.30);
        ROS_WARN("[TDE-fb] feedback=%s  accel_channel=%s  clamp(|w|<=%.2f, |a|<=%.2f)  tau=%.2fs",
                 useTdeFeedback_ ? "ON" : "OFF", useTdeAccelFb_ ? "ON" : "OFF", epsGyroMax_, epsAccelMax_, tdeTau_);

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        pubExtDisturbance = nh.advertise<sensor_msgs::Imu>("tde_lio/imu/external_disturbance", 200);
        pubImuCorrected   = nh.advertise<sensor_msgs::Imu>("tde_lio/imu/corrected", 2000);   // bias+disturbance-corrected IMU for deskew
        subObsBias = nh.subscribe<sensor_msgs::Imu>("tde_lio/observer/imu_bias", 50, &IMUPreintegration::observerBiasHandler, this, ros::TransportHints().tcpNoDelay());
        nh.param<bool>("tde_lio/useObserverBias", useObserverBias_, false);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
        nodeHist.clear();
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
        {
            publishDisturbance(currentCorrectionTime, lastDistGyro, lastDistAccel);
            return;
        }

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (systemInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            publishDisturbance(currentCorrectionTime, lastDistGyro, lastDistAccel);
            return;
        }


        // reset graph for speed
        if (key == 100)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }


        // 1. integrate imu data and optimize
        gtsam::Vector3 lastRawGyro(0, 0, 0), lastRawAccel(0, 0, 0);  // raw IMU nearest the keyframe time (omega/a tilde at node t)
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z) - tdeAccelCorr_,
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z)    - tdeGyroCorr_,  dt);

                lastRawAccel = gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z);
                lastRawGyro  = gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // ===== external disturbance model prediction (TDE) =====
        double dtNode = imuIntegratorOpt_->deltaTij();   // interval [prev keyframe, this keyframe]
        predictExternalDisturbance(currentCorrectionTime, dtNode, lastRawGyro, lastRawAccel);
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z) - tdeAccelCorr_,
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z)    - tdeGyroCorr_, dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    // Publish a disturbance as sensor_msgs/Imu (gyro -> angular_velocity, accel ->
    // linear_acceleration). Called on every code path so the topic never goes silent.
    void publishDisturbance(double stamp, const gtsam::Vector3& distGyro, const gtsam::Vector3& distAccel)
    {
        sensor_msgs::Imu distMsg;
        distMsg.header.stamp    = ros::Time(stamp);
        distMsg.header.frame_id = baselinkFrame;
        distMsg.orientation.w   = 1.0;   // unused
        distMsg.angular_velocity.x    = distGyro.x();
        distMsg.angular_velocity.y    = distGyro.y();
        distMsg.angular_velocity.z    = distGyro.z();
        distMsg.linear_acceleration.x = distAccel.x();
        distMsg.linear_acceleration.y = distAccel.y();
        distMsg.linear_acceleration.z = distAccel.z();
        pubExtDisturbance.publish(distMsg);
    }

    // ============================================================================
    //  Model prediction of the external disturbance via Time-Delay Estimation (TDE)
    //  Runs once per factor-graph node (lidar keyframe time t). Uses ONLY the IMU
    //  measurement model + TDE; the "true" motion increments (Delta-omega, Delta-a)
    //  are taken from the lidar-corrected optimized navstates (lidar odometry is the
    //  reference). mu (the TDE residual) is dropped here — that is what the later
    //  ESKF/LiDAR update is meant to estimate.
    //
    //  Eq.(8)  eps^w_t = w~_{t-T} - b^w_{t-T} - w~_{t-2T} + b^w_{t-2T} + eps^w_{t-2T}
    //                    - ( wbar_{t-T} - wbar_{t-2T} )
    //  Eq.(9)  eps^a_t = a~_{t-T} - b^a_{t-T} - a~_{t-2T} + b^a_{t-2T} + eps^a_{t-2T}
    //                    - ( abody_{t-T} - abody_{t-2T} ) + ( R_{t-T}^T - R_{t-2T}^T ) g
    //
    //  Published as sensor_msgs/Imu:
    //     angular_velocity    = gyro  disturbance eps^w  [rad/s]
    //     linear_acceleration = accel disturbance eps^a  [m/s^2]
    // ============================================================================
    void predictExternalDisturbance(double stamp, double dtNode,
                                    const gtsam::Vector3& gyroRaw,
                                    const gtsam::Vector3& accelRaw)
    {
        // build the record for the current node t from the just-optimized state
        NodeRecord rec;
        rec.time      = stamp;
        rec.dt        = (dtNode > 1e-6) ? dtNode : (1.0 / 10.0);
        rec.gyroRaw   = gyroRaw;
        rec.accelRaw  = accelRaw;
        rec.biasGyro  = prevBias_.gyroscope();
        rec.biasAccel = prevBias_.accelerometer();
        rec.R         = prevPose_.rotation();
        rec.vel       = prevVel_;
        rec.distGyro  = gtsam::Vector3::Zero();   // default during bootstrap
        rec.distAccel = gtsam::Vector3::Zero();

        nodeHist.push_back(rec);
        while (nodeHist.size() > 10) nodeHist.pop_front();   // keep history bounded

        // need t-T, t-2T, t-3T in addition to t  ->  at least 4 nodes.
        // While bootstrapping (or rebuilding after a reset) keep the topic alive by
        // republishing the last valid estimate instead of going silent.
        if (nodeHist.size() < 4)
        {
            publishDisturbance(stamp, lastDistGyro, lastDistAccel);
            return;
        }

        const NodeRecord& n1 = nodeHist[nodeHist.size() - 2]; // t - T
        const NodeRecord& n2 = nodeHist[nodeHist.size() - 3]; // t - 2T
        const NodeRecord& n3 = nodeHist[nodeHist.size() - 4]; // t - 3T

        const gtsam::Vector3 gW(0.0, 0.0, -imuGravity);       // world gravity (MakeSharedU)

        // ---- gyro: change in lidar-derived average angular velocity ----
        // wbar over an interval = Log( R_{start}^T R_{end} ) / dt   (average body rate)
        gtsam::Vector3 wbar_1 = gtsam::Rot3::Logmap(n2.R.between(n1.R)) / n1.dt;  // over [t-2T, t-T]
        gtsam::Vector3 wbar_2 = gtsam::Rot3::Logmap(n3.R.between(n2.R)) / n2.dt;  // over [t-3T, t-2T]
        gtsam::Vector3 dOmega = wbar_1 - wbar_2;

        gtsam::Vector3 distGyro = n1.gyroRaw - n1.biasGyro
                                - n2.gyroRaw + n2.biasGyro
                                + n2.distGyro
                                - dOmega;

        // ---- accel: change in lidar-derived average kinematic acceleration (body frame) ----
        // abar_w = Delta v_world / dt   (from optimized world velocities);  abody = R^T abar_w
        gtsam::Vector3 abarW_1 = (n1.vel - n2.vel) / n1.dt;   // world, over [t-2T, t-T]
        gtsam::Vector3 abarW_2 = (n2.vel - n3.vel) / n2.dt;   // world, over [t-3T, t-2T]
        gtsam::Vector3 aBody_1 = n1.R.unrotate(abarW_1);      // R_{t-T}^T  abar_w
        gtsam::Vector3 aBody_2 = n2.R.unrotate(abarW_2);      // R_{t-2T}^T abar_w
        gtsam::Vector3 dAccel  = aBody_1 - aBody_2;

        // gravity-difference term: ( R_{t-T}^T - R_{t-2T}^T ) g
        gtsam::Vector3 gravTerm = n1.R.unrotate(gW) - n2.R.unrotate(gW);

        gtsam::Vector3 distAccel = n1.accelRaw - n1.biasAccel
                                 - n2.accelRaw + n2.biasAccel
                                 + n2.distAccel
                                 - dAccel
                                 + gravTerm;

        // carry forward on the current node so eps_{t-2T} is available next steps
        nodeHist.back().distGyro  = distGyro;
        nodeHist.back().distAccel = distAccel;

        // remember the latest valid estimate, then publish it
        lastDistGyro  = distGyro;
        lastDistAccel = distAccel;
        publishDisturbance(stamp, distGyro, distAccel);
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void observerBiasHandler(const sensor_msgs::Imu::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        obsBiasGyro_  = gtsam::Vector3(msg->angular_velocity.x,    msg->angular_velocity.y,    msg->angular_velocity.z);
        obsBiasAccel_ = gtsam::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    }

    void tdeCorrectedHandler(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // Stores the latest ESKF disturbance as a TARGET only. The applied correction is
        // slewed toward it at IMU rate (slewTdeFeedback) so nothing steps at the LiDAR rate.
        // NOTE: assumed expressed in the same (extRot-converted) frame the preintegrators
        // integrate in. If the ESKF runs on raw IMU, rotate by extRot before storing.
        std::lock_guard<std::mutex> lock(mtx);
        if (!useTdeFeedback_) {
            tdeGyroTarget_.setZero();  tdeAccelTarget_.setZero();
            tdeGyroCorr_.setZero();    tdeAccelCorr_.setZero();
            return;
        }

        gtsam::Vector3 g(msg->angular_velocity.x,    msg->angular_velocity.y,    msg->angular_velocity.z);
        gtsam::Vector3 a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        if (!g.allFinite() || !a.allFinite()) {
            ROS_WARN_THROTTLE(2.0, "[TDE-fb] non-finite disturbance ignored");
            return;
        }
        for (int i = 0; i < 3; ++i) {
            g(i) = std::max(-epsGyroMax_,  std::min(epsGyroMax_,  g(i)));
            a(i) = std::max(-epsAccelMax_, std::min(epsAccelMax_, a(i)));
        }
        tdeGyroTarget_  = g;
        tdeAccelTarget_ = useTdeAccelFb_ ? a : gtsam::Vector3::Zero();   // accel channel opt-in
        // ROS_INFO_THROTTLE(2.0, "[TDE-fb] target |eps_w|=%.3f rad/s  |eps_a|=%.3f m/s^2 (accel_fb=%d)",
        //                   g.norm(), a.norm(), (int)useTdeAccelFb_);
    }

    // First-order low-pass of the applied correction toward the target, stepped once per IMU
    // sample. alpha = dt/(tau+dt): for tau >> dt the correction moves only a little each sample,
    // so a LiDAR-rate target produces a smooth, continuous high-rate correction (no 10 Hz steps),
    // and the feedback is band-limited to ~1/(2*pi*tau) Hz (the slow, LiDAR-observable part).
    inline void slewTdeFeedback(double dt)
    {
        if (!useTdeFeedback_ || tdeTau_ <= 1e-6 || dt <= 0.0) {
            tdeGyroCorr_  = tdeGyroTarget_;
            tdeAccelCorr_ = tdeAccelTarget_;
            return;
        }
        const double alpha = dt / (tdeTau_ + dt);
        tdeGyroCorr_  += alpha * (tdeGyroTarget_  - tdeGyroCorr_);
        tdeAccelCorr_ += alpha * (tdeAccelTarget_ - tdeAccelCorr_);
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // smoothly slew the applied disturbance toward the latest ESKF target (no 10 Hz steps)
        slewTdeFeedback(dt);

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z) - tdeAccelCorr_,
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z)    - tdeGyroCorr_, dt);

        // Publish the bias- AND disturbance-corrected IMU (in the imuConverter'd / lidar-aligned
        // frame) so imageProjection can deskew with a clean continuous-time IMU model, CTI-style.
        // bias = latest factor-graph estimate (prevBiasOdom); disturbance = smoothed TDE feedback.
        {
            sensor_msgs::Imu imuCorr = thisImu;   // keep header, frame_id, orientation
            gtsam::Vector3 biasG = useObserverBias_ ? obsBiasGyro_  : prevBiasOdom.gyroscope();
            gtsam::Vector3 biasA = useObserverBias_ ? obsBiasAccel_ : prevBiasOdom.accelerometer();
            gtsam::Vector3 gCorr = gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z)
                                 - biasG - tdeGyroCorr_;
            gtsam::Vector3 aCorr = gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z)
                                 - biasA - tdeAccelCorr_;
            imuCorr.angular_velocity.x = gCorr.x(); imuCorr.angular_velocity.y = gCorr.y(); imuCorr.angular_velocity.z = gCorr.z();
            imuCorr.linear_acceleration.x = aCorr.x(); imuCorr.linear_acceleration.y = aCorr.y(); imuCorr.linear_acceleration.z = aCorr.z();
            pubImuCorrected.publish(imuCorr);
        }

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}