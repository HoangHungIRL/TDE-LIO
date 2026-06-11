#include "utility.h"
#include "tde_lio/cloud_info.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubDeskewedWorld;   // deskewed cloud transformed into world frame by the observer pose (CTI-style)
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;
    // bias+disturbance-corrected IMU from imuPreintegration (tde_lio/imu/corrected), used as the
    // preferred source for the CTI continuous-time deskew model.
    ros::Subscriber subImuCorr;
    std::deque<sensor_msgs::Imu> imuCorrQueue;
    std::mutex imuCorrLock;
    bool useCorrectedImu_ = true;   // tde_lio/deskewUseCorrectedImu

    // observer pose stream (tde_lio/observer/odometry) for world-frame deskewed cloud
    ros::Subscriber subObserverOdom;
    std::deque<nav_msgs::Odometry> obsOdomQueue;
    std::mutex obsOdomLock;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];
    // position samples for deskew, relative to scan-start, sourced from the smooth IMU odometry
    double *imuPosX = new double[queueLength];
    double *imuPosY = new double[queueLength];
    double *imuPosZ = new double[queueLength];
    bool useOdomDeskew_   = true;   // drive deskew from imuPreintegration odometry (tde_lio/deskewUseOdom)
    bool useOdomPosition_ = true;   // also deskew translation (tde_lio/deskewUsePosition)

    // ---- CTI-style continuous-time deskew ----
    bool useCtiDeskew_   = true;   // primary deskew engine (tde_lio/ctiDeskew); falls back otherwise
    struct ImuSample { double stamp; float dt; Eigen::Vector3f w; Eigen::Vector3f a; };
    std::vector<double>           ctiStamp;   // sorted unique absolute point times in the scan
    std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> ctiTrel;  // scan-start <- pose(t)
    bool                          ctiValid = false;
    bool                          ctiDeskewPosition_ = true;          // tde_lio/ctiDeskewPosition
    Eigen::Matrix3f               accelSM_ = Eigen::Matrix3f::Identity(); // CTI accel scale-misalignment

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    tde_lio::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    ImageProjection():
    deskewFlag(0)
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subImuCorr    = nh.subscribe<sensor_msgs::Imu>("tde_lio/imu/corrected", 2000, &ImageProjection::imuCorrHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        nh.param<bool>("tde_lio/deskewUseOdom",     useOdomDeskew_,   true);
        nh.param<bool>("tde_lio/deskewUsePosition", useOdomPosition_, true);
        nh.param<bool>("tde_lio/ctiDeskew",        useCtiDeskew_,   true);
        nh.param<bool>("tde_lio/deskewUseCorrectedImu", useCorrectedImu_, true);
        nh.param<bool>("tde_lio/ctiDeskewPosition",     ctiDeskewPosition_, true);
        std::vector<double> sm;
        if (nh.getParam("tde_lio/imuAccelSM", sm) && sm.size() == 9)
            accelSM_ = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(sm.data()).cast<float>();

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("tde_lio/deskew/cloud_deskewed", 1);
        pubDeskewedWorld  = nh.advertise<sensor_msgs::PointCloud2> ("tde_lio/deskew/cloud_deskewed_world", 1);
        subObserverOdom   = nh.subscribe<nav_msgs::Odometry>("tde_lio/observer/odometry", 200, &ImageProjection::obsOdomHandler, this, ros::TransportHints().tcpNoDelay());
        pubLaserCloudInfo = nh.advertise<tde_lio::cloud_info> ("tde_lio/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        ctiValid = false;
        ctiStamp.clear();
        ctiTrel.clear();

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
            imuPosX[i] = 0;
            imuPosY[i] = 0;
            imuPosZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void imuCorrHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        // Already imuConverter'd (lidar-aligned) and bias+disturbance-corrected upstream.
        std::lock_guard<std::mutex> lock(imuCorrLock);
        imuCorrQueue.push_back(*imuMsg);
        while (imuCorrQueue.size() > 3000) imuCorrQueue.pop_front();   // keep bounded
    }

    void obsOdomHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(obsOdomLock);
        obsOdomQueue.push_back(*odomMsg);
        while (obsOdomQueue.size() > 3000) obsOdomQueue.pop_front();
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        // Preferred: CTI-style continuous-time deskew. Sets ctiValid on success; deskewPoint
        // uses it when valid, otherwise it falls back to the imu/odom interpolation arrays above.
        if (useCtiDeskew_)
            buildCtiDeskew();

        return true;
    }

    // =====================================================================================
    //  CTI-style continuous-time motion compensation (ported from continuous-time
    //  Odometry). Between consecutive IMU samples it assumes constant angular acceleration
    //  and constant jerk, propagating orientation with quaternion kinematics and position
    //  with a cubic model, and interpolates a pose at EACH unique point timestamp. The math
    //  uses only Eigen (already a TDE-LIO dependency); CTI's boost circular_buffer/range
    //  plumbing is replaced by std containers, so no extra library is required.
    //
    //  Output is stored as a per-unique-timestamp transform expressed in the SCAN-START
    //  body frame (ctiTrel), matching TDE-LIO's "deskew to scan start" convention so the
    //  rest of the pipeline is unchanged.
    // =====================================================================================
    static inline Eigen::Quaternionf qPropagate(const Eigen::Quaternionf& q,
                                                 const Eigen::Vector3f& w, float dt)
    {
        Eigen::Quaternionf qn(
            q.w() - 0.5f*( q.x()*w[0] + q.y()*w[1] + q.z()*w[2] ) * dt,
            q.x() + 0.5f*( q.w()*w[0] - q.z()*w[1] + q.y()*w[2] ) * dt,
            q.y() + 0.5f*( q.z()*w[0] + q.w()*w[1] - q.x()*w[2] ) * dt,
            q.z() + 0.5f*( q.x()*w[1] - q.y()*w[0] + q.w()*w[2] ) * dt);
        qn.normalize();
        return qn;
    }

    // Look up the world state (orientation, position, velocity) from the smooth IMU odometry
    // at time t. Returns false if odometry doesn't bracket t (then position deskew is dropped).
    bool getOdomStateAt(double t, Eigen::Quaternionf& q, Eigen::Vector3f& p, Eigen::Vector3f& v)
    {
        if (odomQueue.empty()) return false;
        if (odomQueue.front().header.stamp.toSec() > t) return false;
        const nav_msgs::Odometry* best = nullptr;
        for (size_t i = 0; i < odomQueue.size(); ++i) {
            if (odomQueue[i].header.stamp.toSec() <= t) best = &odomQueue[i];
            else break;
        }
        if (!best) return false;
        q = Eigen::Quaternionf(best->pose.pose.orientation.w, best->pose.pose.orientation.x,
                               best->pose.pose.orientation.y, best->pose.pose.orientation.z);
        q.normalize();
        p = Eigen::Vector3f(best->pose.pose.position.x, best->pose.pose.position.y, best->pose.pose.position.z);
        v = Eigen::Vector3f(best->twist.twist.linear.x, best->twist.twist.linear.y, best->twist.twist.linear.z);
        return true;
    }

    // Fill S with IMU samples spanning [startTime, endTime]; returns true only if the queue
    // brackets the interval (a sample <= startTime and a sample >= endTime) with >= 3 samples.
    bool gatherImuSamples(const std::deque<sensor_msgs::Imu>& q, double startTime, double endTime,
                          std::vector<ImuSample>& S)
    {
        if (q.empty()) return false;
        if (q.front().header.stamp.toSec() > startTime) return false;  // no coverage before start
        if (q.back().header.stamp.toSec()  < endTime)   return false;  // no coverage after end
        S.reserve(q.size());
        double prevt = -1;
        for (size_t i = 0; i < q.size(); ++i) {
            const auto& m = q[i];
            double st = m.header.stamp.toSec();
            if (st < startTime - 0.05) { prevt = st; continue; }
            if (st > endTime + 0.05) break;
            ImuSample s;
            s.stamp = st;
            s.dt    = (prevt < 0) ? 0.0f : (float)(st - prevt);
            s.w     = Eigen::Vector3f(m.angular_velocity.x,    m.angular_velocity.y,    m.angular_velocity.z);
            s.a     = accelSM_ * Eigen::Vector3f(m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
            S.push_back(s);
            prevt = st;
        }
        return S.size() >= 3;
    }

    void buildCtiDeskew()
    {
        ctiValid = false;
        if (deskewFlag == -1 || laserCloudIn->points.empty()) return;

        // ---- 1. sorted, unique absolute point timestamps across the sweep ----
        std::vector<double> ts;
        ts.reserve(laserCloudIn->points.size());
        for (const auto& pt : laserCloudIn->points)
            ts.push_back(timeScanCur + pt.time);
        std::sort(ts.begin(), ts.end());
        ts.erase(std::unique(ts.begin(), ts.end()), ts.end());
        if (ts.size() < 2) return;

        // integrate from the scan start; prepend it so frame[0] is the scan-start pose
        double startTime = std::min(timeScanCur, ts.front());
        std::vector<double> stamps;
        stamps.reserve(ts.size() + 1);
        stamps.push_back(startTime);
        for (double t : ts) if (t > startTime) stamps.push_back(t);

        // ---- 2. IMU samples for the CTI model. Prefer the bias+disturbance-corrected stream
        //         from imuPreintegration; fall back to the raw imuConverter'd queue if it does
        //         not yet cover this scan (e.g. at startup before the first optimization). ----
        std::vector<ImuSample> S;
        double endTime = stamps.back();
        bool usedCorr = false;
        if (useCorrectedImu_) {
            std::lock_guard<std::mutex> lock(imuCorrLock);
            if (gatherImuSamples(imuCorrQueue, startTime, endTime, S))
                usedCorr = true;
        }
        if (!usedCorr) {
            S.clear();
            if (!gatherImuSamples(imuQueue, startTime, endTime, S)) {
                // ROS_WARN_THROTTLE(2.0, "[deskew] CTI fallback->euler: IMU does not cover the scan");
                return;
            }
        }
        // if (S.size() < 3) { ROS_WARN_THROTTLE(2.0, "[deskew] CTI fallback->euler: <3 IMU samples"); return; }

        // ---- 3. world state at the scan start (for gravity-correct position deskew) ----
        Eigen::Quaternionf q_init(1,0,0,0);
        Eigen::Vector3f    p_init(0,0,0), v_init(0,0,0);
        bool posValid = getOdomStateAt(startTime, q_init, p_init, v_init);
        // Rotation deskew is independent of q_init (it cancels in the scan-start-relative
        // transform), so it is always valid. Position deskew needs a real world attitude +
        // velocity, so when odometry is missing we keep rotation only.

        // ---- 4. CTI continuous-time integration -> world pose at each stamp ----
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames =
            integrateImuCti(startTime, q_init, p_init, v_init, S, stamps);
        if (frames.size() != stamps.size()) {
            // ROS_WARN_THROTTLE(2.0, "[deskew] CTI fallback->euler: integrator produced %zu/%zu poses (IMU/LiDAR time sync?)",
            //                   frames.size(), stamps.size());
            return;
        }

        bool doPos = posValid && ctiDeskewPosition_;

        // ---- 5. express relative to the scan-start frame (TDE-LIO convention) ----
        Eigen::Affine3f Tstart;  Tstart.matrix() = frames[0];
        Eigen::Affine3f TstartInv = Tstart.inverse();

        ctiStamp.clear(); ctiTrel.clear();
        ctiStamp.reserve(stamps.size()); ctiTrel.reserve(stamps.size());
        for (size_t i = 0; i < stamps.size(); ++i) {
            Eigen::Affine3f Tt;  Tt.matrix() = frames[i];
            Eigen::Affine3f rel = TstartInv * Tt;
            if (!doPos)                          // rotation-only
                rel.translation().setZero();
            ctiStamp.push_back(stamps[i]);
            ctiTrel.push_back(rel);
        }
        ctiValid = true;
        // ROS_INFO_THROTTLE(3.0, "[deskew] CTI active: src=%s, %zu stamps, position=%s",
        //                   usedCorr ? "corrected" : "raw", stamps.size(), doPos ? "on" : "off");
    }

    // Forward continuous-time integration (constant angular accel + constant jerk between
    // IMU samples), interpolating a world pose at every timestamp in `stamps`.
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuCti(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                     Eigen::Vector3f v_init, const std::vector<ImuSample>& S,
                     const std::vector<double>& stamps)
    {
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> out;
        if (stamps.empty() || S.size() < 3) return out;

        // index of the last IMU sample at or before start_time
        int b = -1;
        for (size_t i = 0; i < S.size(); ++i) { if (S[i].stamp <= start_time) b = (int)i; else break; }
        if (b < 0 || b + 1 >= (int)S.size()) return out;

        const float g = imuGravity;   // gravity magnitude [m/s^2]

        // ---- align the init state from start_time back to the first sample S[b] (CTI) ----
        const ImuSample& f1 = S[b];
        const ImuSample& f2 = S[b+1];
        float dt  = (f2.dt > 1e-6f) ? f2.dt : (float)(f2.stamp - f1.stamp);
        float idt = (float)(start_time - f1.stamp);
        Eigen::Vector3f alpha_dt = f2.w - f1.w;
        Eigen::Vector3f alpha    = alpha_dt / dt;
        Eigen::Vector3f omega_i  = -(f1.w + 0.5f*alpha*idt);
        q_init = qPropagate(q_init, omega_i, idt);

        Eigen::Vector3f omega = f1.w + 0.5f*alpha_dt;
        Eigen::Quaternionf q2 = qPropagate(q_init, omega, dt);

        Eigen::Vector3f a1 = q_init._transformVector(f1.a);  a1[2] -= g;
        Eigen::Vector3f a2 = q2._transformVector(f2.a);      a2[2] -= g;
        Eigen::Vector3f j  = (a2 - a1) / dt;
        v_init -= a1*idt + 0.5f*j*idt*idt;
        p_init -= v_init*idt + 0.5f*a1*idt*idt + (1.f/6.f)*j*idt*idt*idt;

        // ---- forward integration with per-timestamp interpolation ----
        Eigen::Quaternionf q = q_init;
        Eigen::Vector3f p = p_init, v = v_init;
        Eigen::Vector3f a = q._transformVector(f1.a);  a[2] -= g;

        size_t s_it = 0;
        for (int it = b + 1; it < (int)S.size(); ++it) {
            const ImuSample& f0 = S[it-1];
            const ImuSample& f  = S[it];
            float fdt = (f.dt > 1e-6f) ? f.dt : (float)(f.stamp - f0.stamp);
            if (fdt <= 1e-6f) fdt = 1e-3f;

            Eigen::Vector3f adt = f.w - f0.w;
            Eigen::Vector3f al  = adt / fdt;
            Eigen::Vector3f om  = f0.w + 0.5f*adt;
            Eigen::Quaternionf q0 = q;          // orientation at the START of this IMU interval
            q = qPropagate(q0, om, fdt);        // advance to the end (f)

            Eigen::Vector3f a0 = a;
            a = q._transformVector(f.a);  a[2] -= g;
            Eigen::Vector3f j_dt = a - a0;
            Eigen::Vector3f jj   = j_dt / fdt;

            while (s_it < stamps.size() && stamps[s_it] <= f.stamp) {
                float sidt = (float)(stamps[s_it] - f0.stamp);
                Eigen::Vector3f om_i = f0.w + 0.5f*al*sidt;
                // Anchor on q0 (start of interval), NOT q (end). This makes the interpolated
                // orientation continuous across IMU boundaries (q_i -> q at sidt=fdt) and
                // consistent with the f0-anchored position, removing the per-interval sawtooth
                // that CTI's literal formulation produces at low IMU rate / high jerk.
                Eigen::Quaternionf q_i = qPropagate(q0, om_i, sidt);
                Eigen::Vector3f p_i = p + v*sidt + 0.5f*a0*sidt*sidt + (1.f/6.f)*jj*sidt*sidt*sidt;

                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                T.block(0,0,3,3) = q_i.toRotationMatrix();
                T.block(0,3,3,1) = p_i;
                out.push_back(T);
                ++s_it;
            }

            p += v*fdt + 0.5f*a0*fdt*fdt + (1.f/6.f)*j_dt*fdt*fdt;
            v += a0*fdt + 0.5f*j_dt*fdt;

            if (s_it >= stamps.size()) break;
        }
        return out;
    }

    // nearest precomputed transform for an absolute point time (points share unique stamps)
    inline const Eigen::Affine3f& lookupCtiTrel(double t)
    {
        auto it = std::lower_bound(ctiStamp.begin(), ctiStamp.end(), t);
        size_t idx;
        if (it == ctiStamp.begin())      idx = 0;
        else if (it == ctiStamp.end())   idx = ctiStamp.size() - 1;
        else {
            size_t hi = it - ctiStamp.begin();
            idx = (t - ctiStamp[hi-1] <= ctiStamp[hi] - t) ? hi-1 : hi;
        }
        return ctiTrel[idx];
    }

    // Build an SE(3) pose from an odometry message (position + orientation).
    Eigen::Affine3f odomToAffine(const nav_msgs::Odometry& m)
    {
        tf::Quaternion q;
        tf::quaternionMsgToTF(m.pose.pose.orientation, q);
        double r, p, y;
        tf::Matrix3x3(q).getRPY(r, p, y);
        return pcl::getTransformation(m.pose.pose.position.x,
                                      m.pose.pose.position.y,
                                      m.pose.pose.position.z, r, p, y);
    }

    // Fill the per-point deskew arrays (rotation + translation) by sampling the SMOOTH,
    // disturbance-corrected IMU odometry across the scan, expressed relative to the
    // scan-start pose. Returns false if the odometry does not cover the scan, in which
    // case the raw-gyro arrays computed by imuDeskewInfoRawGyro() are kept as fallback.
    bool deskewFromOdometry()
    {
        if (odomQueue.empty()) return false;
        if (odomQueue.front().header.stamp.toSec() > timeScanCur) return false; // no coverage at start
        if (odomQueue.back().header.stamp.toSec()  < timeScanEnd) return false; // no coverage at end

        // reference frame = last odometry pose at or before the scan start
        Eigen::Affine3f transStartRef;
        bool haveStart = false;
        for (size_t i = 0; i < odomQueue.size(); ++i)
        {
            if (odomQueue[i].header.stamp.toSec() <= timeScanCur)
            {
                transStartRef = odomToAffine(odomQueue[i]);
                haveStart = true;
            }
            else break;
        }
        if (!haveStart) return false;
        Eigen::Affine3f transStartInv = transStartRef.inverse();

        // sample every odometry pose spanning the scan, relative to the start frame
        int cnt = 0;
        for (size_t i = 0; i < odomQueue.size() && cnt < queueLength; ++i)
        {
            double t = odomQueue[i].header.stamp.toSec();
            if (t < timeScanCur - 0.01) continue;
            if (t > timeScanEnd + 0.01) break;

            Eigen::Affine3f rel = transStartInv * odomToAffine(odomQueue[i]);
            float px, py, pz, rr, pp, yy;
            pcl::getTranslationAndEulerAngles(rel, px, py, pz, rr, pp, yy);

            imuTime[cnt] = t;
            imuRotX[cnt] = rr; imuRotY[cnt] = pp; imuRotZ[cnt] = yy;
            imuPosX[cnt] = useOdomPosition_ ? px : 0.0;
            imuPosY[cnt] = useOdomPosition_ ? py : 0.0;
            imuPosZ[cnt] = useOdomPosition_ ? pz : 0.0;
            ++cnt;
        }
        if (cnt < 2) return false;

        imuPointerCur = cnt - 1;        // valid indices 0..imuPointerCur, consistent with findRotation
        cloudInfo.imuAvailable = true;
        return true;
    }

    void imuDeskewInfo()
    {
        // (1) Raw-gyro path first: it sets the attitude prior (imuRollInit/Pitch/Yaw) and
        //     leaves a valid fallback in the arrays if odometry doesn't cover this scan.
        imuDeskewInfoRawGyro();
        // (2) Preferred: overwrite rotation+position arrays with the smooth, disturbance-corrected
        //     motion from imuPreintegration. On failure the raw-gyro arrays are kept.
        if (useOdomDeskew_)
            deskewFromOdometry();
    }

    void imuDeskewInfoRawGyro()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double pointTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // Position deskew is driven by the smooth IMU-odometry samples (imuPosX/Y/Z), filled
        // by deskewFromOdometry(). When that path didn't run (raw-gyro fallback) these arrays
        // are zero, so this gracefully degrades to rotation-only deskew (original behaviour).
        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *posXCur = imuPosX[imuPointerFront];
            *posYCur = imuPosY[imuPointerFront];
            *posZCur = imuPosZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack  = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *posXCur = imuPosX[imuPointerFront] * ratioFront + imuPosX[imuPointerBack] * ratioBack;
            *posYCur = imuPosY[imuPointerFront] * ratioFront + imuPosY[imuPointerBack] * ratioBack;
            *posZCur = imuPosZ[imuPointerFront] * ratioFront + imuPosZ[imuPointerBack] * ratioBack;
        }
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        // ---- preferred: CTI continuous-time transform (scan-start frame) ----
        if (ctiValid && !ctiTrel.empty())
        {
            const Eigen::Affine3f& transBt = lookupCtiTrel(pointTime);
            PointType newPoint;
            newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
            newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
            newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
            newPoint.intensity = point->intensity;
            return newPoint;
        }

        // ---- fallback: imu/odom euler interpolation ----
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(pointTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
            {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (sensor == SensorType::LIVOX)
            {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }
            
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }
    
    // Transform the deskewed cloud (scan-start lidar frame) into the world/odom frame using the
    // smooth geometric-observer pose at the scan time, and publish it (CTI-style world cloud).
    void publishDeskewedWorld()
    {
        if (pubDeskewedWorld.getNumSubscribers() == 0 || extractedCloud->empty())
            return;

        // observer pose (lidar->odom) nearest the scan time
        const nav_msgs::Odometry* best = nullptr;
        {
            std::lock_guard<std::mutex> lock(obsOdomLock);
            if (obsOdomQueue.empty()) return;
            double bestdt = 1e9;
            for (size_t i = 0; i < obsOdomQueue.size(); ++i) {
                double d = std::fabs(obsOdomQueue[i].header.stamp.toSec() - timeScanCur);
                if (d < bestdt) { bestdt = d; best = &obsOdomQueue[i]; }
            }
            if (!best || bestdt > 0.5) return;   // no fresh observer pose
            static nav_msgs::Odometry hold;
            hold = *best; best = &hold;          // copy out before releasing the lock
        }

        Eigen::Quaternionf q(best->pose.pose.orientation.w, best->pose.pose.orientation.x,
                             best->pose.pose.orientation.y, best->pose.pose.orientation.z);
        Eigen::Vector3f    p(best->pose.pose.position.x, best->pose.pose.position.y, best->pose.pose.position.z);
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        T.linear() = q.normalized().toRotationMatrix();
        T.translation() = p;

        pcl::PointCloud<PointType>::Ptr worldCloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*extractedCloud, *worldCloud, T);
        publishCloud(pubDeskewedWorld, worldCloud, cloudHeader.stamp, odometryFrame);
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
        publishDeskewedWorld();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tde_lio");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}