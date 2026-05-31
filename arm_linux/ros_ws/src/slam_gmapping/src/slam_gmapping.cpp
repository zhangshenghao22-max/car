/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */

//
// Created by shivesh on 29/10/18.
//

#include "slam_gmapping/slam_gmapping.h"

#include "tf2_ros/create_timer_ros.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using std::placeholders::_1;

SlamGmapping::SlamGmapping():
    Node("slam_gmapping"),
    scan_filter_sub_(nullptr),
    scan_filter_(nullptr),
    laser_count_(0),
    transform_thread_(nullptr)
{
    // Create TF buffer with increased cache time (30 seconds) to handle message timestamp delays
    // Default is 10 seconds, but messages with slight delays need more history
    buffer_ = std::make_shared<tf2_ros::Buffer>(
        get_clock(),
        tf2::durationFromSec(30.0)  // 30 second cache time
    );
     auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    map_to_odom_.setIdentity();
    seed_ = static_cast<unsigned long>(time(nullptr));
    init();
    startLiveSlam();
}

void SlamGmapping::init() {
    gsp_ = new GMapping::GridSlamProcessor();

    gsp_laser_ = nullptr;
    gsp_odom_ = nullptr;
    got_first_scan_ = false;
    got_map_ = false;
    
    // Initialize map boundary expansion validation
    has_last_robot_pose_ = false;
    last_robot_pose_ = GMapping::OrientedPoint(0, 0, 0);
    // Minimum linear movement (meters) required before allowing boundary expansion
    // This prevents boundary expansion when robot only rotates near boundary
    min_linear_movement_for_expansion_ = 0.1;  // 10cm minimum movement
    // Maximum boundary expansion per update (meters) to prevent rapid expansion errors
    max_expansion_per_update_ = 0.5;  // 50cm maximum expansion per update
    
    // Clear any previous map data to ensure clean start
    map_ = nav_msgs::msg::OccupancyGrid();

    // Declare parameters with defaults
    this->declare_parameter<int>("throttle_scans", 1);
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<double>("transform_publish_period", 0.05);
    // Startup delay to wait for sensor data (laser and IMU) to stabilize before starting SLAM
    // This prevents TF drift issues at startup that can cause map edge anomalies
    this->declare_parameter<double>("startup_delay", 1.5);  // Default 1.5 seconds delay

    // Get parameters
    throttle_scans_ = this->get_parameter("throttle_scans").as_int();
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    transform_publish_period_ = this->get_parameter("transform_publish_period").as_double();
    startup_delay_ = this->get_parameter("startup_delay").as_double();
    
    // Record startup time for delay check
    startup_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Startup delay: %.2f seconds (waiting for sensor data to stabilize)", startup_delay_);

    RCLCPP_INFO(this->get_logger(), "SLAM Configuration:");
    RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  scan_topic: /scan (from lslidar_driver)");

    // Reduce map update interval for faster boundary updates (0.2 seconds instead of 0.5)
    // This allows multiple scans to cover and refine map edges, helping to fix anomalies
    // caused by initial TF drift
    map_update_interval_ = tf2::durationFromSec(0.2);
    maxUrange_ = 80.0;  maxRange_ = 0.0;
    minimum_score_ = 0;
    sigma_ = 0.05;
    kernelSize_ = 1;
    lstep_ = 0.05;
    astep_ = 0.05;
    iterations_ = 5;
    lsigma_ = 0.075;
    ogain_ = 3.0;
    lskip_ = 0;
    srr_ = 0.1;
    srt_ = 0.2;
    str_ = 0.1;
    stt_ = 0.2;
    // Update thresholds: lower values mean more frequent map updates
    // This helps refine map edges through multiple scans, fixing anomalies from initial TF drift
    linearUpdate_ = 0.5;   // Reduced from 1.0 to 0.5m for more frequent updates
    angularUpdate_ = 0.25; // Reduced from 0.5 to 0.25rad (~14°) for more frequent updates
    temporalUpdate_ = 1.0;
    resampleThreshold_ = 0.5;
    particles_ = 30;
    xmin_ = -10.0;
    ymin_ = -10.0;
    xmax_ = 10.0;
    ymax_ = 10.0;
    delta_ = 0.05;
    occ_thresh_ = 0.25;
    llsamplerange_ = 0.01;
    llsamplestep_ = 0.01;
    lasamplerange_ = 0.005;
    lasamplestep_ = 0.005;
    tf_delay_ = transform_publish_period_;
}

void SlamGmapping::startLiveSlam() {
    // Get scan topic from parameter
    std::string scan_topic = this->get_parameter("scan_topic").as_string();
    
    // Use transient_local QoS for map publishers to ensure new subscribers receive the latest map
    // This prevents old map data from previous runs from being displayed in RViz
    rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    
    entropy_publisher_ = this->create_publisher<std_msgs::msg::Float64>("entropy", rclcpp::SystemDefaultsQoS());
    sst_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
    sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", map_qos);
    scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
            (node_, scan_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
//    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//        "scan", rclcpp::SensorDataQoS(),
//        std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
            (*scan_filter_sub_, *buffer_, odom_frame_, 10, node_);
    scan_filter_->registerCallback(std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    transform_thread_ = std::make_shared<std::thread>
            (std::bind(&SlamGmapping::publishLoop, this, transform_publish_period_));
}

void SlamGmapping::publishLoop(double transform_publish_period){
    if (transform_publish_period == 0)
        return;
    rclcpp::Rate r(1.0 / transform_publish_period);
    while (rclcpp::ok()) {
        publishTransform();
        r.sleep();
    }
}

SlamGmapping::~SlamGmapping()
{
    if(transform_thread_){
        transform_thread_->join();
    }

    delete gsp_;
    delete gsp_laser_;
    delete gsp_odom_;
}

bool SlamGmapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
    // Get the pose of the centered laser at the right time
    centered_laser_pose_.header.stamp = t;
    // Get the laser's pose that is centered
    geometry_msgs::msg::PoseStamped odom_pose;
    try
    {
        buffer_->transform(centered_laser_pose_, odom_pose, odom_frame_, tf2::durationFromSec(1.0));
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }

    double yaw = tf2::getYaw(odom_pose.pose.orientation);

    gmap_pose = GMapping::OrientedPoint(odom_pose.pose.position.x,
                                        odom_pose.pose.position.y,
                                        yaw);
    return true;
}

bool SlamGmapping::initMapper(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    laser_frame_ = scan->header.frame_id;
    // Get the laser's pose, relative to base.
    geometry_msgs::msg::PoseStamped ident;
    geometry_msgs::msg::PoseStamped laser_pose;

    try{
        ident.header.frame_id = laser_frame_;
        ident.header.stamp = scan->header.stamp;
        tf2::Transform transform;
        transform.setIdentity();
        tf2::toMsg(transform, ident.pose);
        buffer_->transform(ident, laser_pose, base_frame_);
    }
    catch (tf2::TransformException& e){
        RCLCPP_WARN(this->get_logger(), "Failed to compute laser pose, aborting initialization (%s)", e.what());
        return false;
    }

    // create a point 1m above the laser position and transform it into the laser-frame
    geometry_msgs::msg::PointStamped up;
    up.header.stamp = scan->header.stamp;
    up.header.frame_id = base_frame_;
    up.point.x = up.point.y = 0;
    up.point.z = 1 + laser_pose.pose.position.z;
    try
    {
        buffer_->transform(up, up, laser_frame_);
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
    if (fabs(fabs(up.point.z) - 1) > 0.001)
    {
        RCLCPP_INFO(this->get_logger(),
                "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.point.z);
        return false;
    }

    gsp_laser_beam_count_ = static_cast<unsigned int>(scan->ranges.size());

    double angle_center = (scan->angle_min + scan->angle_max)/2;

    centered_laser_pose_.header.frame_id = laser_frame_;
    centered_laser_pose_.header.stamp = get_clock()->now();
    tf2::Quaternion q;

    if (up.point.z > 0)
    {
        do_reverse_range_ = scan->angle_min > scan->angle_max;
        q.setEuler(angle_center, 0, 0);
        RCLCPP_INFO(this->get_logger(),"Laser is mounted upwards.");
    }
    else
    {
        do_reverse_range_ = scan->angle_min < scan->angle_max;
        q.setEuler(-angle_center, 0, M_PI);
        RCLCPP_INFO(this->get_logger(), "Laser is mounted upside down.");
    }

    centered_laser_pose_.pose.position.x = 0;
    centered_laser_pose_.pose.position.y = 0;
    centered_laser_pose_.pose.position.z = 0;

    centered_laser_pose_.pose.orientation.w = q.getW();
    centered_laser_pose_.pose.orientation.x = q.getX();
    centered_laser_pose_.pose.orientation.y = q.getY();
    centered_laser_pose_.pose.orientation.z = q.getZ();

    // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
    laser_angles_.resize(scan->ranges.size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan->angle_min - scan->angle_max)/2;
    for(unsigned int i=0; i<scan->ranges.size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan->angle_increment);
    }

    RCLCPP_DEBUG(this->get_logger(), "Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f",
            scan->angle_min, scan->angle_max, scan->angle_increment);
    RCLCPP_DEBUG(this->get_logger(), "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f",
            laser_angles_.front(), laser_angles_.back(), std::fabs(scan->angle_increment));

    GMapping::OrientedPoint gmap_pose(0, 0, 0);

    // setting maxRange and maxUrange here so we can set a reasonable default
    maxRange_ = scan->range_max - 0.01;
    maxUrange_ = maxRange_;

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(scan->angle_increment), gmap_pose,
                                           0.0, maxRange_);

    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);

    gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);

    /// @todo Expose setting an initial pose
    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan->header.stamp))
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, static_cast<unsigned int>(lskip_));

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(static_cast<unsigned int>(particles_), xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1, static_cast<unsigned int>(seed_));

    RCLCPP_INFO(this->get_logger(), "Initialization complete");

    return true;
}

bool SlamGmapping::addScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, GMapping::OrientedPoint& gmap_pose) {
    if (!getOdomPose(gmap_pose, scan->header.stamp))
        return false;

    if (scan->ranges.size() != gsp_laser_beam_count_)
        return false;

    // GMapping wants an array of doubles...
    auto *ranges_double = new double[scan->ranges.size()];
    // If the angle increment is negative, we have to invert the order of the readings.
    if (do_reverse_range_) {
        RCLCPP_DEBUG(this->get_logger(), "Inverting scan");
        int num_ranges = static_cast<int>(scan->ranges.size());
        for (int i = 0; i < num_ranges; i++) {
            // Must filter out short readings, because the mapper won't
            if (scan->ranges[num_ranges - i - 1] < scan->range_min)
                ranges_double[i] = (double) scan->range_max;
            else
                ranges_double[i] = (double) scan->ranges[num_ranges - i - 1];
        }
    } else {
        for (unsigned int i = 0; i < scan->ranges.size(); i++) {
            // Must filter out short readings, because the mapper won't
            if (scan->ranges[i] < scan->range_min)
                ranges_double[i] = (double) scan->range_max;
            else
                ranges_double[i] = (double) scan->ranges[i];
        }
    }

    GMapping::RangeReading reading(static_cast<unsigned int>(scan->ranges.size()),
                                   ranges_double,
                                   gsp_laser_,
                                   scan->header.stamp.sec);

    // ...but it deep copies them in RangeReading constructor, so we don't
    // need to keep our array around.
    delete[] ranges_double;

    reading.setPose(gmap_pose);

    RCLCPP_DEBUG(this->get_logger(), "processing scan");

    return gsp_->processScan(reading);
}


void SlamGmapping::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;

    // Check startup delay: wait for sensor data (laser and IMU) to stabilize before processing
    // This prevents TF drift issues at startup that can cause map edge anomalies
    rclcpp::Time now = this->get_clock()->now();
    double elapsed = (now - startup_time_).seconds();
    if (elapsed < startup_delay_) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for sensor data to stabilize (%.2f/%.2f seconds)...", 
                             elapsed, startup_delay_);
        return;  // Skip processing until delay period has passed
    }
    
    // Log once when delay period ends
    static bool delay_logged = false;
    if (!delay_logged) {
        RCLCPP_INFO(this->get_logger(), "Startup delay complete, starting SLAM processing");
        delay_logged = true;
    }

    tf2::TimePoint last_map_update = tf2::TimePointZero;

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
        if(!initMapper(scan))
            return;
        got_first_scan_ = true;
    }

    GMapping::OrientedPoint odom_pose;

    if(addScan(scan, odom_pose))
    {
        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        tf2::Quaternion q;
        q.setRPY(0, 0, mpose.theta);
        tf2::Transform laser_to_map = tf2::Transform(q, tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        q.setRPY(0, 0, odom_pose.theta);
        tf2::Transform odom_to_laser = tf2::Transform(q, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        tf2::TimePoint timestamp = tf2_ros::fromMsg(scan->header.stamp);
        if(!got_map_ || (timestamp - last_map_update) > map_update_interval_)
        {
            updateMap(scan);
            last_map_update = tf2_ros::fromMsg(scan->header.stamp);
        }
    }
}

double SlamGmapping::computePoseEntropy()
{
    double weight_total=0.0;
    for (const auto &it : gsp_->getParticles()) {
        weight_total += it.weight;
    }
    double entropy = 0.0;
    for (const auto &it : gsp_->getParticles()) {
        if(it.weight/weight_total > 0.0)
            entropy += it.weight/weight_total * log(it.weight/weight_total);
    }
    return -entropy;
}

void SlamGmapping::updateMap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    RCLCPP_DEBUG(this->get_logger(), "Update map");
    map_mutex_.lock();
    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(static_cast<unsigned int>(scan->ranges.size()), &(laser_angles_[0]),
                               gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];
    std_msgs::msg::Float64 entropy;
    entropy.data = computePoseEntropy();
    if(entropy.data > 0.0)
        entropy_publisher_->publish(entropy);

    if(!got_map_) {
        map_.info.resolution = static_cast<nav_msgs::msg::MapMetaData::_resolution_type>(delta_);
        map_.info.origin.position.x = 0.0;
        map_.info.origin.position.y = 0.0;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.x = 0.0;
        map_.info.origin.orientation.y = 0.0;
        map_.info.origin.orientation.z = 0.0;
        map_.info.origin.orientation.w = 1.0;
    }

    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    RCLCPP_DEBUG(this->get_logger(), "Trajectory tree:");
    for(GMapping::GridSlamProcessor::TNode* n = best.node;
        n;
        n = n->parent)
    {
        RCLCPP_DEBUG(this->get_logger(), "  %.3f %.3f %.3f",
                  n->pose.x,
                  n->pose.y,
                  n->pose.theta);
        if(!n->reading)
        {
            RCLCPP_DEBUG(this->get_logger(), "Reading is NULL");
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    // the map may have expanded, so resize ros message as well
    if(map_.info.width != (unsigned int) smap.getMapSizeX() || map_.info.height != (unsigned int) smap.getMapSizeY()) {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        
        // Calculate proposed new boundaries
        double new_xmin = wmin.x;
        double new_ymin = wmin.y;
        double new_xmax = wmax.x;
        double new_ymax = wmax.y;
        
        // Calculate boundary expansion amounts
        double xmin_expansion = (xmin_ - new_xmin);  // Positive if expanding left
        double ymin_expansion = (ymin_ - new_ymin);  // Positive if expanding down
        double xmax_expansion = (new_xmax - xmax_);  // Positive if expanding right
        double ymax_expansion = (new_ymax - ymax_);  // Positive if expanding up
        
        // Get current robot pose from best particle
        GMapping::OrientedPoint current_robot_pose = best.pose;
        
        // Validate and potentially delay boundary expansion if robot only rotated near boundary
        bool should_update_boundaries = true;
        if (has_last_robot_pose_) {
            // Calculate linear movement since last update
            double dx = current_robot_pose.x - last_robot_pose_.x;
            double dy = current_robot_pose.y - last_robot_pose_.y;
            double linear_movement = std::sqrt(dx * dx + dy * dy);
            
            // If robot only rotated (small linear movement) and we're near boundary, be cautious
            if (linear_movement < min_linear_movement_for_expansion_) {
                // Check if we're near a boundary that's trying to expand
                double robot_to_boundary_dist = std::min({
                    std::abs(current_robot_pose.x - xmin_),
                    std::abs(current_robot_pose.x - xmax_),
                    std::abs(current_robot_pose.y - ymin_),
                    std::abs(current_robot_pose.y - ymax_)
                });
                
                // If robot is within 1m of boundary and only rotated, check expansion amount
                if (robot_to_boundary_dist < 1.0) {
                    // Check if expansion is too large (likely error from rotation)
                    double max_expansion = max_expansion_per_update_;
                    if (std::abs(xmin_expansion) > max_expansion || 
                        std::abs(ymin_expansion) > max_expansion ||
                        std::abs(xmax_expansion) > max_expansion || 
                        std::abs(ymax_expansion) > max_expansion) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                            "Delaying boundary expansion near edge (robot only rotated, linear_movement=%.3f m, dist_to_boundary=%.3f m, max_expansion=%.3f m)",
                                            linear_movement, robot_to_boundary_dist, max_expansion);
                        // Delay boundary update: keep current boundaries, but allow smap to expand internally
                        // This allows GMapping to process the scan, but prevents premature boundary expansion
                        // The boundaries will be updated in the next cycle if robot actually moves
                        should_update_boundaries = false;
                    }
                }
            }
        }
        
        // Update boundaries only if validation passed
        if (should_update_boundaries) {
            xmin_ = new_xmin;
            ymin_ = new_ymin;
            xmax_ = new_xmax;
            ymax_ = new_ymax;
        } else {
            // Keep current boundaries, but smap may have expanded internally
            // IMPORTANT: We need to ensure map data consistency
            // If smap expanded but we're keeping old boundaries, we need to extract
            // only the data within our current boundaries from smap
            // For now, we'll allow smap to expand but use its actual boundaries for ROS map
            // This ensures data consistency, but the limitation will be applied gradually
            // Get smap's actual boundaries (may be larger than our controlled boundaries)
            GMapping::Point smap_wmin = smap.map2world(GMapping::IntPoint(0, 0));
            GMapping::Point smap_wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
            
            // Use smap's boundaries for this update to ensure data consistency
            // The boundary limitation will be applied in the next update cycle if robot actually moves
            xmin_ = smap_wmin.x;
            ymin_ = smap_wmin.y;
            xmax_ = smap_wmax.x;
            ymax_ = smap_wmax.y;
            
            RCLCPP_DEBUG(this->get_logger(), "Boundary expansion delayed, but using smap's expanded boundaries for data consistency. "
                          "Controlled boundaries will be applied in next cycle if robot moves.");
        }
        
        // Update last robot pose for next validation
        last_robot_pose_ = current_robot_pose;
        has_last_robot_pose_ = true;

        RCLCPP_DEBUG(this->get_logger(), "map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                  xmin_, ymin_, xmax_, ymax_);

        // Use smap's actual size and our controlled boundaries for ROS map
        // This ensures data consistency between smap and ROS map
        map_.info.width = static_cast<nav_msgs::msg::MapMetaData::_width_type>(smap.getMapSizeX());
        map_.info.height = static_cast<nav_msgs::msg::MapMetaData::_height_type>(smap.getMapSizeY());
        map_.info.origin.position.x = xmin_;
        map_.info.origin.position.y = ymin_;
        map_.data.resize(map_.info.width * map_.info.height);

        RCLCPP_DEBUG(this->get_logger(), "map origin: (%f, %f)", map_.info.origin.position.x, map_.info.origin.position.y);
    } else {
        // Even if map size didn't change, update last robot pose for validation
        last_robot_pose_ = best.pose;
        has_last_robot_pose_ = true;
    }

    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ=smap.cell(p);
            assert(occ <= 1.0);
            if(occ < 0)
                map_.data[MAP_IDX(map_.info.width, x, y)] = -1;
            else if(occ > occ_thresh_)
            {
                //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                map_.data[MAP_IDX(map_.info.width, x, y)] = 100;
            }
            else
                map_.data[MAP_IDX(map_.info.width, x, y)] = 0;
        }
    }
    got_map_ = true;

    //make sure to set the header information on the map
    map_.header.stamp = get_clock()->now();
    map_.header.frame_id = map_frame_;

    sst_->publish(map_);
    sstm_->publish(map_.info);
    map_mutex_.unlock();
}

void SlamGmapping::publishTransform()
{
    map_to_odom_mutex_.lock();
    rclcpp::Time tf_expiration = get_clock()->now() + rclcpp::Duration(
            static_cast<int32_t>(static_cast<rcl_duration_value_t>(tf_delay_)), 0);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.header.stamp = tf_expiration;
    transform.child_frame_id = odom_frame_;
    try {
        transform.transform = tf2::toMsg(map_to_odom_);
        tfB_->sendTransform(transform);
    }
    catch (tf2::LookupException& te){
        RCLCPP_INFO(this->get_logger(), te.what());
    }
    map_to_odom_mutex_.unlock();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto slam_gmapping_node = std::make_shared<SlamGmapping>();
    rclcpp::spin(slam_gmapping_node);
    return(0);
}
