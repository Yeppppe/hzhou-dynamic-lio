#pragma once
// c++
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>

// ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

// eigen 
#include <Eigen/Core>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>	
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include "cloudMap.h"

// cloud processing
#include "cloudProcessing.h"

// IMU processing
#include "state.h"

// eskf estimator
#include "eskfEstimator.h"

// utility
#include "utility.h"
#include "parameters.h"

class cloudFrame
{
public:
    double time_sweep_begin, time_sweep_end;

    int id;   // the index in all_cloud_frame
    int frame_id;

    state *p_state;

    std::vector<point3D> point_frame;

    double offset_begin;
    double offset_end;
    double dt_offset;

    bool success;

    cloudFrame(std::vector<point3D> &point_frame_, state *p_state_);

    cloudFrame(cloudFrame *p_cloud_frame);

    void release();
};

struct Neighborhood {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d center = Eigen::Vector3d::Zero();

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

    double a2D = 1.0; // Planarity coefficient
};

struct ResidualBlock {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d point_closest;

    Eigen::Vector3d pt_imu;

    Eigen::Vector3d normal;

    double alpha_time;

    double weight;
};

class estimationSummary {

public:

    int sample_size = 0; // The number of points sampled

    int number_of_residuals = 0; // The number of keypoints used for ICP registration

    int robust_level = 0;

    double distance_correction = 0.0; // The correction between the last frame's end, and the new frame's beginning

    double relative_distance = 0.0; // The distance between the beginning of the new frame and the end

    double relative_orientation = 0.0; // The distance between the beginning of the new frame and the end

    double ego_orientation = 0.0; // The angular distance between the beginning and the end of the frame

    bool success = true; // Whether the registration was a success

    std::string error_message;

    estimationSummary();

    void release();

};

struct optimizeSummary {

    bool success = false; // Whether the registration succeeded

    int num_residuals_used = 0;

    std::string error_log;
};

class lioOptimization{

private:

	ros::NodeHandle nh;  // ROS节点句柄，用于创建发布者和订阅者

	ros::Publisher pub_cloud_body;     // 发布器：发布当前激光雷达扫描数据，用于可视化
    ros::Publisher pub_cloud_world;    // 发布器：发布全局地图点云，用于可视化
    ros::Publisher pub_odom;		   // 发布器：发布LIO优化后的位姿信息
    ros::Publisher pub_path;		   // 发布器：发布优化后的路径，用于可视化

    ros::Subscriber sub_cloud_ori;     // 订阅器：订阅原始激光雷达点云数据
    ros::Subscriber sub_imu_ori;	   // 订阅器：订阅原始IMU数据（加速度计和陀螺仪）

    std::string lidar_topic;           // 激光雷达数据话题名称
    std::string imu_topic;             // IMU数据话题名称

	cloudProcessing *cloud_pro;        // 点云处理器，处理激光雷达原始数据
    eskfEstimator *eskf_pro;           // 误差状态卡尔曼滤波器，用于IMU数据融合和状态估计

    bool extrin_enable;                // 是否启用外参标定

    double laser_point_cov;            // 激光点云协方差

    std::vector<double> v_G;           // 重力向量参数
    std::vector<double> v_extrin_t;    // IMU与激光雷达之间的外参平移向量
    std::vector<double> v_extrin_R;    // IMU与激光雷达之间的外参旋转矩阵

    Eigen::Matrix3d R_imu_lidar;       // IMU到激光雷达的旋转矩阵
    Eigen::Vector3d t_imu_lidar;       // IMU到激光雷达的平移向量

    Eigen::Vector3d pose_lid;          // 激光雷达位姿

    std::queue<std::pair<double, double>> time_buffer;  // 时间戳缓冲区
    std::queue<std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> feature_buffer;  // 特征点云缓冲区

    std::queue<std::vector<point3D>> lidar_buffer;      // 激光雷达数据缓冲区
    std::queue<sensor_msgs::Imu::ConstPtr> imu_buffer;  // IMU数据缓冲区

    std::vector<cloudFrame*> all_cloud_frame;           // 所有点云帧

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr down_cloud_body;   // 降采样后的局部点云
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr down_cloud_world;  // 降采样后的全局点云

    std::vector<std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>>>  nearest_points;  // 最近邻点存储

	double last_time_lidar;            // 上一帧激光雷达数据时间戳
	double last_time_imu;              // 上一帧IMU数据时间戳
    double last_time_frame;            // 上一帧点云时间戳
    double current_time;               // 当前时间

    int index_frame;                   // 当前帧索引

    double time_max_solver;            // 求解器最大运行时间
    int num_max_iteration;             // 最大迭代次数

    // lidar range
    float det_range;                   // 激光雷达检测范围
    double fov_deg;                    // 激光雷达视场角度

    voxelHashMap voxel_map;            // 体素哈希地图，用于存储环境点云

    voxelHashMap voxel_map_true;       // 真实体素地图（过滤动态物体后）
    std::vector<point3D> points_undecided;  // 未确定是否为动态点的点集
    std::vector<globalPoint*> points_vec_true;  // 确认为静态点的点集

    odometryOptions options;           // 里程计参数选项

    geometry_msgs::Quaternion geoQuat;        // 几何四元数，用于位姿表示
    geometry_msgs::PoseStamped msg_body_pose; // 当前位姿消息
    nav_msgs::Path path;                      // 路径消息
    nav_msgs::Odometry odomAftMapped;         // 优化后的里程计消息

    double dt_sum;                            // 时间增量累加

    std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_meas;  // IMU测量值，存储格式：(时间戳, (角速度, 加速度))
    std::vector<imuState> imu_states;         // IMU状态序列

    // loop closing
    ros::Publisher pub_loop_map;              // 发布回环地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr loop_map;  // 回环检测地图点云

    void pubLocalMap(cloudFrame* p_frame);    // 发布局部地图函数，用于回环检测
    // loop closing

public:

    // initialize class
	lioOptimization();                        // 构造函数，初始化LIO系统

    void readParameters();                    // 读取ROS参数服务器中的参数

    void allocateMemory();                    // 分配内存空间

    void initialValue();                      // 初始化系统参数
    // initialize class

    // get sensor data
    void standardCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);  // 处理激光雷达点云数据回调函数

	void imuHandler(const sensor_msgs::Imu::ConstPtr &msg);                   // 处理IMU数据回调函数
    // get sensor data

    // main loop
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<point3D>>, std::pair<double, double>>> getMeasurements();  // 获取传感器测量数据

    void process(std::vector<point3D> &const_frame, double timestamp_begin, double timestamp_offset);  // 处理一帧点云数据

	void run();                              // 主循环函数，驱动整个系统
    // main loop

    // data handle and state estimation
    cloudFrame* buildFrame(std::vector<point3D> &const_frame, state *cur_state, double timestamp_begin, double timestamp_offset);  // 构建点云帧

    void makePointTimestamp(std::vector<point3D> &sweep, double time_sweep_begin, double time_sweep_end);  // 为点云中的每个点分配时间戳

    void stateInitialization(state *cur_state);  // 状态初始化

    optimizeSummary stateEstimation(cloudFrame *p_frame);  // 状态估计主函数

    optimizeSummary optimize(cloudFrame *p_frame, const icpOptions &cur_icp_options, double sample_voxel_size);  // 优化函数，基于ICP

    optimizeSummary buildPlaneResiduals(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, std::vector<planeParam> &plane_residuals, cloudFrame *p_frame, double &loss_sum);  // 构建平面约束残差

    optimizeSummary updateIEKF(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame);  // 更新迭代扩展卡尔曼滤波器

    Neighborhood computeNeighborhoodDistribution(std::vector<globalPoint> &points);  // 计算邻域分布

    std::vector<globalPoint> searchNeighbors(voxelHashMap &map, const Eigen::Vector3d &point,
        int nb_voxels_visited, double size_voxel_map, int max_num_neighbors, int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);  // 搜索近邻点
    // data handle and state estimation

    // map update
    void addPointToMap(voxelHashMap &map, point3D &point3d, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points, cloudFrame* p_frame);  // 向地图中添加单个点

    void addPointsToMap(voxelHashMap &map, cloudFrame* p_frame, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points = 0);  // 向地图中添加多个点

    void removePointsFarFromLocation(voxelHashMap &map, const Eigen::Vector3d &location, double distance);  // 移除远离指定位置的点

    size_t mapSize(const voxelHashMap &map);  // 获取地图大小
    // map update

    // save result to device
    void recordSinglePose(cloudFrame *p_frame);  // 记录单帧位姿到文件
    // save result to device

    // publish result by ROS for visualization
    void publish_path(ros::Publisher pub_path,cloudFrame *p_frame);  // 发布路径

    void set_posestamp(geometry_msgs::PoseStamped &body_pose_out,cloudFrame *p_frame);  // 设置位姿消息

    void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, globalPoint& point);  // 添加点到PCL点云
    void publishCLoudWorld(ros::Publisher & pub_cloud_world, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes, cloudFrame* p_frame);  // 发布全局地图点云

    pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;  // 全局地图点云
    void publish_odometry(const ros::Publisher & pubOdomAftMapped, cloudFrame *p_frame);  // 发布里程计消息
    // publish result by ROS for visualization

    // remove dynamic points
    void addPointToTrueMap(voxelHashMap &map, point3D &point3d, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points);  // 向真实地图添加点（过滤动态点）

    void decidePoints(const icpOptions &cur_icp_options, cloudFrame *p_frame);  // 决定点是静态还是动态

    void addDecidedPointsToTrueMap(double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points = 0);  // 将确认后的点添加到真实地图

    void detectDynamicLabel(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, cloudFrame *p_frame);  // 检测动态物体标签
    // remove dynamic points

    tf::TransformBroadcaster tfBroadcaster;      // TF广播器
    tf::StampedTransform laserOdometryTrans;     // 激光里程计变换
};