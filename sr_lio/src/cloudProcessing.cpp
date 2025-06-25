#include "cloudProcessing.h"
#include "utility.h"

class PlaneFactor:public ceres::SizedCostFunction<1, 4> 
{
public:
    PlaneFactor( double x, double y, double z )
      : x_(x), y_(y), z_(z) { }
      
    virtual ~PlaneFactor() {};

    virtual bool Evaluate(
      double const* const* parameters, double *residuals, double **jacobians) const 
      {
        double a = parameters[0][0];
        double b = parameters[0][1];
        double c = parameters[0][2];
        double d = parameters[0][3];

        double n = a * x_ + b * y_ + c * z_ + d;
        double m = sqrt(a * a + b * b + c * c);
        residuals[0] = n / m;

        if(jacobians != NULL && jacobians[0] != NULL) 
        { 
          jacobians[0][0] = (x_ * m - a / m *n) / (m* m);
          jacobians[0][1] = (y_ * m - b / m *n) / (m* m);
          jacobians[0][2] = (z_ * m - c / m *n) / (m* m);
          jacobians[0][3] = 1 / m; 
        }
        return true;
    }
private:
    const double x_;
    const double y_;
    const double z_;
};

cloudProcessing::cloudProcessing()
{
	point_filter_num = 1;
	sweep_id = 0;
}

void cloudProcessing::initialization()
{
    full_cloud.resize(N_SCANS * Horizon_SCANS);

    //* 表示创建一个坐标都是无效的点
    nan_point.raw_point.x() = std::numeric_limits<float>::quiet_NaN();
    nan_point.raw_point.y() = std::numeric_limits<float>::quiet_NaN();
    nan_point.raw_point.z() = std::numeric_limits<float>::quiet_NaN();
    nan_point.point.x() = std::numeric_limits<float>::quiet_NaN();
    nan_point.point.y() = std::numeric_limits<float>::quiet_NaN();
    nan_point.point.z() = std::numeric_limits<float>::quiet_NaN();
    nan_point.label = -1;

    sensor_mount_angle = 0.0;              //* 激光雷达安装角度 默认为水平安装即 0°
    segment_theta = 60.0 / 180.0 * M_PI;     //* 分割算法角度阈值
    segment_valid_point_num = 5;             //* 有效分割最少点数
    segment_valid_line_num = 3;              //* 有效分割最少线数
    segment_alpha_x = ang_res_x / 180.0 * M_PI;    //* 水平角度分辨率 弧度
    segment_alpha_y = ang_res_y / 180.0 * M_PI;

    std::pair<int8_t, int8_t> neighbor;        //* 定义四个搜索方向
    neighbor.first = -1; neighbor.second =  0; neighbor_iterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighbor_iterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighbor_iterator.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighbor_iterator.push_back(neighbor);

    //* all_pushed_id_x，all_pushed_id_y 记录已经处理过的点的坐标索引
    all_pushed_id_x = new uint16_t[N_SCANS * Horizon_SCANS];
    all_pushed_id_y = new uint16_t[N_SCANS * Horizon_SCANS];

    //* 区域生长算法中待处理的点的坐标
    queue_id_x = new uint16_t[N_SCANS * Horizon_SCANS];
    queue_id_y = new uint16_t[N_SCANS * Horizon_SCANS];

    resetParameters();
}

//* 重置点云处理过程中使用的各种数据结构
void cloudProcessing::resetParameters()
{
    //* cv::Mat(rows, cols, type, initial_value)：OpenCV矩阵构造函数
    range_mat = cv::Mat(N_SCANS, Horizon_SCANS, CV_32F, cv::Scalar::all(FLT_MAX));   //* FLT_MAX：c++标准库浮点数最大常量
    ground_mat = cv::Mat(N_SCANS, Horizon_SCANS, CV_8S, cv::Scalar::all(0));    //* 标记地面点
    label_mat = cv::Mat(N_SCANS, Horizon_SCANS, CV_32S, cv::Scalar::all(0));    //* 编辑分割点
    label_count = 1;

    std::fill(full_cloud.begin(), full_cloud.end(), nan_point);  //* std::vector<point3D> full_cloud;
}

void cloudProcessing::setLidarType(int para)
{
	lidar_type = para;
}

void cloudProcessing::setNumScans(int para)
{
	N_SCANS = para;

	for(int i = 0; i < N_SCANS; i++){
		pcl::PointCloud<pcl::PointXYZINormal> v_cloud_temp;
		v_cloud_temp.clear();
		scan_cloud.push_back(v_cloud_temp);
	}

	assert(N_SCANS == scan_cloud.size());

	for(int i = 0; i < N_SCANS; i++){
		std::vector<extraElement> v_elem_temp;
		v_extra_elem.push_back(v_elem_temp);
	}

	assert(N_SCANS == v_extra_elem.size());
}

void cloudProcessing::setHorizonScans(int para)
{
    Horizon_SCANS = para;
}

void cloudProcessing::setAngResX(float para)
{
    ang_res_x = para;
}

void cloudProcessing::setAngResY(float para)
{
    ang_res_y = para;
}

void cloudProcessing::setAngBottom(float para)
{
    ang_bottom = para;
}

void cloudProcessing::setGroundScanInd(int para)
{
    ground_scan_id = para;
}

void cloudProcessing::setScanRate(int para)
{
	SCAN_RATE = para;
     delta_cut_time = 1.0 / (double)SCAN_RATE * 1000.0;
}

void cloudProcessing::setTimeUnit(int para)
{
	time_unit = para;

	switch (time_unit)
	{
	case SEC:
		time_unit_scale = 1.e3f;
		break;
	case MS:
		time_unit_scale = 1.f;
		break;
	case US:
		time_unit_scale = 1.e-3f;
		break;
	case NS:
		time_unit_scale = 1.e-6f;
		break;
	default:
		time_unit_scale = 1.f;
		break;
	}
}

void cloudProcessing::setBlind(double para)
{
	blind = para;
}

void cloudProcessing::setExtrinR(Eigen::Matrix3d &R)
{
	R_imu_lidar = R;
}

void cloudProcessing::setExtrinT(Eigen::Vector3d &t)
{
	t_imu_lidar = t;
}

void cloudProcessing::setPointFilterNum(int para)
{
	point_filter_num = para;
}

//* 输出去除降采样后的有效点(包括地面点)
void cloudProcessing::process(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<point3D> &v_cloud_out, double &dt_offset)
{
	switch (lidar_type)
	{
	case OUST:
		ousterHandler(msg, dt_offset);
		break;

	case VELO:   
		velodyneHandler(msg, dt_offset);    //* 将msg 注册到距离图 同时更新到局部点云 full_point 并返回一帧点云扫描时间dt_offset
		break;

	case ROBO:
		robosenseHandler(msg, dt_offset);
		break;

	default:
		ROS_ERROR("Only Velodyne LiDAR interface is supported currently.");
		break;
	}
     groundRemoval();   //* 更新label_mat:将无效点和地面点标志置位-1 都认为是无效点

     cloudSegmentation(v_cloud_out);    //* 每间隔point_filter_num获取一个点，去除无效点，同时将地面点和有效点都输出到v_cloud_out中

     sweep_id++;    //* sweep_id应该是帧id

     resetParameters();   //* 更新重置
}

void cloudProcessing::ousterHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, double &dt_offset)
{
     pcl::PointCloud<ouster_ros::Point> raw_cloud;
     pcl::fromROSMsg(*msg, raw_cloud);
     int size = raw_cloud.points.size();

     double dt_last_point;

     if (size == 0)
     {
          dt_offset = delta_cut_time;
          return;
     }

     if (raw_cloud.points[size - 1].t > 0)
          given_offset_time = true;
     else
          given_offset_time = false;

     if (raw_cloud.points[size - 1].ring > 0)
        given_ring = true;
    else
        given_ring = false;

     if (given_offset_time)
     {
          sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_ouster);
          dt_last_point = raw_cloud.points.back().t * time_unit_scale;
          delta_cut_time = dt_last_point;
     }

     double omega = 0.361 * SCAN_RATE;

     std::vector<bool> is_first;
     is_first.resize(N_SCANS);
     fill(is_first.begin(), is_first.end(), true);

     std::vector<double> yaw_first_point;
     yaw_first_point.resize(N_SCANS);
     fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

     double max_relative_time = -1.0;
     int num_full_points = 0;

     for (int i = 0; i < size; i++)
     {
          if (std::isnan(raw_cloud.points[i].x) || std::isnan(raw_cloud.points[i].y) || std::isnan(raw_cloud.points[i].z))
               continue;

          point3D point_temp;

          point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
          point_temp.point = point_temp.raw_point;
          point_temp.relative_time = raw_cloud.points[i].t * time_unit_scale;

          int row_id, col_id;
          double vertical_angle, horizon_angle, yaw_angle, range;

          if (given_ring)
               row_id = raw_cloud.points[i].ring;
          else
          {
               vertical_angle = atan2(point_temp.raw_point.z(), sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
                    + point_temp.raw_point.y() * point_temp.raw_point.y())) * 180 / M_PI;

               row_id = (vertical_angle + ang_bottom) / ang_res_y;
               if (row_id < 0 || row_id >= N_SCANS) continue;
          }

          yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

          if (is_first[row_id])
          {
               horizon_angle = atan2(point_temp.raw_point.x(), point_temp.raw_point.y()) * 180 / M_PI;

               col_id = -round((horizon_angle - 90.0)/ang_res_x) + Horizon_SCANS/2;
               if (col_id >= Horizon_SCANS)
                    col_id -= Horizon_SCANS;

               if (col_id < 0 || col_id >= Horizon_SCANS) continue;

               range = sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
                    + point_temp.raw_point.y() * point_temp.raw_point.y() 
                    + point_temp.raw_point.z() * point_temp.raw_point.z());

               if (range < blind) continue;

               point_temp.range = range;

               yaw_first_point[row_id] = yaw_angle;
               is_first[row_id] = false;

               if (given_offset_time)
               {
                    point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                    point_temp.alpha_time = point_temp.relative_time / dt_last_point;
               }
               else
                    point_temp.relative_time = 0.0;

               range_mat.at<float>(row_id, col_id) = range;

               num_full_points++;

               full_cloud[col_id  + row_id * Horizon_SCANS] = point_temp;

               continue;
          }

          if (given_offset_time)
          {
               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
               point_temp.alpha_time = point_temp.relative_time / dt_last_point;
          }
          else
          {
               if (yaw_angle <= yaw_first_point[row_id])
                    point_temp.relative_time = (yaw_first_point[row_id] - yaw_angle) / omega;
               else
                    point_temp.relative_time = (yaw_first_point[row_id] - yaw_angle + 360.0) / omega;

               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();

               if (point_temp.relative_time > max_relative_time) max_relative_time = point_temp.relative_time;
          }

          horizon_angle = atan2(point_temp.raw_point.x(), point_temp.raw_point.y()) * 180 / M_PI;

          col_id = -round((horizon_angle - 90.0)/ang_res_x) + Horizon_SCANS/2;
          if (col_id >= Horizon_SCANS)
               col_id -= Horizon_SCANS;

          if (col_id < 0 || col_id >= Horizon_SCANS) continue;

          range = sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
               + point_temp.raw_point.y() * point_temp.raw_point.y() 
               + point_temp.raw_point.z() * point_temp.raw_point.z());

          if (range < blind) continue;

          point_temp.range = range;

          range_mat.at<float>(row_id, col_id) = range;

          num_full_points++;

          full_cloud[col_id  + row_id * Horizon_SCANS] = point_temp;
     }

     if (!given_offset_time)
     {
          dt_last_point = max_relative_time;

          for (int i = 0; i < full_cloud.size(); i++)
          {
               if (full_cloud[i].label >= 0)
               {
                    full_cloud[i].alpha_time = (full_cloud[i].relative_time / dt_last_point);

                    if (full_cloud[i].alpha_time > 1) full_cloud[i].alpha_time = 1;
                    if (full_cloud[i].alpha_time < 0) full_cloud[i].alpha_time = 0;
               }
          }
     }

     dt_offset = dt_last_point;
}

//* 使用的是velodyne16 所以主要关注这个回调函数   dt_offset是一帧扫描点云的时间
void cloudProcessing::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, double &dt_offset)
{
	pcl::PointCloud<velodyne_ros::Point> raw_cloud;
     pcl::fromROSMsg(*msg, raw_cloud);
     int size = raw_cloud.points.size();

     double dt_last_point;

     if(size == 0)
     {
          dt_offset = delta_cut_time;
          return;
     }

     //* 检查是否包含时间戳以及环号信息
     if (raw_cloud.points[size - 1].time > 0)
     	given_offset_time = true;
     else
          given_offset_time = false;

     if (raw_cloud.points[size - 1].ring > 0)
          given_ring = true;
     else
          given_ring = false;


     if(given_offset_time)   //* 雷达如果给时间 给的都是相对时间
     {    //* 按照点时间重小到大排序
          sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_velodyne);

          dt_last_point = raw_cloud.points.back().time * time_unit_scale;   //* 保留当前帧最后一个点的事件戳
          delta_cut_time = dt_last_point;
     }

     double omega = 0.361 * SCAN_RATE;

     std::vector<bool> is_first;
     is_first.resize(N_SCANS);
     fill(is_first.begin(), is_first.end(), true);

     std::vector<double> yaw_first_point;    //* 每行扫描点的第一个偏航角
     yaw_first_point.resize(N_SCANS);
     fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

     double max_relative_time = -1.0;   //* 记录相对最大时间
     int num_full_points = 0;           //* 有效点数

     for (int i = 0; i < size; i++)
     {
          if (std::isnan(raw_cloud.points[i].x) || std::isnan(raw_cloud.points[i].y) || std::isnan(raw_cloud.points[i].z))
               continue;

          point3D point_temp;

          point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
          point_temp.point = point_temp.raw_point;
          point_temp.relative_time = raw_cloud.points[i].time * time_unit_scale;

          int row_id, col_id;
          double vertical_angle, horizon_angle, yaw_angle, range;

          if (given_ring)
          {
               row_id = raw_cloud.points[i].ring;
               assert(row_id >= 0 && row_id < N_SCANS);
          }
          //* 根据点的XYZ计算垂直角度，进而计算行索引
          else
          {
               vertical_angle = atan2(point_temp.raw_point.z(), sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
                    + point_temp.raw_point.y() * point_temp.raw_point.y())) * 180 / M_PI;

               row_id = (vertical_angle + ang_bottom) / ang_res_y;
               if (row_id < 0 || row_id >= N_SCANS) continue;
          }

          yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

          //* 对每一行的第一个点
          if (is_first[row_id])
          {
               horizon_angle = atan2(point_temp.raw_point.x(), point_temp.raw_point.y()) * 180 / M_PI;
               
               //* 计算列索引
               col_id = -round((horizon_angle - 90.0)/ang_res_x) + Horizon_SCANS/2;
               if (col_id >= Horizon_SCANS)
                    col_id -= Horizon_SCANS;

               if (col_id < 0 || col_id >= Horizon_SCANS) continue;

               //* 计算点的距离
               range = sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
                    + point_temp.raw_point.y() * point_temp.raw_point.y() 
                    + point_temp.raw_point.z() * point_temp.raw_point.z());
               //* 去除小于盲区的点
               if (range < blind) continue;

               point_temp.range = range;

               yaw_first_point[row_id] = yaw_angle;
               is_first[row_id] = false;

               if (given_offset_time)
               {
                    point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                    point_temp.alpha_time = point_temp.relative_time / dt_last_point;
               }
               else
                    point_temp.relative_time = 0.0;

               //* 存放到平面图中
               range_mat.at<float>(row_id, col_id) = range;

               num_full_points++;

               //* full_cloud是局部扫描点云
               full_cloud[col_id  + row_id * Horizon_SCANS] = point_temp;

               continue;
          }

          //* 更新普通点的时间戳
          if (given_offset_time)   //* 如果有时间戳信息 则更新时间戳
          {
               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
               point_temp.alpha_time = point_temp.relative_time / dt_last_point;
          }
          else
          {
               if (yaw_angle <= yaw_first_point[row_id])
                    point_temp.relative_time = (yaw_first_point[row_id] - yaw_angle) / omega;
               else
                    point_temp.relative_time = (yaw_first_point[row_id] - yaw_angle + 360.0) / omega;

               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();

               if (point_temp.relative_time > max_relative_time) max_relative_time = point_temp.relative_time;
          }

          //* 计算普通点的列索引 并存入距离图以及注册到局部点云中
          horizon_angle = atan2(point_temp.raw_point.x(), point_temp.raw_point.y()) * 180 / M_PI;

          col_id = -round((horizon_angle - 90.0)/ang_res_x) + Horizon_SCANS/2;
          if (col_id >= Horizon_SCANS)
               col_id -= Horizon_SCANS;

          if (col_id < 0 || col_id >= Horizon_SCANS) continue;

          range = sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
               + point_temp.raw_point.y() * point_temp.raw_point.y() 
               + point_temp.raw_point.z() * point_temp.raw_point.z());

          if (range < blind) continue;

          point_temp.range = range;

          range_mat.at<float>(row_id, col_id) = range;

          num_full_points++;

          full_cloud[col_id  + row_id * Horizon_SCANS] = point_temp;
     }

     if (!given_offset_time)
     {
          dt_last_point = max_relative_time;

          for (int i = 0; i < full_cloud.size(); i++)
          {
               if (full_cloud[i].label >= 0)
               {
                    full_cloud[i].alpha_time = (full_cloud[i].relative_time / dt_last_point);

                    if (full_cloud[i].alpha_time > 1) full_cloud[i].alpha_time = 1;
                    if (full_cloud[i].alpha_time < 0) full_cloud[i].alpha_time = 0;
               }
          }
     }

     dt_offset = dt_last_point;
}

void cloudProcessing::robosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, double &dt_offset)
{
     int ring_max = 0;

     pcl::PointCloud<robosense_ros::Point> raw_cloud;
     pcl::fromROSMsg(*msg, raw_cloud);
     int size = raw_cloud.points.size();

     double dt_last_point;

     if (size == 0)
     {
          dt_offset = delta_cut_time;
          return;
     }

     if (raw_cloud.points[size - 1].timestamp > 0)
          given_offset_time = true;
     else
          given_offset_time = false;

     if (raw_cloud.points[size - 1].ring > 0)
          given_ring = true;
     else
          given_ring = false;

     if (given_offset_time)
     {
          sort(raw_cloud.points.begin(), raw_cloud.points.end(), time_list_robosense);
          dt_last_point = (raw_cloud.points.back().timestamp - raw_cloud.points.front().timestamp) * time_unit_scale;
          delta_cut_time = dt_last_point;
     }

     double omega = 0.361 * SCAN_RATE;

     std::vector<bool> is_first;
     is_first.resize(N_SCANS);
     fill(is_first.begin(), is_first.end(), true);

     std::vector<double> yaw_first_point;
     yaw_first_point.resize(N_SCANS);
     fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

     double max_relative_time = -1.0;
     int num_full_points = 0;

     for (int i = 0; i < size; i++)
     {
          if (std::isnan(raw_cloud.points[i].x) || std::isnan(raw_cloud.points[i].y) || std::isnan(raw_cloud.points[i].z))
               continue;

          point3D point_temp;

          point_temp.raw_point = Eigen::Vector3d(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);
          point_temp.point = point_temp.raw_point;
          point_temp.relative_time = raw_cloud.points[i].timestamp * time_unit_scale;

          int row_id, col_id;
          double vertical_angle, horizon_angle, yaw_angle, range;

          if (given_ring)
               row_id = raw_cloud.points[i].ring;
          else
          {
               vertical_angle = atan2(point_temp.raw_point.z(), sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
                    + point_temp.raw_point.y() * point_temp.raw_point.y())) * 180 / M_PI;

               row_id = (vertical_angle + ang_bottom) / ang_res_y;
               if (row_id > ring_max)
               {
                    ring_max = row_id;
               } 

               if (row_id < 0 || row_id >= N_SCANS) continue;
          }

          yaw_angle = atan2(point_temp.raw_point.y(), point_temp.raw_point.x()) * 57.2957;

          if (is_first[row_id])
          {
               horizon_angle = atan2(point_temp.raw_point.x(), point_temp.raw_point.y()) * 180 / M_PI;

               col_id = -round((horizon_angle - 90.0)/ang_res_x) + Horizon_SCANS/2;
               if (col_id >= Horizon_SCANS)
                    col_id -= Horizon_SCANS;

               if (col_id < 0 || col_id >= Horizon_SCANS) continue;

               range = sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
                    + point_temp.raw_point.y() * point_temp.raw_point.y() 
                    + point_temp.raw_point.z() * point_temp.raw_point.z());

               if (range < blind) continue;

               point_temp.range = range;

               yaw_first_point[row_id] = yaw_angle;
               is_first[row_id] = false;

               if (given_offset_time)
               {
                    point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
                    point_temp.alpha_time = point_temp.relative_time / dt_last_point;
               }
               else
                    point_temp.relative_time = 0.0;

               range_mat.at<float>(row_id, col_id) = range;

               num_full_points++;

               full_cloud[col_id  + row_id * Horizon_SCANS] = point_temp;

               continue;
          }

          if (given_offset_time)
          {
               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();
               point_temp.alpha_time = point_temp.relative_time / dt_last_point;
          }
          else
          {
               if (yaw_angle <= yaw_first_point[row_id])
                    point_temp.relative_time = (yaw_first_point[row_id] - yaw_angle) / omega;
               else
                    point_temp.relative_time = (yaw_first_point[row_id] - yaw_angle + 360.0) / omega;

               point_temp.timestamp = point_temp.relative_time / double(1000) + msg->header.stamp.toSec();

               if (point_temp.relative_time > max_relative_time) max_relative_time = point_temp.relative_time;
          }

          horizon_angle = atan2(point_temp.raw_point.x(), point_temp.raw_point.y()) * 180 / M_PI;

          col_id = -round((horizon_angle - 90.0)/ang_res_x) + Horizon_SCANS/2;
          if (col_id >= Horizon_SCANS)
               col_id -= Horizon_SCANS;

          if (col_id < 0 || col_id >= Horizon_SCANS) continue;

          range = sqrt(point_temp.raw_point.x() * point_temp.raw_point.x() 
               + point_temp.raw_point.y() * point_temp.raw_point.y() 
               + point_temp.raw_point.z() * point_temp.raw_point.z());

          if (range < blind) continue;

          point_temp.range = range;

          range_mat.at<float>(row_id, col_id) = range;

          full_cloud[col_id  + row_id * Horizon_SCANS] = point_temp;
     }

     if (!given_offset_time)
     {
          dt_last_point = max_relative_time;

          for (int i = 0; i < full_cloud.size(); i++)
          {
               if (full_cloud[i].label >= 0)
               {
                    full_cloud[i].alpha_time = (full_cloud[i].relative_time / dt_last_point);

                    if (full_cloud[i].alpha_time > 1) full_cloud[i].alpha_time = 1;
                    if (full_cloud[i].alpha_time < 0) full_cloud[i].alpha_time = 0;
               }
          }
     }

     dt_offset = dt_last_point;

     std::cout << "ring_max = " << ring_max << std::endl;
}

void cloudProcessing::groundRemoval()
{
     size_t lower_id, upper_id;
     float diff_x, diff_y, diff_z, angle;

     for (size_t j = 0; j < Horizon_SCANS; j++)
     {
          for (size_t i = 0; i < ground_scan_id; i++)    //* 只扫描下面几个环线
          {
               lower_id = j + i * Horizon_SCANS;                  //* 当前扫描线上的点
               upper_id = j + (i + 1) * Horizon_SCANS;            //* 下一扫描线上的店

               if (full_cloud[lower_id].label == -1 || full_cloud[upper_id].label == -1)
               {
                    // no info to check, invalid points
                    ground_mat.at<int8_t>(i, j) = -1;             //* -1 标记为无效点
                    continue;
               }
                
               diff_x = full_cloud[upper_id].raw_point.x() - full_cloud[lower_id].raw_point.x();
               diff_y = full_cloud[upper_id].raw_point.y() - full_cloud[lower_id].raw_point.y();
               diff_z = full_cloud[upper_id].raw_point.z() - full_cloud[lower_id].raw_point.z();

               angle = atan2(diff_z, sqrt(diff_x * diff_x + diff_y * diff_y)) * 180 / M_PI;     //* 计算上下两点连线与水平面夹角

               if (abs(angle - sensor_mount_angle) <= 1)           //* 与传感器安装角度不超过1° 则判定为地面点（上下两条线都标记为地面点）
               {
                    if (ground_mat.at<int8_t>(i, j) != 1) 
                         ground_mat.at<int8_t>(i, j) = 1;

                    if (ground_mat.at<int8_t>(i + 1, j) != 1)
                         ground_mat.at<int8_t>(i + 1, j) = 1;
               }
          }
     }

     //* 再次遍历所有点更新点云标签：无效点和地面点都统称为无效点
     for (size_t i = 0; i < N_SCANS; i++)
     {
          for (size_t j = 0; j < Horizon_SCANS; j++)
          {
               if (ground_mat.at<int8_t>(i, j) == 1 || ground_mat.at<float>(i, j) == -1)
                    label_mat.at<int>(i,j) = -1;
          }
     }
}

//* 每间隔point_filter_num获取一个点，去除无效点，同时将地面点和有效点都输出到v_cloud_out中
void cloudProcessing::cloudSegmentation(std::vector<point3D> &v_cloud_out)
{
     int num_points = 0;

     for (size_t i = 0; i < N_SCANS; ++i)
     {
          for (size_t j = 0; j < Horizon_SCANS; j++)
          {
              if (label_mat.at<int>(i, j) == 999999) continue;

              if (num_points % point_filter_num != 0)    //* 暴力降采样，每间隔point_filter_num点采样一次
              {
                   num_points++;
                   continue;
              }

              if (ground_mat.at<int8_t>(i, j) == 1)
              {
                   full_cloud[j + i * Horizon_SCANS].label = 0;      //* 标记为地面点
                   v_cloud_out.push_back(full_cloud[j + i * Horizon_SCANS]);   
                   assert(!std::isnan(full_cloud[j + i * Horizon_SCANS].raw_point.x()));
                   assert(!std::isnan(full_cloud[j + i * Horizon_SCANS].raw_point.y()));
                   assert(!std::isnan(full_cloud[j + i * Horizon_SCANS].raw_point.z()));
              }
              else if (full_cloud[j + i * Horizon_SCANS].label >= 0)
              {
                   assert(ground_mat.at<int8_t>(i, j) != 1);
                   full_cloud[j + i * Horizon_SCANS].label = 1;
                   v_cloud_out.push_back(full_cloud[j + i * Horizon_SCANS]);
                   assert(!std::isnan(full_cloud[j + i * Horizon_SCANS].raw_point.x()));
                   assert(!std::isnan(full_cloud[j + i * Horizon_SCANS].raw_point.y()));
                   assert(!std::isnan(full_cloud[j + i * Horizon_SCANS].raw_point.z()));
              }

              num_points++;
          }
     }
}

void cloudProcessing::labelComponents(int row, int col)
{
     float d1, d2, alpha, angle;
     int from_id_x, from_id_y, this_id_x, this_id_y;

     bool line_count_flag[N_SCANS] = {false};

     queue_id_x[0] = row;
     queue_id_y[0] = col;

     int queue_size = 1;
     int queue_start_id = 0;
     int queue_end_id = 1;

     all_pushed_id_x[0] = row;
     all_pushed_id_y[0] = col;

     int all_pushed_id_size = 1;

     while(queue_size > 0)
     {
          from_id_x = queue_id_x[queue_start_id];
          from_id_y = queue_id_y[queue_start_id];

          queue_size--;
          queue_start_id++;

          label_mat.at<int>(from_id_x, from_id_y) = label_count;

          for (auto iter = neighbor_iterator.begin(); iter != neighbor_iterator.end(); ++iter)
          {
               this_id_x = from_id_x + (*iter).first;
               this_id_y = from_id_y + (*iter).second;

               if (this_id_x < 0 || this_id_x >= N_SCANS) continue;

               if (this_id_y < 0)
                    this_id_y = Horizon_SCANS - 1;
               if (this_id_y >= Horizon_SCANS)
                    this_id_y = 0;

               if (label_mat.at<int>(this_id_x, this_id_y) != 0) continue;

               d1 = std::max(range_mat.at<float>(from_id_x, from_id_y), 
                              range_mat.at<float>(this_id_x, this_id_y));

               d2 = std::min(range_mat.at<float>(from_id_x, from_id_y), 
                              range_mat.at<float>(this_id_x, this_id_y));

               if ((*iter).first == 0)
                    alpha = segment_alpha_x;
               else
                    alpha = segment_alpha_y;

               angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

               if (angle > segment_theta)
               {
                    queue_id_x[queue_end_id] = this_id_x;
                    queue_id_y[queue_end_id] = this_id_y;
                    queue_size++;
                    queue_end_id++;

                    label_mat.at<int>(this_id_x, this_id_y) = label_count;
                    line_count_flag[this_id_x] = true;

                    all_pushed_id_x[all_pushed_id_size] = this_id_x;
                    all_pushed_id_y[all_pushed_id_size] = this_id_y;
                    all_pushed_id_size++;
               }
          }
     }

     bool feasible_segment = false;

     if (all_pushed_id_size >= 2)
          feasible_segment = true;

     if (feasible_segment == true)
     {
          label_count++;
     }
     else
     {
          for (size_t i = 0; i < all_pushed_id_size; i++)
               label_mat.at<int>(all_pushed_id_x[i], all_pushed_id_y[i]) = 999999;
     }
}