// c++
#include <iostream>
#include <math.h>
#include <vector>

// eigen 
#include <Eigen/Core>

#include "cloudMap.h"
#include "lioOptimization.h"

#include <thread>

// utility
#include "utility.h"
#include "parameters.h"
//* cur_icp_options：icp选项   voxel_map_temp：体素化地图    keypoints：降采样后的关键点   plane_residuals：平面残差    p_frame
optimizeSummary lioOptimization::buildPlaneResiduals(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, 
    std::vector<planeParam> &plane_residuals, cloudFrame *p_frame, double &loss_sum)
{
    const short nb_voxels_visited = p_frame->frame_id < cur_icp_options.init_num_frames ? 2 : cur_icp_options.voxel_neighborhood;
    const int kMinNumNeighbors = cur_icp_options.min_number_neighbors;
    const int kThresholdCapacity = p_frame->frame_id < cur_icp_options.init_num_frames ? 1 : cur_icp_options.threshold_voxel_occupancy;

    //* 查询上一帧和当前帧的状态 以及当前帧的旋转四元数和平移向量
    state *last_state = all_cloud_frame[p_frame->id - 1]->p_state;
    state *current_state = p_frame->p_state;
    Eigen::Quaterniond end_quat = Eigen::Quaterniond(current_state->rotation);
    Eigen::Vector3d end_t = current_state->translation;

    //* 将点云从激光雷达坐标系转换到世界坐标系
    auto transformKeypoints = [&]()
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        for (auto &keypoint: keypoints) {
            R = end_quat.normalized().toRotationMatrix();
            t = end_t;

            keypoint.point = R * (R_imu_lidar * keypoint.raw_point + t_imu_lidar) + t;  //* 先从雷达坐标系转换为imu坐标系，再转换到世界坐标系
        }
    };

    auto estimatePointNeighborhood = [&](std::vector<globalPoint> &vector_neighbors, Eigen::Vector3d &location, double &planarity_weight)
    {
        //* 计算vector_neighbors点云分布情况
        auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
        planarity_weight = std::pow(neighborhood.a2D, cur_icp_options.power_planarity);   //* 将获取的分布系数平方处理

        if (neighborhood.normal.dot(last_state->translation - location) < 0) {
            neighborhood.normal = -1.0 * neighborhood.normal;
        }
        return neighborhood;
    };

    double lambda_weight = std::abs(cur_icp_options.weight_alpha);
    double lambda_neighborhood = std::abs(cur_icp_options.weight_neighborhood);
    const double kMaxPointToPlane = cur_icp_options.max_dist_to_plane_icp;   //* 配置点到平面最大距离为0.3
    const double sum = lambda_weight + lambda_neighborhood;

    lambda_weight /= sum;
    lambda_neighborhood /= sum;

    int num_residuals = 0;
    int num_keypoints = keypoints.size();

    //* 将以雷达为坐标系的点转换到世界坐标系当中
    transformKeypoints();

    for (int k = 0; k < num_keypoints; k++) {
        auto &keypoint = keypoints[k];
        auto &raw_point = keypoint.raw_point;

        std::vector<voxel> voxels;
        //* vector_neighbors是与当前keypoint.point最近的一系列点
        auto vector_neighbors = searchNeighbors(voxel_map_temp, keypoint.point,
                                                 nb_voxels_visited, cur_icp_options.size_voxel_map,
                                                 cur_icp_options.max_number_neighbors, kThresholdCapacity,
                                                 cur_icp_options.estimate_normal_from_neighborhood ? nullptr : &voxels);

        //* 要求邻域内找到的最临近点足够多
        if (vector_neighbors.size() < kMinNumNeighbors)
            continue;

        double weight;

        Eigen::Vector3d location = R_imu_lidar * raw_point + t_imu_lidar;

        auto neighborhood = estimatePointNeighborhood(vector_neighbors, location, weight);

        //* 平面度越大，距离越近，权重越高
        weight = lambda_weight * weight + lambda_neighborhood * std::exp(-(vector_neighbors[0].getPosition() -
                 keypoint.point).norm() / (kMaxPointToPlane * kMinNumNeighbors));

        planeParam plane_temp;
        plane_temp.raw_point = location;
        plane_temp.norm_vector = neighborhood.normal;
        plane_temp.norm_vector.normalize();
        plane_temp.norm_offset = - plane_temp.norm_vector.dot(vector_neighbors[0].getPosition());
        plane_temp.distance = plane_temp.norm_vector.dot(end_quat.toRotationMatrix() * plane_temp.raw_point + end_t) + plane_temp.norm_offset;
        plane_temp.weight = weight;

        if (plane_temp.distance < cur_icp_options.max_dist_to_plane_icp) {
            num_residuals++;
            plane_temp.jacobians.block<1, 3>(0, 0) = plane_temp.norm_vector.transpose() * weight;
            plane_temp.jacobians.block<1, 3>(0, 3) = - plane_temp.norm_vector.transpose() * end_quat.toRotationMatrix() * numType::skewSymmetric(plane_temp.raw_point) * weight;
            plane_residuals.push_back(plane_temp);

            loss_sum += plane_temp.distance * plane_temp.distance;
        }

        if(num_residuals >= cur_icp_options.max_num_residuals) break;
    }

    if (num_residuals < cur_icp_options.min_number_neighbors)
    {
        std::stringstream ss_out;
        ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
        ss_out << "[Optimization] number_of_residuals : " << num_residuals << std::endl;
        optimizeSummary summary;
        summary.success = false;
        summary.num_residuals_used = num_residuals;
        summary.error_log = ss_out.str();
        if (cur_icp_options.debug_print) {
            std::cout << summary.error_log;
        }
        return summary;
    }
    else
    {
        optimizeSummary summary;
        summary.success = true;
        summary.num_residuals_used = num_residuals;
        return summary;
    }
}


//* IEKF
optimizeSummary lioOptimization::updateIEKF(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame)
{    
    int max_num_iter = p_frame->frame_id < cur_icp_options.init_num_frames ? 
        std::max(15, cur_icp_options.num_iters_icp) : cur_icp_options.num_iters_icp;

    Eigen::Vector3d p_predict = eskf_pro->getTranslation();
    Eigen::Quaterniond q_predict = eskf_pro->getRotation();
    Eigen::Vector3d v_predict = eskf_pro->getVelocity();
    Eigen::Vector3d ba_predict = eskf_pro->getBa();
    Eigen::Vector3d bg_predict = eskf_pro->getBg();
    Eigen::Vector3d g_predict = eskf_pro->getGravity();

    optimizeSummary summary;

    for (int i = -1; i < max_num_iter; i++)
    {
        std::vector<planeParam> plane_residuals;

        double loss_old = 0.0;

        summary = buildPlaneResiduals(cur_icp_options, voxel_map_temp, keypoints, plane_residuals, p_frame, loss_old);

        if (summary.success == false)
            return summary;

        //* 根据残差数量更新观测矩阵
        int num_plane_residuals = plane_residuals.size();

        Eigen::Matrix<double, Eigen::Dynamic, 6> H_x;
        Eigen::Matrix<double, Eigen::Dynamic, 1> h; 

        H_x.resize(num_plane_residuals, 6);
        h.resize(num_plane_residuals, 1);

        for (int i = 0; i < num_plane_residuals; i++)
        {
            H_x.block<1, 6>(i, 0) = plane_residuals[i].jacobians;
            h.block<1, 1>(i, 0) = Eigen::Matrix<double, 1, 1>(plane_residuals[i].distance * plane_residuals[i].weight);
        }

        //* 
        Eigen::Vector3d d_p = eskf_pro->getTranslation() - p_predict;
        Eigen::Quaterniond d_q = q_predict.inverse() * eskf_pro->getRotation();
        Eigen::Vector3d d_so3 = numType::quatToSo3(d_q);
        Eigen::Vector3d d_v = eskf_pro->getVelocity() - v_predict;
        Eigen::Vector3d d_ba = eskf_pro->getBa() - ba_predict;
        Eigen::Vector3d d_bg = eskf_pro->getBg() - bg_predict;

        Eigen::Vector3d g = eskf_pro->getGravity();

        Eigen::Vector3d g_predict_normalize = g_predict;
        Eigen::Vector3d g_normalize = g;

        g_predict_normalize.normalize();
        g_normalize.normalize();

        Eigen::Vector3d cross = g_predict_normalize.cross(g_normalize);
        double dot = g_predict_normalize.dot(g_normalize);

        Eigen::Matrix3d R_dg;

        if (fabs(1.0 - dot) < 1e-6)
            R_dg = Eigen::Matrix3d::Identity();
        else
        {
            Eigen::Matrix3d skew = numType::skewSymmetric(cross);
            R_dg = Eigen::Matrix3d::Identity() + skew + skew * skew * (1.0 - dot) 
                / (cross(0) * cross(0) + cross(1) * cross(1) + cross(2) * cross(2));
        }

        Eigen::Vector3d so3_dg = numType::rotationToSo3(R_dg);
        Eigen::Matrix<double, 3, 2> B_x_predict = numType::derivativeS2(g_predict);
        Eigen::Vector2d d_g = B_x_predict.transpose() * so3_dg;

        Eigen::Matrix<double, 17, 1> d_x;
        d_x.head<3>() = d_p;
        d_x.segment<3>(3) = d_so3;
        d_x.segment<3>(6) = d_v;
        d_x.segment<3>(9) = d_ba;
        d_x.segment<3>(12) = d_bg;
        d_x.tail<2>() = d_g;

        Eigen::Matrix3d J_k_so3 = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(d_so3);
        Eigen::Matrix2d J_k_s2 = Eigen::Matrix2d::Identity() + 0.5 * B_x_predict.transpose() * numType::skewSymmetric(so3_dg) * B_x_predict;

        Eigen::Matrix<double, 17, 1> d_x_new = d_x;
        d_x_new.segment<3>(3) = J_k_so3 * d_so3;
        d_x_new.segment<2>(15) = J_k_s2 * d_g;
        //* 更新协方差矩阵 
        Eigen::Matrix<double, 17, 17> covariance = eskf_pro->getCovariance();

        for (int j = 0; j < covariance.cols(); j++)
            covariance.block<3, 1>(3, j) = J_k_so3 * covariance.block<3, 1>(3, j);

        for (int j = 0; j < covariance.cols(); j++)
            covariance.block<2, 1>(15, j) = J_k_s2 * covariance.block<2, 1>(15, j);

        for (int j = 0; j < covariance.rows(); j++)
            covariance.block<1, 3>(j, 3) = covariance.block<1, 3>(j, 3) * J_k_so3.transpose();

        for (int j = 0; j < covariance.rows(); j++)
            covariance.block<1, 2>(j, 15) = covariance.block<1, 2>(j, 15) * J_k_s2.transpose();
        
        //* 计算卡尔曼增益
        Eigen::Matrix<double, 17, 17> temp = (covariance/laser_point_cov).inverse();
        Eigen::Matrix<double, 6, 6> HTH = H_x.transpose() * H_x;
        temp.block<6, 6>(0, 0) += HTH;
        Eigen::Matrix<double, 17, 17> temp_inv = temp.inverse();

        Eigen::Matrix<double, 17, 1> K_h = temp_inv.block<17, 6>(0, 0) * H_x.transpose() * h;

        Eigen::Matrix<double, 17, 17> K_x = Eigen::Matrix<double, 17, 17>::Zero();
        K_x.block<17, 6>(0, 0) = temp_inv.block<17, 6>(0, 0) * HTH;

        //! 状态更新公式！
        d_x = - K_h + (K_x - Eigen::Matrix<double, 17, 17>::Identity()) * d_x_new;

        Eigen::Vector3d g_before = eskf_pro->getGravity();

        if ((d_x.head<3>()).norm() > 100.0 || AngularDistance(d_x.segment<3>(3)) > 100.0)
        {
            continue;
        }

        eskf_pro->observe(d_x);

        p_frame->p_state->translation = eskf_pro->getTranslation();
        p_frame->p_state->rotation = eskf_pro->getRotation();
        p_frame->p_state->velocity = eskf_pro->getVelocity();
        p_frame->p_state->ba = eskf_pro->getBa();
        p_frame->p_state->bg = eskf_pro->getBg();
        G = eskf_pro->getGravity();
        G_norm = G.norm();

        bool converage = false;

        if (p_frame->frame_id > 1 && 
            (d_x.head<3>()).norm() < cur_icp_options.threshold_translation_norm && 
            AngularDistance(d_x.segment<3>(3)) < cur_icp_options.threshold_orientation_norm)    //* 要求平移和旋转的更新量都小于阈值
        {
            converage = true;
        }

        //* 协方差更新
        if (converage || i == max_num_iter - 1)
        {
            Eigen::Matrix<double, 17, 17> covariance_new = covariance;

            Eigen::Matrix<double, 3, 2> B_x_before = numType::derivativeS2(g_before);

            J_k_so3 = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(d_x.segment<3>(3));
            J_k_s2 = Eigen::Matrix2d::Identity() + 0.5 * B_x_before.transpose() * numType::skewSymmetric(B_x_before * d_x.tail<2>()) * B_x_before;

            for (int j = 0; j < covariance.cols(); j++)
                covariance_new.block<3, 1>(3, j) = J_k_so3 * covariance.block<3, 1>(3, j);

            for (int j = 0; j < covariance.cols(); j++)
                covariance_new.block<2, 1>(15, j) = J_k_s2 * covariance.block<2, 1>(15, j);

            for (int j = 0; j < covariance.rows(); j++)
            {
                covariance_new.block<1, 3>(j, 3) = covariance.block<1, 3>(j, 3) * J_k_so3.transpose();
                covariance.block<1, 3>(j, 3) = covariance.block<1, 3>(j, 3) * J_k_so3.transpose();
            }

            for (int j = 0; j < covariance.rows(); j++)
            {
                covariance_new.block<1, 2>(j, 15) = covariance.block<1, 2>(j, 15) * J_k_s2.transpose();
                covariance.block<1, 2>(j, 15) = covariance.block<1, 2>(j, 15) * J_k_s2.transpose();
            }

            for (int j = 0; j < 6; j++)
                K_x.block<3, 1>(3, j) = J_k_so3 * K_x.block<3, 1>(3, j);

            for (int j = 0; j < 6; j++)
                K_x.block<2, 1>(15, j) = J_k_s2 * K_x.block<2, 1>(15, j);

            covariance = covariance_new - K_x.block<17, 6>(0, 0) * covariance.block<6, 17>(0, 0);

            eskf_pro->setCovariance(covariance);

            break;
        }
    }

    return summary;
}

Neighborhood lioOptimization::computeNeighborhoodDistribution(std::vector<globalPoint> &points)
{
    Neighborhood neighborhood;
    // Compute the normals
    Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
    for (auto &point: points) {
        barycenter += point.getPosition();
    }

    //* 计算点云的质心 barycenter
    barycenter /= (double) points.size();
    neighborhood.center = barycenter;

    //* 计算点云的协方差矩阵
    Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
    for (auto &point: points) {
        for (int k = 0; k < 3; ++k)
            for (int l = k; l < 3; ++l)
                covariance_Matrix(k, l) += (point.getPosition()(k) - barycenter(k)) *
                                           (point.getPosition()(l) - barycenter(l));
    }
    //* 只计算上三角矩阵，利用协方差矩阵的对称性，补充下三角矩阵
    covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
    covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
    covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
    neighborhood.covariance = covariance_Matrix;
    //* 创建一个特征值求解器
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
    Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());    //* 获取最小特征值对应的特征向量，并归一化
    neighborhood.normal = normal;   //* 记录最小特征值对应的单位向量

    double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));    //* 最大特征值
    double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
    double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));    //* 最小特征值
    neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;        //* a2D接近1 则为平面结构 

    if (neighborhood.a2D != neighborhood.a2D) {
        throw std::runtime_error("error");
    }

    return neighborhood;
}

using pair_distance_t = std::tuple<double, globalPoint, voxel>;

struct comparator {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
        return std::get<0>(left) < std::get<0>(right);
    }
};

using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

std::vector<globalPoint> lioOptimization::searchNeighbors(voxelHashMap &map, const Eigen::Vector3d &point,
        int nb_voxels_visited, double size_voxel_map, int max_num_neighbors, int threshold_voxel_capacity, std::vector<voxel> *voxels)
{
    //* 分配邻居体素的数量
    if (voxels != nullptr)
        voxels->reserve(max_num_neighbors);

    //* 计算查询点所在体素的索引
    short kx = static_cast<short>(point[0] / size_voxel_map);
    short ky = static_cast<short>(point[1] / size_voxel_map);
    short kz = static_cast<short>(point[2] / size_voxel_map);


    priority_queue_t priority_queue;

    //* 获取当前点临近体素内最近的几个点
    voxel voxel_temp(kx, ky, kz);
    for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
        for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
            for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                voxel_temp.x = kxx;
                voxel_temp.y = kyy;
                voxel_temp.z = kzz;

                auto search = map.find(voxel_temp);
                if (search != map.end()) {
                    auto &voxel_block = search.value();
                    if (voxel_block.NumPoints() < threshold_voxel_capacity)    //* 如果邻域体素内的点过少，则跳过当前体素
                        continue;
                    for (int i(0); i < voxel_block.NumPoints(); ++i) { 
                        auto &neighbor = voxel_block.points[i];
                        double distance = (neighbor.getPosition() - point).norm();
                        if (priority_queue.size() == max_num_neighbors) {
                            if (distance < std::get<0>(priority_queue.top())) {
                                priority_queue.pop();
                                priority_queue.emplace(distance, neighbor, voxel_temp);   //* 较大的元素排在顶部
                            }
                        } else
                            priority_queue.emplace(distance, neighbor, voxel_temp);
                    }
                }
            }
        }
    }

    auto size = priority_queue.size();
    std::vector<globalPoint> closest_neighbors(size);
    if (voxels != nullptr) {
        voxels->resize(size);
    }
    for (auto i = 0; i < size; ++i) {
        closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());   //* 将顺序反转，小的放在前面
        if (voxels != nullptr)
            (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
        priority_queue.pop();
    }


    return closest_neighbors;
} 

optimizeSummary lioOptimization::optimize(cloudFrame *p_frame, const icpOptions &cur_icp_options, double sample_voxel_size)
{
    std::vector<point3D> keypoints;

    //* 对p_frame->point_frame进行降采样，每sample_voxel_size单位体素下采集一个点 将采样后存入keypoints中
    gridSampling(p_frame->point_frame, keypoints, sample_voxel_size);

    optimizeSummary optimize_summary;

    optimize_summary = updateIEKF(cur_icp_options, voxel_map, keypoints, p_frame);

    if (!optimize_summary.success) {
        return optimize_summary;
    }

    Eigen::Quaterniond q_end = p_frame->p_state->rotation;
    Eigen::Vector3d t_end = p_frame->p_state->translation;
    for (auto &point_temp: p_frame->point_frame) {
        transformPoint(point_temp, q_end, t_end, R_imu_lidar, t_imu_lidar);   //* 将点云从激光雷达坐标系转换到世界坐标系
    }
 
    //* 此时点云是以世界坐标系为中心
    detectDynamicLabel(cur_icp_options, voxel_map, p_frame);

    return optimize_summary;
}

void lioOptimization::detectDynamicLabel(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, cloudFrame *p_frame)
{
    //* 初始化的时候相邻体素为2 在此之后相邻体素为1
    const short nb_voxels_visited = p_frame->frame_id < cur_icp_options.init_num_frames ? 2 : cur_icp_options.voxel_neighborhood;
    const int kMinNumNeighbors = cur_icp_options.min_number_neighbors;
    const int kThresholdCapacity = p_frame->frame_id < cur_icp_options.init_num_frames ? 1 : cur_icp_options.threshold_voxel_occupancy;
    const double voxel_size = options.optimize_options.size_voxel_map;

    for (int i = 0; i < p_frame->point_frame.size(); i++)
    {
        //* 首先过滤掉高超过3.2m的点
        if (p_frame->point_frame[i].raw_point.z() > 3.2)
        {
             continue;
        }

        short kx = static_cast<short>(p_frame->point_frame[i].point.x() / voxel_size);
        short ky = static_cast<short>(p_frame->point_frame[i].point.y() / voxel_size);
        short kz = static_cast<short>(p_frame->point_frame[i].point.z() / voxel_size);

        voxelHashMap::iterator search = voxel_map_temp.find(voxel(kx, ky, kz));

        if(search != voxel_map_temp.end())
        {
            //* 查找当前点所在的体素
            auto &voxel_block = (search.value());

            //* 当某个体素中点较多
            if(voxel_block.NumPoints() > 5)
            {
                if (p_frame->point_frame[i].label > 0)    //* 非地面点
                {
                    int num_ground = 0;

                    for (int i = 0; i < voxel_block.NumPoints(); i++)
                        if (voxel_block.points[i].getLabel() == 0)
                            num_ground++;

                    double ratio_ground = (double)num_ground / (double)voxel_block.NumPoints();

                    if (ratio_ground > 0.1)
                    {
                        p_frame->point_frame[i].is_dynamic = true;
                    }
                }
                continue;
            }
            else
            {
                if (p_frame->point_frame[i].label == 999999)
                {
                    p_frame->point_frame[i].is_dynamic = true;
                    continue;
                }

                //* 当前帧id超过3 且当前点的距离大于30m 将其标定为未决定点
                if (p_frame->point_frame[i].range > 30.0)
                {
                    p_frame->point_frame[i].is_undecided = true;
                    if (p_frame->frame_id > 3)
                        points_undecided.push_back(p_frame->point_frame[i]);
                    continue;
                }

                if (p_frame->point_frame[i].label == 0)
                {
                    continue;
                }

                p_frame->point_frame[i].is_dynamic = true;
                continue;
            }
        }
        else
        {
            if (p_frame->point_frame[i].label == 999999)
            {
                p_frame->point_frame[i].is_dynamic = true;
                continue;
            }

            if (p_frame->point_frame[i].range > 30.0)
            {
                p_frame->point_frame[i].is_undecided = true;
                points_undecided.push_back(p_frame->point_frame[i]);
                continue;
            }

            if (p_frame->point_frame[i].label == 0)
            {
                continue;
            }

            p_frame->point_frame[i].is_dynamic = true;
            continue;
        }   
    }
}