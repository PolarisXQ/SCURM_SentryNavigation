#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl/registration/gicp.h"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"

#include <ctime>
#include <Eigen/Core>
#include <Eigen/Geometry>

using Feature = pcl::FPFHSignature33;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointNormal = pcl::PointCloud<pcl::Normal>;
using FPFHFeature = pcl::PointCloud<Feature>;

class SACIAGICPNode : public rclcpp::Node
{
public:
    SACIAGICPNode() : Node("sac_ia_gicp_node")
    {
        // parameters
        std::string target_pcd_file;
        this->declare_parameter<int>("num_threads", 8);
        this->declare_parameter<int>("k_serach_source", 10);
        this->declare_parameter<int>("k_serach_target", 10);
        this->declare_parameter<double>("voxel_grid_leaf_size_source", 2.0);
        this->declare_parameter<double>("voxel_grid_leaf_size_target", 2.0);
        this->declare_parameter<double>("sac_ia_min_sample_distance", 0.5);
        this->declare_parameter<int>("sac_ia_correspondence_randomness", 6);
        this->declare_parameter<int>("sac_ia_num_samples", 3);
        this->declare_parameter<double>("icp_max_correspondence_distance", 10.0);
        this->declare_parameter<int>("icp_max_iteration", 100);
        this->declare_parameter<double>("icp_transformation_epsilon", 0.01);
        this->declare_parameter<double>("icp_euclidean_fitness_epsilon", 0.01);
        this->declare_parameter<std::string>("target_pcd_file", "/path/to/target.pcd");
        this->declare_parameter<double>("fitness_score_thre", 0.0);
        this->declare_parameter<int>("mode", 0);
        this->declare_parameter<int>("max_optimize_times", 3);

        this->get_parameter("num_threads", num_threads_);
        this->get_parameter("k_serach_source", k_serach_source_);
        this->get_parameter("k_serach_target", k_serach_target_);
        this->get_parameter("voxel_grid_leaf_size_source", voxel_grid_leaf_size_source_);
        this->get_parameter("voxel_grid_leaf_size_target", voxel_grid_leaf_size_target_);
        this->get_parameter("sac_ia_min_sample_distance", sac_ia_min_sample_distance_);
        this->get_parameter("sac_ia_correspondence_randomness", sac_ia_correspondence_randomness_);
        this->get_parameter("sac_ia_num_samples", sac_ia_num_samples_);
        this->get_parameter("icp_max_correspondence_distance", icp_max_correspondence_distance_);
        this->get_parameter("icp_max_iteration", icp_max_iteration_);
        this->get_parameter("icp_transformation_epsilon", icp_transformation_epsilon_);
        this->get_parameter("icp_euclidean_fitness_epsilon", icp_euclidean_fitness_epsilon_);
        this->get_parameter("target_pcd_file", target_pcd_file);
        this->get_parameter("fitness_score_thre", fitness_score_thre_);
        this->get_parameter("mode", mode); // 0 - keep running; 1 - run when needed
        this->get_parameter("max_optimize_times", max_optimize_times_);

        // guranatee that the lastest message is processed since the callback function is time-consuming
        rclcpp::CallbackGroup::SharedPtr callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group;
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "source_cloud", rclcpp::SensorDataQoS(), std::bind(&SACIAGICPNode::sourceCloudCallback, this, std::placeholders::_1), options);
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/relocalization_result", 10);
        pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/icp_cloud", 10);
        template_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/template_cloud", 10);
        if (mode == 1)
        {
            trigger_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "/need_recovery", 1, [this](const std_msgs::msg::Bool::SharedPtr msg)
                {
                    if (msg->data)
                    {
                        RCLCPP_INFO(this->get_logger(), "Trigger received");
                        triggered_ = true;
                    } });
        }

        RCLCPP_INFO(this->get_logger(), "SAC-IA GICP node initialized");

        RCLCPP_INFO(this->get_logger(), "Loading target cloud from %s", target_pcd_file.c_str());

        target_cloud_ = std::make_shared<PointCloud>();
        pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_file, *target_cloud_);

        pcl::Indices indices_tgt;
        pcl::removeNaNFromPointCloud(*target_cloud_, *target_cloud_, indices_tgt);

        pcl::VoxelGrid<pcl::PointXYZ> vg_target;
        vg_target.setLeafSize(voxel_grid_leaf_size_target_, voxel_grid_leaf_size_target_, voxel_grid_leaf_size_target_);
        vg_target.setInputCloud(target_cloud_);
        vg_target_cloud_ = std::make_shared<PointCloud>();
        vg_target.filter(*vg_target_cloud_);
        RCLCPP_INFO(this->get_logger(), "Downsampled target cloud from %ld to %ld points", target_cloud_->size(), vg_target_cloud_->size());

        tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        target_fpfh_ = computeFPFHFeature(vg_target_cloud_, k_serach_target_, tree_);
        RCLCPP_INFO(this->get_logger(), "Target FPFH feature computed");

        pcl::toROSMsg(*vg_target_cloud_, target_cloud_msg);
        target_cloud_msg.header.frame_id = "map";
    }

private:
    void sourceCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (mode == 1 && !triggered_)
        {
            return;
        }

        // ------------------------ SAC-IA ------------------------
        PointCloud::Ptr source_cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *source_cloud);

        pcl::Indices indices_src;
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);

        // use voxelgrid to downsample pointcloud
        pcl::VoxelGrid<pcl::PointXYZ> vg_source;
        vg_source.setLeafSize(voxel_grid_leaf_size_source_, voxel_grid_leaf_size_source_, voxel_grid_leaf_size_source_);
        vg_source.setInputCloud(source_cloud);
        PointCloud::Ptr vg_source_cloud(new PointCloud);
        vg_source.filter(*vg_source_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled source cloud from %ld to %ld points", source_cloud->size(), vg_source_cloud->size());

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
        FPFHFeature::Ptr source_fpfh = computeFPFHFeature(vg_source_cloud, k_serach_source_, tree_src);

        start_sac_ia = clock();
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        sac_ia.setInputSource(vg_source_cloud);
        sac_ia.setSourceFeatures(source_fpfh);
        sac_ia.setInputTarget(vg_target_cloud_);
        sac_ia.setTargetFeatures(target_fpfh_);

        // min_sample_distance the minimum distances between samples
        sac_ia.setMinSampleDistance(sac_ia_min_sample_distance_);

        // k the number of neighbors to use when selecting a random feature correspondence. A higher value will add more randomness to the feature matching.
        sac_ia.setCorrespondenceRandomness(sac_ia_correspondence_randomness_);

        // nr_samples the number of samples to use during each iteration
        sac_ia.setNumberOfSamples(sac_ia_num_samples_);

        PointCloud::Ptr sac_ia_cloud(new PointCloud);
        sac_ia.align(*sac_ia_cloud);
        end_sac_ia = clock();
        RCLCPP_INFO(this->get_logger(), "calculate time is: %.6f s", float(end_sac_ia - start_sac_ia) / CLOCKS_PER_SEC);
        RCLCPP_INFO(this->get_logger(), "SAC-IA has converged, score: %.6f", sac_ia.getFitnessScore());
        RCLCPP_INFO(this->get_logger(), "Transformation matrix: \n \t %.6f \t %.6f \t %.6f \t %.6f \n \t %.6f \t %.6f \t %.6f \t %.6f \n \t %.6f \t %.6f \t %.6f \t %.6f \n \t %.6f \t %.6f \t %.6f \t %.6f",
                    sac_ia.getFinalTransformation()(0, 0), sac_ia.getFinalTransformation()(0, 1), sac_ia.getFinalTransformation()(0, 2), sac_ia.getFinalTransformation()(0, 3),
                    sac_ia.getFinalTransformation()(1, 0), sac_ia.getFinalTransformation()(1, 1), sac_ia.getFinalTransformation()(1, 2), sac_ia.getFinalTransformation()(1, 3),
                    sac_ia.getFinalTransformation()(2, 0), sac_ia.getFinalTransformation()(2, 1), sac_ia.getFinalTransformation()(2, 2), sac_ia.getFinalTransformation()(2, 3),
                    sac_ia.getFinalTransformation()(3, 0), sac_ia.getFinalTransformation()(3, 1), sac_ia.getFinalTransformation()(3, 2), sac_ia.getFinalTransformation()(3, 3));

        // ------------------------ GICP ------------------------
        Eigen::Matrix4f initial_guess = sac_ia.getFinalTransformation();
        double fitness_score = 100.0;
        bool icp_converged = false;
        int optimize_times = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        while (fitness_score > fitness_score_thre_ && !icp_converged)
        {
            start_icp = clock();
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(vg_source_cloud);
            icp.setInputTarget(vg_target_cloud_);

            // Set the maximum distance threshold between two correspondent points in
            // source <-> target. If the distance is larger than this threshold, the points will
            // be ignored in the alignment process
            icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);

            icp.setMaximumIterations(icp_max_iteration_);

            // The method considers a point to be an inlier, if the distance between the target
            // data index and the transformed source index is smaller than the given inlier
            // distance threshold. The value is set by default to 0.05m.
            icp.setRANSACIterations(100);
            icp.setRANSACOutlierRejectionThreshold(0.05);

            // Set the maximum allowed Euclidean error between two consecutive steps in
            // the ICP loop, before the algorithm is considered to have converged. The error is
            // estimated as the sum of the differences between correspondences in an Euclidean
            // sense, divided by the number of correspondences.
            // icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_); // 在认为算法已经收敛之前，设置 ICP 循环中两个连续步骤之间允许的最大欧几里得误差。误差被估计为欧几里得意义上的对应之间的差异之和，除以对应的数量（即为欧几里得距离平均值）。

            // Set the transformation epsilon (maximum allowable translation squared
            // difference between two consecutive transformations) in order for an optimization to
            // be considered as having converged to the final solution.
            icp.setTransformationEpsilon(icp_transformation_epsilon_);

            // icp.setTransformationRotationEpsilon(1e-8);

            icp.align(*icp_cloud, initial_guess); //! align the source cloud to the target cloud using the transformation from SAC-IA
            end_icp = clock();
            fitness_score = icp.getFitnessScore();
            icp_converged = icp.hasConverged();
            RCLCPP_INFO(this->get_logger(), "calculate time is: %.6f s", float(end_icp - start_icp) / CLOCKS_PER_SEC);
            if (icp_converged)
                RCLCPP_INFO(this->get_logger(), "GICP has converged, score: %.6f", fitness_score);
            else
                RCLCPP_INFO(this->get_logger(), "GICP has not converged, score: %.6f", fitness_score);
            RCLCPP_INFO(this->get_logger(), "Transformation matrix:\n \t %.6f \t %.6f \t %.6f \t %.6f \n \t %.6f \t %.6f \t %.6f \t %.6f \n \t %.6f \t %.6f \t %.6f \t %.6f \n \t %.6f \t %.6f \t %.6f \t %.6f",
                        icp.getFinalTransformation()(0, 0), icp.getFinalTransformation()(0, 1), icp.getFinalTransformation()(0, 2), icp.getFinalTransformation()(0, 3),
                        icp.getFinalTransformation()(1, 0), icp.getFinalTransformation()(1, 1), icp.getFinalTransformation()(1, 2), icp.getFinalTransformation()(1, 3),
                        icp.getFinalTransformation()(2, 0), icp.getFinalTransformation()(2, 1), icp.getFinalTransformation()(2, 2), icp.getFinalTransformation()(2, 3),
                        icp.getFinalTransformation()(3, 0), icp.getFinalTransformation()(3, 1), icp.getFinalTransformation()(3, 2), icp.getFinalTransformation()(3, 3));

            if (icp_converged && fitness_score < fitness_score_thre_)
            {
                RCLCPP_INFO(this->get_logger(), "ICP fitness score is lower than the threshold, pose is published!");
                Eigen::Matrix4f transformation = icp.getFinalTransformation();
                geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "map";
                pose_msg.pose.pose.position.x = transformation(0, 3);
                pose_msg.pose.pose.position.y = transformation(1, 3);
                pose_msg.pose.pose.position.z = transformation(2, 3);
                // rot mtx to quaternion
                Eigen::Matrix3f rot_mtx = transformation.block<3, 3>(0, 0);
                Eigen::Quaternionf q(rot_mtx);
                pose_msg.pose.pose.orientation.x = q.x();
                pose_msg.pose.pose.orientation.y = q.y();
                pose_msg.pose.pose.orientation.z = q.z();
                pose_msg.pose.pose.orientation.w = q.w();
                publisher_->publish(pose_msg);

                if (mode == 1)
                    triggered_ = false;

                break;
            }
            else if (icp_converged && optimize_times < max_optimize_times_)
            {
                initial_guess = icp.getFinalTransformation();
                RCLCPP_INFO(this->get_logger(), "ICP converged, but fitness score is higher than the threshold, continue to optimize!");
                optimize_times++;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "ICP did not converge for this scan, pose is not published!");
                break;
            }
        }

        // publish the aligned pointcloud
        sensor_msgs::msg::PointCloud2 icp_cloud_msg;
        pcl::toROSMsg(*icp_cloud, icp_cloud_msg);
        icp_cloud_msg.header.stamp = this->now();
        icp_cloud_msg.header.frame_id = "map";
        pcd_publisher_->publish(icp_cloud_msg);

        target_cloud_msg.header.stamp = this->now();
        template_publisher_->publish(target_cloud_msg);
    }

    FPFHFeature::Ptr computeFPFHFeature(const PointCloud::Ptr &input_cloud, int k_serch, const pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree)
    {
        // use OpenMP to parallelize the computation
        PointNormal::Ptr normals(new PointNormal);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_norm(new pcl::search::KdTree<pcl::PointXYZ>());
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
        n.setInputCloud(input_cloud);
        n.setNumberOfThreads(8);
        n.setSearchMethod(tree_norm);
        n.setKSearch(k_serch);
        n.compute(*normals);

        FPFHFeature::Ptr fpfh(new FPFHFeature);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZ>());
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
        f.setNumberOfThreads(8);
        f.setInputCloud(input_cloud);
        f.setInputNormals(normals);
        f.setSearchMethod(tree_fpfh);
        f.setKSearch(k_serch);
        f.compute(*fpfh);
        return fpfh;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr template_publisher_;
    PointCloud::Ptr target_cloud_;
    FPFHFeature::Ptr target_fpfh_;
    PointCloud::Ptr vg_target_cloud_;
    PointCloud::Ptr icp_cloud_;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

    int num_threads_;
    int k_serach_source_;
    int k_serach_target_;
    double voxel_grid_leaf_size_source_;
    double voxel_grid_leaf_size_target_;
    double sac_ia_min_sample_distance_;
    int sac_ia_correspondence_randomness_;
    int sac_ia_num_samples_;
    double icp_max_correspondence_distance_;
    int icp_max_iteration_;
    double icp_transformation_epsilon_;
    double icp_euclidean_fitness_epsilon_;
    double fitness_score_thre_;
    clock_t start_sac_ia, end_sac_ia, start_icp, end_icp;
    sensor_msgs::msg::PointCloud2 target_cloud_msg;
    int mode;
    int max_optimize_times_;
    bool triggered_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<SACIAGICPNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
