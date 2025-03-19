#include <execution>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "rm_interface/srv/save_pcd.hpp"

#include "occupied_mapping/grid_map.h"

namespace occupied_mapping{
class OccupiedMapping: public rclcpp::Node{
private:
    typedef std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>> SynchronizerCloudOdom;
    SynchronizerCloudOdom syn_chronizer_odom_add_;

    GridMap<PointType>::Ptr grid_map_;
    bool sliding_en_{false}, is_init_frame_name_{false}, is_pub_FOV_{false};
    std::string lidar_frame_{"body"}, odom_frame_{"camera_init"};
    OdomType ext_tf_;
    Eigen::Vector4f FOV_range_;
    Eigen::Vector3f odom_pos_for_visual_;
    BoxPointType box_for_visual_;
    float max_range_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_cloud_add_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_add_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_del_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_del_;

    std::deque<OdomType> buffer_odom_del_;
    std::deque<std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>> buffer_cloud_del_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_grid_map_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_FOV_;
    rclcpp::TimerBase::SharedPtr timer_visual_, timer_del_;

    rclcpp::Service<rm_interface::srv::SavePCD>::SharedPtr service_save_pcd_;

    void callBackCloudOdomAdd(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odom){
        // debug_tools::Timer t_0;
        OdomType odom_add;
        odom_add.pos(0) = msg_odom->pose.pose.position.x;
        odom_add.pos(1) = msg_odom->pose.pose.position.y;
        odom_add.pos(2) = msg_odom->pose.pose.position.z;
        odom_add.q.w() = msg_odom->pose.pose.orientation.w;
        odom_add.q.x() = msg_odom->pose.pose.orientation.x;
        odom_add.q.y() = msg_odom->pose.pose.orientation.y;
        odom_add.q.z() = msg_odom->pose.pose.orientation.z;
        odom_add.R = odom_add.q.matrix();

        odom_pos_for_visual_ = odom_add.pos;

        auto cloud_in = std::make_shared<pcl::PointCloud<PointType>>();
        pcl::fromROSMsg(*msg_cloud, *cloud_in);
        if(cloud_in->points.size() <= 0) return;

        if(!is_init_frame_name_){
            lidar_frame_ = msg_cloud->header.frame_id;
            is_init_frame_name_ = true;
        }

        auto cloud = std::make_shared<pcl::PointCloud<PointType>>();
        for(auto& p : cloud_in->points){
            if(grid_map_->checkFOV(p)) cloud->push_back(p);
        }

        std::for_each(std::execution::par, cloud->points.begin(), cloud->points.end(), [&](PointType& p){
            Eigen::Vector3f point_origin(p.x, p.y, p.z), point_world;
            point_world = odom_add.R * (ext_tf_.R*point_origin + ext_tf_.pos) + odom_add.pos;
            p.x = point_world(0);
            p.y = point_world(1);
            p.z = point_world(2);
        });

        if(sliding_en_) grid_map_->slideMap(odom_add);
        grid_map_->addPoints(cloud->points, odom_add);
        // debug_tools::Timer t_1(t_0); t_1.durLog("callBackCloudOdom Add", "ms");
    }

    void callBackCloudOdomDel(){
        if(this->buffer_cloud_del_.size() <= 0 || this->buffer_odom_del_.size() <= 0) return;
        // debug_tools::Timer t_0;
        auto cloud_del = buffer_cloud_del_.at(0);
        buffer_cloud_del_.pop_front();

        auto odom_del = buffer_odom_del_.at(0);
        buffer_odom_del_.pop_front();

        std::for_each(std::execution::par, cloud_del->points.begin(), cloud_del->points.end(), [&](PointType& p){
            Eigen::Vector3f point_origin(p.x, p.y, p.z), point_world;
            point_world = odom_del.R * (ext_tf_.R*point_origin + ext_tf_.pos) + odom_del.pos;
            p.x = point_world(0);
            p.y = point_world(1);
            p.z = point_world(2);
        });

        grid_map_->deletePoints(cloud_del->points, odom_del);
        // debug_tools::Timer t_1(t_0); t_1.durLog("callBackCloudOdom Del", "ms");
    }

    void callBackVisualization(){
        debug_tools::Timer t_0;
        PointVector point_vec;
        BoxPointType box_visual{box_for_visual_.vertex_min[0]+odom_pos_for_visual_(0), box_for_visual_.vertex_max[0]+odom_pos_for_visual_(0), 
                                box_for_visual_.vertex_min[1]+odom_pos_for_visual_(1), box_for_visual_.vertex_max[1]+odom_pos_for_visual_(1),
                                box_for_visual_.vertex_min[2]+odom_pos_for_visual_(2), box_for_visual_.vertex_max[2]+odom_pos_for_visual_(2)};
        grid_map_->getPointsToVec(point_vec, 8, box_visual);
        PointCloud cloud_pcl;
        cloud_pcl.clear();
        cloud_pcl.assign(point_vec.begin(), point_vec.end());
        sensor_msgs::msg::PointCloud2 msg_grid_cloud;
        pcl::toROSMsg(cloud_pcl, msg_grid_cloud);
        msg_grid_cloud.header.frame_id = odom_frame_;
        msg_grid_cloud.header.stamp = this->now();
        pub_grid_map_->publish(msg_grid_cloud);
        debug_tools::Timer t_1(t_0); t_1.log("cloud visualization", "ms");
    }

    void callBackPublishFOV() {  
        visualization_msgs::msg::Marker marker;  
        marker.header.frame_id = lidar_frame_;
        marker.header.stamp = this->now();  
        marker.ns = "fov";  
        marker.id = 0;  
        marker.type = visualization_msgs::msg::Marker::LINE_LIST; // 使用线段列表  
        marker.scale.x = 0.02; // 线宽  
        marker.color.r = 77/256.0;  
        marker.color.g = 56/256.0;  
        marker.color.b = 243/256.0;  
        marker.color.a = 1.0;  

        // 计算矩形的四个定点  
        geometry_msgs::msg::Point top_left, top_right, bottom_left, bottom_right;  

        // 计算顶部和底部的高度  
        float top_z = max_range_ * sin(FOV_range_[1]);   // 垂直方向最大角度  
        float bottom_z = max_range_ * sin(FOV_range_[0]); // 垂直方向最小角度  
        
        // 水平方向计算左边和右边的点  
        float horizontal_angle_left = FOV_range_[2]; // 水平最小角度  
        float horizontal_angle_right = FOV_range_[3]; // 水平最大角度  
        
        top_left.x = max_range_ * cos(horizontal_angle_left);  
        top_left.y = max_range_ * sin(horizontal_angle_left);  
        top_left.z = top_z; // 上边的Z坐标  
        
        top_right.x = max_range_ * cos(horizontal_angle_right);  
        top_right.y = max_range_ * sin(horizontal_angle_right);  
        top_right.z = top_z; // 上边的Z坐标  
        
        bottom_left.x = max_range_ * cos(horizontal_angle_left);  
        bottom_left.y = max_range_ * sin(horizontal_angle_left);  
        bottom_left.z = bottom_z; // 下边的Z坐标  
        
        bottom_right.x = max_range_ * cos(horizontal_angle_right);  
        bottom_right.y = max_range_ * sin(horizontal_angle_right);  
        bottom_right.z = bottom_z; // 下边的Z坐标  

        // 矩形的四条边  
        marker.points.push_back(top_left);  
        marker.points.push_back(top_right); // 上边  
            
        marker.points.push_back(top_right);  
        marker.points.push_back(bottom_right); // 右边  

        marker.points.push_back(bottom_right);  
        marker.points.push_back(bottom_left); // 下边  

        marker.points.push_back(bottom_left);  
        marker.points.push_back(top_left); // 左边  

        // 连接原点与矩形的四个顶点  
        geometry_msgs::msg::Point origin; origin.x = 0, origin.y = 0, origin.z = 0;
        marker.points.push_back(origin); // 原点  
        marker.points.push_back(top_left);  

        marker.points.push_back(origin); // 原点  
        marker.points.push_back(top_right);  

        marker.points.push_back(origin); // 原点  
        marker.points.push_back(bottom_left);  
        
        marker.points.push_back(origin); // 原点  
        marker.points.push_back(bottom_right);  

        pub_FOV_->publish(marker);
    }  

public:
    OccupiedMapping(std::string node_name, rclcpp::NodeOptions option): Node(node_name, option){
        lidar_frame_ = declare_parameter<std::string>("frame_lidar", "body");
        odom_frame_ = declare_parameter<std::string>("frame_odom", "camera_init");
        float resolution = declare_parameter<float>("resolution", 0.01);
        auto box_mapping_vec = declare_parameter<std::vector<float>>("box_mapping", std::vector<float>({-10, 10, -11, 11, -1, 2}));  //单位m
        BoxPointType box(box_mapping_vec);
        auto fov_range_vec = declare_parameter<std::vector<float>>("FOV_range", std::vector<float>{-7, 52, -45, 45}); // 角度制
        FOV_range_ = Eigen::Vector4f(fov_range_vec[0]/180.0*M_PI, fov_range_vec[1]/180.0*M_PI, fov_range_vec[2]/180.0*M_PI, fov_range_vec[3]/180.0*M_PI);
        max_range_ = declare_parameter<float>("max_range", 10.0);
        float sliding_threshold = declare_parameter<float>("sliding_threshold", 0.2);
        auto box_visual_vec = declare_parameter<std::vector<float>>("box_visual", std::vector<float>({-5, 5, -5, 5, -1, 2}));
        box_for_visual_ = box_visual_vec;

        is_pub_FOV_ = declare_parameter<bool>("is_pub_FOV", true);

        grid_map_ = std::make_shared<GridMap<PointType>>();
        grid_map_->setMapParams(resolution, box, FOV_range_, max_range_, sliding_threshold);

        std::string topic_cloud_in_add = declare_parameter<std::string>("topic_cloud_in_add", "/cloud_registered_body");
        std::string topic_cloud_in_del = declare_parameter<std::string>("topic_cloud_in_del", "/m_detector_cloud_body");
        std::string topic_odom_in = declare_parameter<std::string>("topic_odom", "/fast_lio_odom");
        std::string topic_grid_out = declare_parameter<std::string>("topic_grid_out", "/grid_out_visual");
        std::string topic_save_pcd = declare_parameter<std::string>("topic_save_pcd", "/occupied_mapping_save_pcd");

        pub_grid_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_grid_out, rclcpp::QoS(1));
        std::string topic_FOV_visual = declare_parameter<std::string>("topic_FOV_visual", "/FOV_range_visual");
        if(is_pub_FOV_) pub_FOV_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_FOV_visual, 10);

        auto ext_tran_ = declare_parameter<std::vector<float>>("extrinsic_T", std::vector<float>({0.0, 0.0, 0.0}));
        auto ext_qua_ = declare_parameter<std::vector<float>>("extrinsic_Q", std::vector<float>({0.0, 0.0, 0.0, 1.0}));
        ext_tf_.pos = Eigen::Vector3f(ext_tran_[0], ext_tran_[1], ext_tran_[2]);
        ext_tf_.q = Eigen::Quaternion<float>(ext_qua_[3], ext_qua_[2], ext_qua_[1], ext_qua_[0]);
        ext_tf_.R = ext_tf_.q.toRotationMatrix();

        sub_cloud_add_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
        sub_cloud_add_->subscribe(this, topic_cloud_in_add);

        sub_odom_add_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
        sub_odom_add_->subscribe(this, topic_odom_in);

        syn_chronizer_odom_add_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, 
                    nav_msgs::msg::Odometry>>(*sub_cloud_add_, *sub_odom_add_, 10);
        syn_chronizer_odom_add_->registerCallback(std::bind(&OccupiedMapping::callBackCloudOdomAdd, this, std::placeholders::_1, std::placeholders::_2));

        sub_odom_del_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_odom_in,
            rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                OdomType odom_temp;
                odom_temp.pos(0) = msg->pose.pose.position.x;
                odom_temp.pos(1) = msg->pose.pose.position.y;
                odom_temp.pos(2) = msg->pose.pose.position.z;
                odom_temp.q.w() = msg->pose.pose.orientation.w;
                odom_temp.q.x() = msg->pose.pose.orientation.x;
                odom_temp.q.y() = msg->pose.pose.orientation.y;
                odom_temp.q.z() = msg->pose.pose.orientation.z;
                odom_temp.R = odom_temp.q.matrix();
                buffer_odom_del_.push_back(odom_temp);
            }
        );

        sub_cloud_del_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_cloud_in_del, 
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
                auto cloud_in = std::make_shared<pcl::PointCloud<PointType>>();
                pcl::fromROSMsg(*msg, *cloud_in);
                this->buffer_cloud_del_.push_back(cloud_in);
            }
        );

        service_save_pcd_ = this->create_service<rm_interface::srv::SavePCD>(
            topic_save_pcd, 
            [this](const std::shared_ptr<rm_interface::srv::SavePCD::Request> request, const std::shared_ptr<rm_interface::srv::SavePCD::Response> response){
                auto cloud = std::make_shared<PointCloud>();
                grid_map_->getPointsToCloud(cloud);
                pcl::PCDWriter writer;
                response->memory_gb = static_cast<float>(grid_map_->getMemoryGB());
                response->result = writer.write<PointType>(request->file_path, *cloud);
            }
        );

        timer_del_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&OccupiedMapping::callBackCloudOdomDel, this));

        timer_visual_ = this->create_wall_timer(std::chrono::milliseconds(200), [this](){
            this->callBackVisualization();
            if(is_init_frame_name_ && is_pub_FOV_) this->callBackPublishFOV();
        });
    }
};
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto occupied_mapping_node = std::make_shared<occupied_mapping::OccupiedMapping>("occupied_mapping_node", rclcpp::NodeOptions());
    rclcpp::spin(occupied_mapping_node);
    rclcpp::shutdown();
    return 0;
}