#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

class roi_curved_Node : public rclcpp::Node
{
public:
    roi_curved_Node() // 생성자
        : Node("roi_curved")
    {
    	// 파라미터 선언
        this->declare_parameter<std::string>("input_topic", "/sampling_points");
        this->declare_parameter<std::string>("output_topic", "/roi_points");
        this->declare_parameter<double>("angle_left");
        this->declare_parameter<double>("angle_right");
        this->declare_parameter<double>("distance_min");
        this->declare_parameter<double>("distance_max");
        this->declare_parameter<double>("height_limit");

	// 파라미터 대입
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        angle_left = this->get_parameter("angle_left").as_double();
        angle_right = this->get_parameter("angle_right").as_double();
        distance_min = this->get_parameter("distance_min").as_double();
        distance_max = this->get_parameter("distance_max").as_double();
        height_limit = this->get_parameter("height_limit").as_double();

	// 퍼블리셔 생성
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        
        // 구독자 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&roi_curved_Node::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) // 콜백 함수
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

	// 범위에 맞는 것 분류
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
         for (const auto& point : cloud->points) {
            double angle = atan2(point.y, point.x) * 180.0 / M_PI; 
            double distance = sqrt(point.x * point.x + point.y * point.y); 
            if (!(angle > angle_left && angle < angle_right) && distance >= distance_min &&distance <= distance_max && point.z < height_limit) { 
                cloud_filtered->points.push_back(point);
            } 
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, output);
        output.header.frame_id = msg->header.frame_id;
        output.header.stamp = this->now();

	// 토픽 퍼블리시
        publisher_->publish(output);
    }

    // 변수 선언
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    double angle_left;
    double angle_right;
    double distance_min;
    double distance_max;
    double height_limit;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); //ROS2 초기화
    rclcpp::spin(std::make_shared<roi_curved_Node>()); // 노드 실행
    rclcpp::shutdown(); // ROS2 종료
    return 0;
}
