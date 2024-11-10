#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"

class ransacNode : public rclcpp::Node
{
public:
    ransacNode() // 생성자
        : Node("ransac")
    {
    	// 파라미터 선언
        this->declare_parameter<std::string>("input_topic", "/roi_points");
        this->declare_parameter<std::string>("output_topic", "/ransac_points");
        this->declare_parameter<int>("max_iterations", 1000);
        this->declare_parameter<double>("distance_threshold", 0.10);

	// 파라미터 대입
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

	// 퍼블리셔 생성
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        
        // 구독자 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&ransacNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) // 콜백 함수
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
	// 파라미터 대입
        int max_iterations = this->get_parameter("max_iterations").as_int();
        double distance_threshold = this->get_parameter("distance_threshold").as_double();

	// RANSAC 세그멘테이션을 위한 객체 생성
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations); 
        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

	// 인라이어를 제외한 포인트 클라우드 추출
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);

	// 결과를 ROS 메시지로 변환
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = msg->header.frame_id;
        output.header.stamp = this->now();


	// 토픽 퍼블리시
        publisher_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // ROS2 초기화
    rclcpp::spin(std::make_shared<ransacNode>()); // 노드 실행   
    rclcpp::shutdown(); // ROS2 종료
    return 0;
}
