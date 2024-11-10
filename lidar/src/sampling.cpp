#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

class samplingNode : public rclcpp::Node
{
public:
    samplingNode() // 생성자
    : Node("sampling")
    {
    	// 파라미터 선언
        this->declare_parameter<float>("leaf_size"); 
        
        // 퍼블리셔 생성
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sampling_points", 10);
        
        // QoS 설정
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        // 구독자 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", qos, std::bind(&samplingNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const // 콜백 함수
    {
    	// pcd 포인터 생성 및 변환
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *cloud);
	
	// 필터 객체 생성 및 입력 클라우드 설정
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
	
	// 파라미터 파일에서 leaf_size 가져와 설정        
        float leaf_size;
        this->get_parameter("leaf_size", leaf_size); 
        sor.setLeafSize(leaf_size, leaf_size, leaf_size); 
	
	// 필터링 수행
        sor.filter(*cloud_filtered);
	
	// 토픽 퍼블리시
        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(*cloud_filtered, output);
        publisher_->publish(output);
    }
		
    // 구독자와 퍼블리셔 포인터	
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // ROS2 초기화
    rclcpp::spin(std::make_shared<samplingNode>()); // 노드 실행
    rclcpp::shutdown(); // ROS2 종료
    return 0;
}
