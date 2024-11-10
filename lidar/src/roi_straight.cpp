#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

class roi_straight_Node : public rclcpp::Node
{
public:
    roi_straight_Node()
        : Node("roi_straight")
    {
        this->declare_parameter<std::string>("input_topic", "/sampling_points");
        this->declare_parameter<std::string>("output_topic", "/roi_points");
        this->declare_parameter<double>("height_limit");
        this->declare_parameter<double>("left_limit");
        this->declare_parameter<double>("right_limit");
        this->declare_parameter<double>("front_limit");
        this->declare_parameter<double>("back_limit");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        height_limit = this->get_parameter("height_limit").as_double();
        left_limit = this->get_parameter("left_limit").as_double();
        right_limit = this->get_parameter("right_limit").as_double();
        front_limit = this->get_parameter("front_limit").as_double();
        back_limit = this->get_parameter("back_limit").as_double();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&roi_straight_Node::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
         for (const auto& point : cloud->points) {
            if (point.z < height_limit && point.y >= left_limit && point.y <= right_limit && point.x <= front_limit && point.x >= back_limit) { 
                cloud_filtered->points.push_back(point);
            } 
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, output);
        output.header.frame_id = msg->header.frame_id;
        output.header.stamp = this->now();

        publisher_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    double height_limit;
    double left_limit;
    double right_limit;
    double front_limit;
    double back_limit;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<roi_straight_Node>());
    rclcpp::shutdown();
    return 0;
}
