#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/centroid.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/kdtree/kdtree.h"

// 최소 거리와 최대 거리를 구하는 함수
void getMinMax3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& min_pt, pcl::PointXYZ& max_pt)
{
    min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
    max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

    for(auto& point: cloud->points)
    {
        min_pt.x = std::min(min_pt.x, point.x);
        min_pt.y = std::min(min_pt.y, point.y);
        min_pt.z = std::min(min_pt.z, point.z);

        max_pt.x = std::max(max_pt.x, point.x);
        max_pt.y = std::max(max_pt.y, point.y);
        max_pt.z = std::max(max_pt.z, point.z);
    }
}

class dbscanNode : public rclcpp::Node
{
public:
    dbscanNode() // 생성자
    : Node("dbscan")
    {
    
    	// 파라미터 선언
        this->declare_parameter<std::string>("input_topic", "/ransac_points");
        this->declare_parameter<std::string>("output_topic", "/dbscan_markers");
        this->declare_parameter<double>("cluster_tolerance"); 
        this->declare_parameter<int>("min_cluster_size");
        this->declare_parameter<int>("max_cluster_size");
        this->declare_parameter<double>("min_x");
        this->declare_parameter<double>("max_x");
        this->declare_parameter<double>("min_y");
        this->declare_parameter<double>("max_y");
        this->declare_parameter<double>("min_z");
        this->declare_parameter<double>("max_z");

	// 파라미터 대입
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

	// 퍼블리셔 생성
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);
        
        // 구독자 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&dbscanNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) // 콜백 함수
    {
    	// 변수 선언
        double cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;
        double min_x;
        double max_x;
        double min_y;
        double max_y;
        double min_z;
        double max_z;

	// 변수에 파라미터 대입
        this->get_parameter("cluster_tolerance", cluster_tolerance);
        this->get_parameter("min_cluster_size", min_cluster_size);
        this->get_parameter("max_cluster_size", max_cluster_size);
        this->get_parameter("min_x", min_x);
        this->get_parameter("max_x", max_x);
        this->get_parameter("min_y", min_y);
        this->get_parameter("max_y", max_y);
        this->get_parameter("min_z", min_z);
        this->get_parameter("max_z", max_z);

	// ROS2 메시지를 PCL 포인트 클라우드로 변환
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

	// KdTree 객체 생성
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

	// 클러스터 설정
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (cluster_tolerance); 
        ec.setMinClusterSize (min_cluster_size);
        ec.setMaxClusterSize (max_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices); // 클러스터 추출 수행

        visualization_msgs::msg::MarkerArray markers;
        int cluster_id = 0;
        
        // 클러스터 인덱스를 순회하며 조건에 맞는 객체를 마커로 추가
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud->points[*pit]); 
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            pcl::PointXYZ min_pt, max_pt;
            getMinMax3D(cloud_cluster, min_pt, max_pt);

            double max_diff_x = max_pt.x - min_pt.x;
            double max_diff_y = max_pt.y - min_pt.y;
            double max_diff_z = max_pt.z - min_pt.z;
            
	
	    // 설정된 범위 내에 있는지 확인		
            if ((min_x <= max_diff_x && max_diff_x <= max_x) &&
                (min_y <= max_diff_y && max_diff_y <= max_y) &&
                (min_z <= max_diff_z && max_diff_z <= max_z))
            {

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_cluster, centroid);

		// 마커 설정
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = this->now();
                marker.header.frame_id = msg->header.frame_id;
                marker.ns = "clusters";
                marker.id = cluster_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = centroid[0];
                marker.pose.position.y = centroid[1];
                marker.pose.position.z = centroid[2];
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.lifetime = rclcpp::Duration(std::chrono::nanoseconds(static_cast<int64_t>(0.50 * 1e9)));


                markers.markers.push_back(marker);
            }
        }

	// 토픽 퍼블리시
        publisher_->publish(markers);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // ROS2 초기화
    rclcpp::spin(std::make_shared<dbscanNode>()); // 노드 실행
    rclcpp::shutdown(); // ROS2 종료
    return 0;
}
