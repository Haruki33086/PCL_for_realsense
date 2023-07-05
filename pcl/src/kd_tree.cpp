#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h> // ダウンサンプリング用
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>

class Kd_tree : public rclcpp::Node
{
  public:
  	Kd_tree() : Node("kd_tree")
  	{
  	  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
"/camera/depth/color/points", 1, std::bind(&Kd_tree::callback, this, std::placeholders::_1));
	  // Twist型のメッセージをpublishするためのPublisherを宣言
	  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  	}
  private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc2);
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
	// Twist型のメッセージをpublishするためのPublisherを宣言
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

void Kd_tree::callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc2) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nan (new pcl::PointCloud<pcl::PointXYZ>); // NaN値あり
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // NaN値なし
	pcl::VoxelGrid<pcl::PointXYZ> vg_; // ダウンサンプリング用
	double leaf_size = 0.5; // ダウンサンプリングの解像度[m]

	auto twist = geometry_msgs::msg::Twist(); // Twist型の変数を宣言
	twist.linear.x = 0.0;
	twist.linear.z = 0.0;
	
	//sensor_msgs::PointCloud2からpcl::PointXYZに変換
	pcl::fromROSMsg(*pc2, *cloud_nan);

	// ダウンサンプリング実行 cloud_nanに行い、leaf_size[m]の解像度でダウンサンプリング、結果はcloud_nanに格納
	vg_.setInputCloud(cloud_nan);
	vg_.setLeafSize(leaf_size, leaf_size, leaf_size);
	vg_.filter(*cloud_nan);

	// NaN値が入ってるといろいろ面倒なので除去
	std::vector<int> nan_index;
	pcl::removeNaNFromPointCloud(*cloud_nan, *cloud, nan_index);

	// KD木を作っておく。近傍点探索とかが早くなる。
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	//近傍点探索に使うパラメータと結果が入る変数
	double radius = 0.5;  //半径r[m]
	std::vector<int> k_indices;  //範囲内の点のインデックスが入る
	std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
	unsigned int max_nn = 1;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
	
	pcl::PointXYZ p;  //中心座標[m]
	p.x = 0.0;
	p.y = 0.0;
	p.z = 0.0;

	//半径r以内にある点を探索
	tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
	
	if(k_indices.size() == 0) return;
	
	pcl::PointXYZ result = cloud->points[k_indices[0]];

	RCLCPP_INFO(this->get_logger(), "A nearest point of (0.5, 0.5) is...\nx: %lf, y:%lf, z:%lf", result.x, result.y, result.z);

	if(result.z < 0.2) {
	RCLCPP_INFO(this->get_logger(), "Obstacle distance is less than 0.20[m]");

	//速度を0にする
	publisher_->publish(twist);
	};
}

int main(int argc, char** argv){
	rclcpp::init(argc,argv);
	std::cout << "Launching NNS using KD-tree ..." << std::endl;
	rclcpp::spin(std::make_shared<Kd_tree>());
	rclcpp::shutdown();
	
	return 0;
}
