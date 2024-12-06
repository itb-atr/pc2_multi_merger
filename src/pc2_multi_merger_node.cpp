#include <cstring>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;
using namespace pcl;

class PC2Merger : public rclcpp::Node
{
public:
  PC2Merger();
private:
  void pc2_cb_(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, const std::string& topic);
  void pc2_topic_parser_();

	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscribers_;
	std::vector<bool> clouds_modified_;

	std::vector<pcl::PCLPointCloud2> clouds_;
	std::vector<string> input_topics_;

	string destination_frame_;
	string cloud_destination_topic_;
	string pc2_topics_;
};

PC2Merger::PC2Merger() : Node("pc2_multi_merger")
{
	this->declare_parameter<std::string>("destination_frame", "merged_cloud_frame");
	this->declare_parameter<std::string>("cloud_destination_topic", "~/merged_cloud");
	this->declare_parameter<std::string>("pc2_topics", "");

	this->get_parameter("destination_frame", destination_frame_);
	this->get_parameter("cloud_destination_topic", cloud_destination_topic_);
	this->get_parameter("pc2_topics", pc2_topics_);

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	this->pc2_topic_parser_();

	point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic_, rclcpp::SensorDataQoS());
}

void PC2Merger::pc2_topic_parser_()
{
	std::map<std::string, std::vector<std::string>> topics;

	istringstream iss(pc2_topics_);
	set<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), inserter<set<string>>(tokens, tokens.begin()));
	std::vector<string> tmp_input_topics;

	while (!tokens.empty())
	{
		RCLCPP_INFO(this->get_logger(), "Waiting for topics ...");
		sleep(1);

		topics = this->get_topic_names_and_types();

		for (const auto &topic_it : topics)
		{
			std::vector<std::string> topic_types = topic_it.second;

			if (std::find(topic_types.begin(), topic_types.end(), "sensor_msgs/msg/PointCloud2") != topic_types.end() && tokens.erase(topic_it.first) > 0)
			{
				tmp_input_topics.push_back(topic_it.first);
			}
		}
	}

	sort(tmp_input_topics.begin(), tmp_input_topics.end());
	auto last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	// Do not re-subscribe if the topics are the same
	if ((tmp_input_topics.size() != input_topics_.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics_.begin()))
	{
    input_topics_ = tmp_input_topics;

		if (!input_topics_.empty())
		{
			cloud_subscribers_.resize(input_topics_.size());
			clouds_modified_.resize(input_topics_.size());
			clouds_.resize(input_topics_.size());
			RCLCPP_INFO(this->get_logger(), "Subscribing to topics\t%ld", cloud_subscribers_.size());
			for (std::vector<int>::size_type i = 0; i < input_topics_.size(); ++i)
			{
				// workaround for std::bind https://github.com/ros2/rclcpp/issues/583
				std::function<void(const sensor_msgs::msg::PointCloud2 ::SharedPtr)> callback =
						std::bind(
                &PC2Merger::pc2_cb_,
                this, std::placeholders::_1, input_topics_[i]);
        cloud_subscribers_[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topics_[i], rclcpp::SensorDataQoS(), callback);
        clouds_modified_[i] = false;
				cout << input_topics_[i] << " ";
			}
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Not subscribed to any topic.");
		}
	}
}

void PC2Merger::pc2_cb_(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, const std::string& topic)
{
	sensor_msgs::msg::PointCloud2 transformed_cloud;
	try
	{
		tf_buffer_->lookupTransform(cloud->header.frame_id, destination_frame_, cloud->header.stamp, rclcpp::Duration(1, 0));
		pcl_ros::transformPointCloud(destination_frame_, *cloud, transformed_cloud, *tf_buffer_);
	}
	catch (tf2::TransformException &ex)
	{
		return;
	}

	for (std::vector<int>::size_type i = 0; i < input_topics_.size(); i++)
	{
		if (topic == input_topics_[i])
		{
			pcl_conversions::toPCL(transformed_cloud, clouds_[i]);
      clouds_modified_[i] = true;
		}
	}

	// Count how many scans we have
	std::vector<int>::size_type totalClouds = 0;
	for (auto && i : clouds_modified_)
	{
		if (i)
			totalClouds++;
	}

	// Go ahead only if all subscribed scans have arrived
	if (totalClouds == clouds_modified_.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds_[0];
    clouds_modified_[0] = false;

		for (std::vector<int>::size_type i = 1; i < clouds_modified_.size(); i++)
		{
			merged_cloud += clouds_[i];
      clouds_modified_[i] = false;
		}

		std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
		pcl_conversions::moveFromPCL(merged_cloud, *cloud_msg);
		point_cloud_publisher_->publish(*cloud_msg);
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PC2Merger>());
	rclcpp::shutdown();
	return 0;
}
