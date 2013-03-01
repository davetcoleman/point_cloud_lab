#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>

namespace point_cloud_lab
{

class PointCloudLab
{
private:

  ros::NodeHandle nh_;

  ros::Subscriber point_cloud_sub_; // reads in data from sensor/rosbag file
  ros::Publisher filtered_pub_; // filtered point cloud for testing the algorithms
  ros::Publisher block_pose_pub_; // publishes to rviz
  tf::TransformListener tf_listener_;
  tf::StampedTransform transform_;
  bool has_transform_;

  geometry_msgs::PoseArray block_poses_;

  // Parameters of problem
  std::string base_link_;

  // Viewer
  pcl::visualization::CloudViewer viewer_;

public:

  PointCloudLab(bool pointless) : // we need a parameter here for some reason?
    nh_("~"),
    viewer_("Simple Cloud Viewer"),
    has_transform_(false)
  {
    // Parameters
    base_link_ = "/base_link";

    // Go ahead and fill out rviz arrow message
    block_poses_.header.stamp = ros::Time::now();
    block_poses_.header.frame_id = base_link_;

    // Subscribe to point cloud
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudLab::cloudCallback, this);

    // Publish a point cloud of filtered data that was not part of table
    filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Publish interactive markers for blocks
    block_pose_pub_ = nh_.advertise< geometry_msgs::PoseArray >("/", 1, true);


    ROS_INFO("Waiting to recieve point cloud...");
  }


  // Proccess the point clouds
  void cloudCallback( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    ROS_INFO_STREAM("Recieved callback");

    block_poses_.poses.clear();
    // Basic point cloud conversions ---------------------------------------------------------------

    // Convert from ROS to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    viewer_.showCloud(cloud);


    // YOUR CODE HERE ----------------------------------------------------------------------------------------
    // But actually you can edit anything in this file :)










    // Hint:
    // If you are using rviz you can publish "filtered" point clouds to see the intermediate steps of your
    // PCL process, e.g.:
    //   Publish a point cloud of filtered data that was not part of table
    //   filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);


    // Fill in the results
    double block_x = 0;
    double block_y = 0;
    double block_z = 0;
    double block_theta = 0;

    // END YOUR CODE HERE ------------------------------------------------------------------------------------

    // Add the results to a Rviz marker and publish it
    addBlock( block_x, block_y, block_z, block_theta );

    if(block_poses_.poses.size() > 0)
    {
      block_pose_pub_.publish(block_poses_);
      ROS_INFO_STREAM("Found " << block_poses_.poses.size() << " blocks this iteration");
    }
    else
    {
      ROS_INFO("Couldn't find any blocks this iteration!");
    }
  }

  void addBlock(double x, double y, double z, double angle)
  {
    geometry_msgs::Pose block_pose;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;

    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(angle, Eigen::Vector3d(0,0,1)));

    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();

    //ROS_INFO_STREAM("Added block: \n" << block_pose );
    block_poses_.poses.push_back(block_pose);
  }

};

};

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Starting Point Cloud Lab node");

  ros::init(argc, argv, "point_cloud_lab");

  point_cloud_lab::PointCloudLab detector(true);

  ros::spin();
  return 0;
}

