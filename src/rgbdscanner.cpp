//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
//TF
#include <tf/transform_listener.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//CPP
#include <iostream>
//Costmap
#include <simple_layers/grid_layer.h>

// Type definitions.
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class rgbdscanner {
  ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  ros::Publisher pub;
  ros::Subscriber sub;
  tf::TransformListener tf_listener;

  // Initialize publisher, to publish a "2D-Point-Cloud".
  void initPublisher() {
    this->pub = nh2.advertise<PointCloud> ("/rgbdscanner/points", 1);
  }

  public:
  
  rgbdscanner(ros::NodeHandle &n1, ros::NodeHandle &n2) {
    nh1 = n1;
    nh2 = n1;
  }

  // Initialize subscriber for 3D point cloud and publisher and start
  // downprojection of 3D points.
  void scan() {
    this->initPublisher();
    this->sub = this->nh1.subscribe ("/camera/depth_registered/points", 1, &rgbdscanner::cloud_cb, this);
  }

  // Callbackfunction for subscriber.
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud) {
    // Point clouds.
    PointCloud PC;
    PointCloud PC_filtered;
    PointCloud PC_reduced;
    PointCloud PC_new;

    // Transform to pcl.
    pcl::fromROSMsg(*cloud,  PC);

    PC_reduced.header.frame_id = "/base_link";
    PC_new.header.frame_id = "/base_link";
    
    // Filter real values from point cloud.
    std::vector<int> indexRealValues;
    pcl::removeNaNFromPointCloud(PC, PC, indexRealValues);

    for (int i = 0; i < PC.points.size(); i+=100) {
      PC_reduced.points.push_back(PC.points[i]);
    }

    // Time stamp.
    ros::Time now = ros::Time::now();
    for (int i = 0; i < PC_reduced.size(); i++) {

      // Transform PCL points into stamped ROS vector form.
      tf::Stamped<tf::Vector3> tf_point_in = 
        tf::Stamped<tf::Vector3>(tf::Vector3(PC_reduced.points[i].x, 
              PC_reduced.points[i].y, 
              PC_reduced.points[i].z), 
          now, cloud->header.frame_id);

      tf::Stamped<tf::Vector3> tf_point_out;

      // Transform point from the /camera frame into the /base_link frame.
      try {
        tf_listener.waitForTransform("/base_link", cloud->header.frame_id, now, ros::Duration(3.0));
        tf_listener.transformPoint("/base_link", tf_point_in, tf_point_out);
      } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
        ros::shutdown();
      }
      if (tf_point_out.getZ() > 0.02) {
        // No floor.
        pcl::PointXYZ p;
        p.x = tf_point_out.getX();
        p.y = tf_point_out.getY();
        p.z = 0;
        PC_new.points.push_back(p);
      }
      
    }

    sensor_msgs::PointCloud2 mesg;
    pcl::toROSMsg(PC_new, mesg);
    this->pub.publish(mesg);
  }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rgbdscanner");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  
  // Start rgbdscanner.
  ROS_INFO("Start...");
  rgbdscanner scanner(nh, nh2);
  scanner.scan();

  // Spin
  while(ros::ok()) {
    ros::spin ();
  }
  return 0;
}
