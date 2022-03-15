#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "transform/VecPointsWn.h"
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include "inc/eigen_nicp_2d.h"
#include "math.h"
#include <sensor_msgs/PointCloud.h>

//ln: retrieving the quaternion from the isometry matrix.
//Needed to broadcast a tf.
tf2::Quaternion getQ(const Eigen::Isometry2f& iso){
  float th=atan2f(iso(1,0),iso(0,0));
  tf2::Quaternion q(0,0,sinf(th/2),cosf(th/2));
  return q;
}
class icp_run{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  icp_run(){
    transf_story_.setIdentity();
    sub_= n_.subscribe("/icp_topic", 100, &icp_run::icpCallback,this);
    //ln: topic on which a pointCloud will be published. It will be used by another node
    //to build a map of the environment, using the tf that is published.
    pub_ = n_.advertise<sensor_msgs::PointCloud> ("/icp_data", 100, false);
  }
  void icpCallback(const transform::VecPointsWn& msg){
    old_.clear();
    new_.clear();
    PointNormal2f pwn;
    
    geometry_msgs::Point32 point;
    sensor_msgs::PointCloud cloud;
    //gg: the timestamp will be used by the node subscribed to icp_data
    //to apply the tf on the appropriate set of points.
    cloud.header.stamp=msg.stamp;

    for(const auto& p:msg.fixed){
      pwn(0)=p.x;
      pwn(1)=p.y;
      pwn(2)=p.nx;
      pwn(3)=p.ny;      
      old_.push_back(pwn);

      point.x=p.x;
      point.y=p.y;
      point.z=0;
      cloud.points.push_back(point);
    }
    for(const auto& p:msg.moving){
      pwn(0)=p.x;
      pwn(1)=p.y;
      pwn(2)=p.nx;
      pwn(3)=p.ny;
      new_.push_back(pwn);
    }
    NICP icp(old_,new_,20);
    icp.run(100);

    //gg: creating the structures needed to broadcast a tf.
    //The transform published is the one resulting from icp.
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = msg.stamp;

    transformStamped.header.frame_id = "odom";

    transformStamped.child_frame_id = "rotation";

    Eigen::Isometry2f iso=icp.X();
    transformStamped.transform.translation.x = iso(0,2);
    transformStamped.transform.translation.y = iso(1,2);
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion transf_quat=getQ(iso);

    transformStamped.transform.rotation.x = transf_quat.x();
    transformStamped.transform.rotation.y = transf_quat.y();
    transformStamped.transform.rotation.z = transf_quat.z();
    transformStamped.transform.rotation.w = transf_quat.w();

    br.sendTransform(transformStamped);
    pub_.publish(cloud);
  }
private:
  ros::NodeHandle n_;
  ros::Publisher tf2_pub_;
  ros::Subscriber sub_;
  NICP::ContainerType old_;
  NICP::ContainerType new_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_run");

    icp_run icp_runner;

    ros::spin();

    return 0;
}
