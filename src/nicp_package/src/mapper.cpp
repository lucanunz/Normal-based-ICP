#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>
#include <tf2/utils.h>
#include <Eigen/Core>

static void removeLeadingCharacters(std::string &str, const char charToRemove) {
    str.erase(0, std::min(str.find_first_not_of(charToRemove), str.size() - 1));
}

class ric{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ric():
    tf2_(buffer_){
    sub_= n_.subscribe("/cloud", 100, &ric::callback,this);
    }
  using Vector2f = Eigen::Vector2f;
  using ContainerType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;
    
  void callback(const sensor_msgs::PointCloud& msg){
    std::string src_frame=msg.header.frame_id;
    removeLeadingCharacters(src_frame,'/');
    //gg: rotate and translate the points from the laser to the odom frame 
    //and create a new structure, vec, of eigen elements (needed for kdtree).
    if(!buffer_.canTransform("odom",src_frame,msg.header.stamp))
      return;

    transformStamped_=buffer_.lookupTransform("odom",src_frame,msg.header.stamp);
    float th=tf2::getYaw(transformStamped_.transform.rotation);
    geometry_msgs::Point32 p;
    ContainerType vec;
    vec.reserve(msg.points.size());

    Vector2f p_eig;
    for(const auto& val : msg.points){
      p.x = val.x*cosf(th)-val.y*sinf(th);
      p.y = val.x*sinf(th)+val.y*cosf(th);
      
      p.x += transformStamped_.transform.translation.x;
      p.y += transformStamped_.transform.translation.y;

      p_eig(0)=p.x;
      p_eig(1)=p.y;

      vec.push_back(p_eig);
    }
  }
protected:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  geometry_msgs::TransformStamped transformStamped_;
  ros::NodeHandle n_;
  ros::Subscriber sub_;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver");
  ric r;
  ros::spin();

  return 0;
}
