#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include "nicp_package/VecPointsWn.h"
#include "nicp_package/VecPointsWn.h"
#include <iostream>
#include <tf2/utils.h>
#include "inc/eigen_nicp_2d.h"

static void removeLeadingCharacters(std::string &str, const char charToRemove) {
    str.erase(0, std::min(str.find_first_not_of(charToRemove), str.size() - 1));
}

class ric{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ric():
    tf2_(buffer_){
      sub_= n_.subscribe("/cloud", 100, &ric::callback,this);
      icp_pub_ = n_.advertise<nicp_package::VecPointsWn> ("/icp_topic", 100, false);
    }
  using ContainerType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;
  using TreeNodeType = TreeNode_<ContainerType::iterator>;
  
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
    //ln: here starts the process of extracting the normals from the scan
    TreeNodeType  kd_tree(vec.begin(), vec.end(), 20);

    //ln: neighborsp will contain pointers, neighbors the values
    TreeNodeType::AnswerType neighborsp;
    ContainerType neighbors;

    float ball_radius=1;
    Eigen::Matrix<float,2,1> mean;
    Eigen::Matrix<float,2,2> cov;

    //ln: when a new message arrives, the current "moving" becomes the fixed and in moving we put the new data
    fixed_=moving_;
    moving_.clear();
    PointNormal2f p_to_add;
    ContainerType added; //ln: used later on to check if a point in vec has been already assigned its normal.
    added.reserve(vec.size());
    //ln:
    //structures with 2d points (Vector2f):vec, added, neighbors, neighborsp
    //structures with 4d points (PointNormal2f): fixed_,moving_
    for(const auto& val : vec){
      //ln: if this val was a neighbors of one prevoiusly considered, skip it.
      if(std::find(added.begin(),added.end(),val) != added.end())
        continue;
      neighborsp.clear();
      neighbors.clear();

      kd_tree.fullSearch(neighborsp, val, ball_radius);

      //ln: if you find 1 neighbor, since we are looking in a set where our query point is, it means that
      //actually no neighbor is found. Hence we skip it.                                                
      if(neighborsp.size() == 1)
        continue;
      convert(neighbors,neighborsp); //ln: converts array of pointers to array of elements, needed for computing the cov
      computeMeanAndCovariance(mean,cov,neighbors.begin(),neighbors.end());
      const auto pn=smallestEigenVector(cov);

      //ln: we add all the neighbors and thei normal to moving_. We add the neighb also to "added"
      //because we want to consider them only once.
      for(const auto& neighb : neighbors){

      //ln: if a neighbor (neighb) of the current value (val) was also a neighbor of another value, 
      //we already added it and we skip it
        if(std::find(added.begin(),added.end(),neighb) != added.end())
          continue;
        
        p_to_add.head<2>()=neighb;
        p_to_add.tail<2>()=pn;
        moving_.push_back(p_to_add);
        added.push_back(neighb);
      }
    }
    //if moving==fixed it means the robot has not moved since last time, so useless to run icp.
    //if fixed.size() is 0 it means that this is the first message received, so we just have to wait for another message.
    nicp_package::VecPointsWn message;
    nicp_package::PointWnormal pwn;
    if(fixed_.size() && moving_ != fixed_){
      for(const auto& point : fixed_){
        pwn.x=point(0);
        pwn.y=point(1);
        pwn.nx=point(2);
        pwn.ny=point(3);
        message.fixed.push_back(pwn);
      }
      for(const auto& point : moving_){
        pwn.x=point(0);
        pwn.y=point(1);
        pwn.nx=point(2);
        pwn.ny=point(3);
        message.moving.push_back(pwn);
      }
      icp_pub_.publish(message);
    }
  }
protected:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  geometry_msgs::TransformStamped transformStamped_;
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  NICP::ContainerType fixed_;
  NICP::ContainerType moving_;
  ros::Publisher icp_pub_;
  inline void convert(ContainerType& array,const TreeNodeType::AnswerType& array_of_p){
    for(const auto& val:array_of_p)
      array.push_back(*val);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver");
  ric r;
  ros::spin();

  return 0;
}
