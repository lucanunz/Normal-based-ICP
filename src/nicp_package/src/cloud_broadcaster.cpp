#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

//ln: creating a node that transforms the laser scan in a cloud of points
//ros existing types are used (Point32, PointCloud) and a new topic is created (/cloud)
class cloud_br{
public:
  cloud_br(){
    sub_= n_.subscribe("/base_scan", 10, &cloud_br::laserCallback,this);
    point_cloud_publisher_ = n_.advertise<sensor_msgs::PointCloud> ("/cloud", 100, false);
  }
  void laserCallback(const sensor_msgs::LaserScan& msg){
    sensor_msgs::PointCloud cloud;
    cloud.header.seq=seq_;
    cloud.header.stamp=msg.header.stamp;
    cloud.header.frame_id=msg.header.frame_id;
    geometry_msgs::Point32 p;
    float th=msg.angle_min;
    for(int i=0;i<msg.ranges.size();++i){
      if(i!=0)
	      th+=msg.angle_increment;
      if(msg.ranges[i] == msg.range_max)
	      continue;
      p.x=msg.ranges[i]*cosf(th);
      p.y=msg.ranges[i]*sinf(th);
      p.z=0;
      cloud.points.push_back(p);
    }
    point_cloud_publisher_.publish(cloud);
    ++seq_;

  }
private:
  ros::NodeHandle n_;
  ros::Publisher point_cloud_publisher_;
  ros::Subscriber sub_;
  int seq_=0;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_br");

    cloud_br broadcaster;

    ros::spin();

    return 0;
}
