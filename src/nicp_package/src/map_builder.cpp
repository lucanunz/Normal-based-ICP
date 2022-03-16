#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2/utils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "inc/rotations.h"
#include <tf2/utils.h>
#include <stdlib.h>
#include <string>
/*
This node represents an example of how the tf that is published by icp_runner can be used.
The aim is to build a map of the environment around the robot and save it
in the file map.txt in the home directory. This file is intended to be used in gnuplot.
Example of command in the gnuplot shell: plot "map.txt" using 1:2 w p pt 7 ps 0.1
*/
class mapper{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Vector2f = Eigen::Vector2f;
    mapper():
    tf2_(buffer_){
        transf_story_.setIdentity();
        sub_=n_.subscribe("/icp_data",100,&mapper::mapCallback,this);
        char* path;
        path=std::getenv("HOME");
        os.open(std::string(path)+"/map.txt");
    }
    void mapCallback(const sensor_msgs::PointCloud& msg){
        if(!buffer_.canTransform("odom","rotation",msg.header.stamp))
            return;
        transformStamped_=buffer_.lookupTransform("odom","rotation",msg.header.stamp);
        /* ln:
        From the translation vector and the quaternion we reconstruct the isometry matrix.
        We do this to easily update the history of transformations with just a matrix multiplication (last callback instruction).
        */
        float th=tf2::getYaw(transformStamped_.transform.rotation);

        Eigen::Isometry2f X;
        const Eigen::Matrix2f rot=Rtheta(th);
        X.setIdentity();
        X.linear()=rot;
        Vector2f trans_;
        trans_ << transformStamped_.transform.translation.x,transformStamped_.transform.translation.y;
        X.translation()=trans_;

        /* gg:
        In order to build the map, the cloud of points has to be premultiplied by the current transf_story 
        before updating it with the new tf.
        Note: the points sent by icp_runner through icp_data topic are the "old" ones 
        in order not to lose points that are only in the first scan.
        */
        Vector2f p;
        for(const auto& val : msg.points){
            p(0)=val.x;
            p(1)=val.y;
            os << (transf_story_*p).transpose() << std::endl;
        }
        //Every time a tf is available, we update the transf story with this multiplication
        transf_story_=transf_story_*X;
    }
private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    geometry_msgs::TransformStamped transformStamped_;
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    Eigen::Isometry2f transf_story_;
    std::ofstream os;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_builder");

    mapper mapper;

    ros::spin();

    return 0;
}