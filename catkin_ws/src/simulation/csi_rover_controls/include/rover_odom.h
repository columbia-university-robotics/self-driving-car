//
// Created by neil on 2/17/20.
//

#ifndef SRC_ROVER_ODOM_H
#define SRC_ROVER_ODOM_H

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>


namespace gazebo {

    class RoverOdomPlugin : public ModelPlugin {

    public:
        RoverOdomPlugin();
        ~RoverOdomPlugin();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
        virtual void UpdateChild();
        virtual void FiniChild();

    private:
        void publish_odometry(double step_time);

        physics::WorldPtr world;
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;

        tf::TransformBroadcaster *transform_broadcaster_;
        ros::Publisher odometry_publisher;
        ros::NodeHandle *nh;
        nav_msgs::Odometry odom_;

        double covariance_x_;
        double covariance_y_;
        double covariance_yaw_;

        std::string robot_namespace_;
        std::string odometry_topic;
        std::string odometry_frame;
        std::string robot_base_frame;
        std::string tf_prefix;

        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

    };
}

#endif //SRC_ROVER_ODOM_H
