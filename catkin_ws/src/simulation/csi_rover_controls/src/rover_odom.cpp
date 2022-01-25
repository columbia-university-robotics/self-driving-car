//
// Created by neil on 2/14/20.
//

#include <algorithm>

#include "./rover_odom.h"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


namespace gazebo {

    // constructor
    RoverOdomPlugin::RoverOdomPlugin() {}

    // deconstructor
    RoverOdomPlugin::~RoverOdomPlugin() {
        delete nh;
        delete transform_broadcaster_;
    }

    // Load the controller
    void RoverOdomPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent = _parent;
        this->world = _parent->GetWorld();

        this->robot_namespace_ = "";
        if (!_sdf->HasElement("robotNamespace")) {
            ROS_INFO_NAMED("rover_odom", "RoverOdomPlugin Plugin missing <robotNamespace>, defaults to \"%s\"",
                           this->robot_namespace_.c_str());
        } else {
            this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        }
        
        this->odometry_topic = "odom";
        if (!_sdf->HasElement("odometryTopic")) {
            ROS_WARN_NAMED("rover_odom", "RoverOdomPlugin Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                           this->robot_namespace_.c_str(), this->odometry_topic.c_str());
        } else {
            this->odometry_topic = _sdf->GetElement("odometryTopic")->Get<std::string>();
        }

        this->odometry_frame = "odom";
        if (!_sdf->HasElement("odometryFrame")) {
            ROS_WARN_NAMED("rover_odom", "RoverOdomPlugin Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                           this->robot_namespace_.c_str(), this->odometry_frame.c_str());
        } else {
            this->odometry_frame = _sdf->GetElement("odometryFrame")->Get<std::string>();
        }

        this->robot_base_frame = "base_footprint";
        if (!_sdf->HasElement("robotBaseFrame")) {
            ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                           this->robot_namespace_.c_str(), this->robot_base_frame.c_str());
        } else {
            this->robot_base_frame = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
        }

        this->update_rate_ = 100.0;
        if (!_sdf->HasElement("updateRate")) {
            ROS_WARN_NAMED("rover_odom", "RoverOdomPlugin Plugin (ns = %s) missing <updateRate>, defaults to %f",
                           this->robot_namespace_.c_str(), this->update_rate_);
        } else {
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
        }

        this->covariance_x_ = 0.0001;
        if (!_sdf->HasElement("covariance_x"))
            ROS_WARN_NAMED("rover_odom", "RoverOdomPlugin Plugin (ns = %s) missing <covariance_x>, defaults to %f",
                           this->robot_namespace_.c_str(), covariance_x_);
        else
            covariance_x_ = _sdf->GetElement("covariance_x")->Get<double>();

        this->covariance_y_ = 0.0001;
        if (!_sdf->HasElement("covariance_y"))
            ROS_WARN_NAMED("rover_odom", "RoverOdomPlugin Plugin (ns = %s) missing <covariance_y>, defaults to %f",
                           this->robot_namespace_.c_str(), covariance_y_);
        else
            covariance_y_ = _sdf->GetElement("covariance_y")->Get<double>();

        this->covariance_yaw_ = 0.01;
        if (!_sdf->HasElement("covariance_yaw")) {
            ROS_WARN_NAMED("rover_odom", "RoverOdomPlugin Plugin (ns = %s) missing <covariance_yaw>, defaults to %f",
                           this->robot_namespace_.c_str(), covariance_yaw_);
        } else {
            covariance_yaw_ = _sdf->GetElement("covariance_yaw")->Get<double>();
        }

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0)
            this->update_period_ = 1.0 / this->update_rate_;
        else
            this->update_period_ = 0.0;

        last_update_time_ = this->world->SimTime();

        // initialize ros
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized()){
            ROS_FATAL_STREAM_NAMED("rover_odom", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        nh = new ros::NodeHandle(this->robot_namespace_);

        ROS_INFO_NAMED("rover odom plugin", "Starting rover odom Plugin (ns = %s)", this->robot_namespace_.c_str());

        tf_prefix = tf::getPrefixParam(*nh);
        transform_broadcaster_ = new tf::TransformBroadcaster();

        odometry_publisher = nh->advertise<nav_msgs::Odometry>(odometry_topic, 3);

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&RoverOdomPlugin::UpdateChild, this));
    }

    void RoverOdomPlugin::UpdateChild() {

        common::Time current_time = this->world->SimTime();

        double seconds_since_last_update = (current_time - last_update_time_).Double();
        if (seconds_since_last_update > update_period_) {

            publish_odometry(seconds_since_last_update);

            last_update_time_+= common::Time(update_period_);
        }
    }

    void RoverOdomPlugin::FiniChild() {
        nh->shutdown();
    }

    void RoverOdomPlugin::publish_odometry(double step_time) {

        ros::Time current_time = ros::Time::now();
        std::string odom_frame = tf::resolve(tf_prefix, odometry_frame);
        std::string base_footprint_frame = tf::resolve(tf_prefix, robot_base_frame);

//        ROS_WARN_NAMED("odom", "%s", odom_frame.c_str());
//        ROS_WARN_NAMED("base", "%s", base_footprint_frame.c_str());

        // TODO create some non-perfect odometry!
        // getting data for base_footprint to odom transform
        ignition::math::Pose3d pose = this->parent->WorldPose();

        tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

        tf::Transform base_footprint_to_odom(qt, vt);

        transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time,
                odom_frame, base_footprint_frame));

        // publish odom topic
        odom_.pose.pose.position.x = pose.Pos().X();
        odom_.pose.pose.position.y = pose.Pos().Y();

        odom_.pose.pose.orientation.x = pose.Rot().X();
        odom_.pose.pose.orientation.y = pose.Rot().Y();
        odom_.pose.pose.orientation.z = pose.Rot().Z();
        odom_.pose.pose.orientation.w = pose.Rot().W();
        odom_.pose.covariance[0] = this->covariance_x_;
        odom_.pose.covariance[7] = this->covariance_y_;
        odom_.pose.covariance[14] = 1000000000000.0;
        odom_.pose.covariance[21] = 1000000000000.0;
        odom_.pose.covariance[28] = 1000000000000.0;
        odom_.pose.covariance[35] = this->covariance_yaw_;

        // get velocity in /odom frame
        ignition::math::Vector3d linear;

        linear = this->parent->WorldLinearVel();
        odom_.twist.twist.angular.z = this->parent->WorldAngularVel().Z();

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
        odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
        odom_.twist.covariance[0] = this->covariance_x_;
        odom_.twist.covariance[7] = this->covariance_y_;
        odom_.twist.covariance[14] = 1000000000000.0;
        odom_.twist.covariance[21] = 1000000000000.0;
        odom_.twist.covariance[28] = 1000000000000.0;
        odom_.twist.covariance[35] = this->covariance_yaw_;

        odom_.header.stamp = current_time;
        odom_.header.frame_id = odom_frame;
        odom_.child_frame_id = base_footprint_frame;

        odometry_publisher.publish(odom_);
    }

    GZ_REGISTER_MODEL_PLUGIN(RoverOdomPlugin)
}
