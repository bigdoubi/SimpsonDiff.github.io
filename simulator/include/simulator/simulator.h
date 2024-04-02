#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <carstatemsgs/CarState.h>
#include <carstatemsgs/KinematicState.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

class Simulation{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber Pose_sub_;
        ros::Publisher Pose_pub_;
        ros::Publisher Rviz_pub_;
        ros::Publisher Rviz_Arrow_pub_;
        ros::Publisher KinematicState_pub_;

        ros::Timer Pose_pub_timer_;
        double Pose_pub_rate_;
        ros::Timer State_Propa_timer_;
        double State_Propa_rate_;

        ros::Time current_time_;
        Eigen::Vector3d current_XYTheta_;
        Eigen::Vector4d current_SVAJ_;
        Eigen::Vector4d current_YOAJ_;


        double length_;
        double width_;
        double height_;
        double wheel_base_;
        double tread_;
        double front_suspension_;
        double rear_suspension_;


        double max_v_;
        double max_omega_;
        double max_a_;
        double max_domega_;
        double max_centripetal_acc_;

        double offset_;

    public:
        Simulation(ros::NodeHandle nh){
            nh_ = nh;
            
            double start_x, start_y, start_yaw;
            nh_.param<double>(ros::this_node::getName()+"/start_x",start_x,0.0);
            nh_.param<double>(ros::this_node::getName()+"/start_y",start_y,0.0);
            nh_.param<double>(ros::this_node::getName()+"/start_yaw",start_yaw,0.0);
            current_XYTheta_ << start_x, start_y, start_yaw;
            current_SVAJ_.setZero();
            current_YOAJ_.setZero();
            current_YOAJ_[0] = start_yaw;

            nh_.param<double>(ros::this_node::getName()+"/length",length_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/width",width_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/height",height_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/wheel_base",wheel_base_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/tread",tread_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/front_suspension",front_suspension_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/front_suspension",rear_suspension_,0.0);
            offset_ = length_/2 - rear_suspension_;

            nh_.param<double>(ros::this_node::getName()+"/max_vel",max_v_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/max_omega",max_omega_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/max_acc",max_a_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/max_domega",max_domega_,0.0);
            nh_.param<double>(ros::this_node::getName()+"/max_centripetal_acc",max_centripetal_acc_,0.0);

            int State_Propa_rate, Pose_pub_rate;
            nh_.param<int>(ros::this_node::getName()+"/State_Propa_rate",State_Propa_rate,50);
            nh_.param<int>(ros::this_node::getName()+"/Pose_pub_rate",Pose_pub_rate,50);
            State_Propa_rate_ = 1.0/State_Propa_rate;
            Pose_pub_rate_ = 1.0/Pose_pub_rate;

            Pose_sub_ = nh_.subscribe<carstatemsgs::CarState>("/simulation/PoseSub",1,&Simulation::PoseSubCallback,this,ros::TransportHints().unreliable());
            Pose_pub_ = nh_.advertise<carstatemsgs::CarState>("/simulation/PosePub",1);
            Rviz_pub_ = nh_.advertise<visualization_msgs::Marker>("simulation/RvizCar",1);
            Rviz_Arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("simulation/RvizCarArrow",1);
            KinematicState_pub_ = nh_.advertise<carstatemsgs::KinematicState>("/simulation/KinematicState",1);

            State_Propa_timer_ = nh_.createTimer(ros::Duration(State_Propa_rate_),&Simulation::StatePropaCallback, this);
            Pose_pub_timer_ = nh_.createTimer(ros::Duration(Pose_pub_rate_),&Simulation::PosePubCallback, this);

            current_time_ = ros::Time::now();


        }

        void PoseSubCallback(const carstatemsgs::CarState::ConstPtr &msg){
            // 一般来说只会接收速度层及以上的信息
            current_SVAJ_.tail(3) << msg->v, msg->a, msg->js;
            current_YOAJ_.tail(3) << msg->omega, msg->alpha, msg->jyaw;

            current_time_ = msg->Header.stamp;
        }

        void StatePropaCallback(const ros::TimerEvent& event){
            // 目前只用速度进行状态转换
            current_XYTheta_.x() += current_SVAJ_[1]*State_Propa_rate_*cos(current_XYTheta_.z());
            current_XYTheta_.y() += current_SVAJ_[1]*State_Propa_rate_*sin(current_XYTheta_.z());
            current_XYTheta_.z() += current_YOAJ_[1]*State_Propa_rate_;

            current_SVAJ_[0] += current_SVAJ_[1] * State_Propa_rate_;
            current_YOAJ_[0] += current_YOAJ_[1] * State_Propa_rate_;
        }

        void PosePubCallback(const ros::TimerEvent& event){
            carstatemsgs::CarState carstate;
            carstate.Header.frame_id = "base";
            carstate.Header.stamp = ros::Time::now();
            carstate.x = current_XYTheta_.x();
            carstate.y = current_XYTheta_.y();
            carstate.yaw = current_XYTheta_.z();

            carstate.s = current_SVAJ_[0];
            carstate.v = current_SVAJ_[1];
            carstate.a = current_SVAJ_[2];
            carstate.js = current_SVAJ_[3];


            carstate.omega = current_YOAJ_[1];
            carstate.alpha = current_YOAJ_[2];
            carstate.jyaw = current_YOAJ_[3];

            Pose_pub_.publish(carstate);

            carstatemsgs::KinematicState kinematicstate;
            kinematicstate.Header.frame_id = "base";
            kinematicstate.Header.stamp = ros::Time::now();
            kinematicstate.centripetal_acc = current_YOAJ_[1] * current_SVAJ_[1];
            kinematicstate.max_centripetal_acc = max_centripetal_acc_;
            kinematicstate.min_centripetal_acc = -max_centripetal_acc_;
            kinematicstate.moment = fabs(current_SVAJ_[1]) * max_omega_ + fabs(current_YOAJ_[1]) * max_v_;
            kinematicstate.max_moment = max_v_ * max_omega_;
            kinematicstate.min_moment = -max_v_ * max_omega_;
            KinematicState_pub_.publish(kinematicstate);

            Eigen::Vector2d offset;
            Eigen::Matrix2d R;
            double yaw = current_XYTheta_.z();
            R<<cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "base";
            marker.ns = "rvizCar";
            marker.lifetime = ros::Duration();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 1;
            marker.color.r = 0.512;
            marker.color.g = 0.654;
            marker.color.b = 0.854;
            marker.header.stamp = ros::Time::now();

            // 车体
            offset << offset_, 0;
            marker.scale.x = width_;
            marker.scale.y = length_;
            marker.scale.z = height_;
            marker.id = 0;
            marker.pose.position.x = (R*offset).x() + current_XYTheta_.x();
            marker.pose.position.y = (R*offset).y() + current_XYTheta_.y();
            marker.pose.position.z = height_/2;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            Rviz_pub_.publish(marker);

            // 箭头
            marker.header.frame_id = "base";
            marker.ns = "rvizCarArrow";
            marker.lifetime = ros::Duration();
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.2;
            marker.scale.y = 0.06;
            marker.scale.z = 0.06;
            marker.color.a = 1;
            marker.color.r = 0.512;
            marker.color.g = 0.654;
            marker.color.b = 0.854;
            marker.header.stamp = ros::Time::now();
            marker.id = 0;
            marker.pose.position.x = current_XYTheta_.x();
            marker.pose.position.y = current_XYTheta_.y();
            marker.pose.position.z = height_/2;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            Rviz_Arrow_pub_.publish(marker);
        }

};

#endif