#ifndef _MPC_H_
#define _MPC_H_

#include <vector>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "carstatemsgs/CarState.h"
#include "mpc_controller/Polynome.h"
#include "mpc_controller/traj_anal.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>

class MPCState{
    public:
        double x = 0;
        double y = 0;
        double v = 0;
        double theta = 0;
        double w = 0;
};

class TrajPoint{
    public:
        double x = 0;
        double y = 0;
        double v = 0;
        double a = 0;
        double theta = 0;
        double w = 0;
};

class MpcController{
    
    private:
        ros::NodeHandle nh_;

        ros::Timer cmd_timer_;
        double cmd_timer_rate_;
        void CmdCallback(const ros::TimerEvent& event);
        
        ros::Subscriber odom_sub_;
        ros::Subscriber traj_sub_;
        void OdomCallback(const carstatemsgs::CarState::ConstPtr& msg);
        void TrajCallback(const mpc_controller::Polynome::ConstPtr& msg);

        ros::Publisher sequence_pub_;
        void sequencePub();

        ros::Publisher cmd_pub_;
        
        // debug
        ros::Publisher Ref_path_pub_;
        ros::Publisher cmd_path_pub_;
        ros::Publisher Ref_velocity_pub_;
        ros::Publisher Real_velocity_pub_;

        TrajAnal traj_;
        TrajAnal new_traj_;
        double new_traj_start_time_;

        // 收敛条件之一  若迭代过程中控制量的变化量小于该值则认为收敛
        double du_th;
        // 收敛条件之一  若迭代次数大于该值则认为收敛
        int max_iter;
        // MPC预测的时间的步长
        double dt;
        // MPC预测的时间的步数
        int T;
        // 由于计算时间导致的滞后，将第delay_num个控制量发出，一般取值为计算时间除以步长
        int delay_num;
        // 跟踪误差权重，分别为x y v yaw
        std::vector<double> Q;
        // 控制量权重，目的是使得控制量尽可能小，分别为v w
        std::vector<double> R;
        // 控制量变化权重，目的是使得控制量变化尽可能小，分别为v w
        std::vector<double> Rd;
        // 运动学限制
        // 最大角速度，角加速度，单位步长的最大角速度变化量
        double max_omega;
        double max_domega;
        double max_comega;
        // 最大速度，最小速度，单位步长的最大速度变化量，加速度
        double max_speed;
        double min_speed;
        double max_cv;
        double max_accel;

        // MPC 线性状态传递函数矩阵
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::VectorXd C;

        // MPC状态矩阵
        MPCState xbar[500];
        // 参考状态
        Eigen::MatrixXd xref;
        // 参考输入
        Eigen::MatrixXd dref;
        // 输出buff
        Eigen::MatrixXd output;
        // 储存上一次的输出，用作MPC初值
        Eigen::MatrixXd last_output;
        // MPC计算结果
        std::vector<Eigen::Vector2d> output_buff;

        Eigen::Vector2d current_output;
        double Last_plan_time = -1;

        // 状态变量
        // 是否有机器人定位
        bool has_odom;
        // 是否已经收到轨迹
        bool receive_traj_ = false;
        // 当前机器人状态
        MPCState now_state;
        ros::Time now_state_time_;
        // 用于预测机器人未来的状态
        MPCState xopt[500];

        // 轨迹开始时间
        double start_time;
        // 轨迹持续时间
        double traj_duration; 
        
        // 是否到达目标
        bool at_goal = false;
        std::vector<TrajPoint> P;

        // 最大计算时间，目前的判达条件
        double max_mpc_time_;



        //将输入角度转到-pi到pi区间内
        void normlize_theta(double& th);

        // 将xref里面的yaw角变化平滑化，避免发生2pi突变
        void smooth_yaw(void);

        // 根据输入的状态获得线性模型的参数
        void getLinearModel(const MPCState& s);

        // 根据输入的状态和控制量获得输出状态
        void stateTrans(MPCState& s, double a, double yaw_dot);

        // 根据输入状态和控制量b获得所有状态
        void predictMotion(void);
        void predictMotion(MPCState* b);

        // 速度控制的MPC模型
        void solveMPCV(void);

        // 解MPC问题输出
        void getCmd(void);
        
        // 发送控制指令
        void cmdPub();
        
        // 获得MPC的一系列参考点 // 待定
        void getRefPoints(const int T, double dt);

        


    public:
        MpcController(const ros::NodeHandle &nh);

};



#endif