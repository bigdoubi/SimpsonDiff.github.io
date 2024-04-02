#ifndef _PLAN_MANAGER_HPP_
#define _PLAN_MANAGER_HPP_

#include "model_gazebo/world2oct.h"
#include "visualizer/visualizer.hpp"
#include "front_end/kino_astar.h"
#include "geometry_msgs/PoseStamped.h"
#include "back_end/optimizer.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "carstatemsgs/CarState.h"
#include "mpc_controller/Polynome.h"

#include <thread>

enum StateMachine{
  INIT,
  IDLE,
  PLANNING,
  // EXECUTE,
  // MPC,
  REPLAN,
  GOINGTOGOAL,
};

class PlanManager
{
  private:
    ros::NodeHandle nh_;

    std::shared_ptr<World2oct> world2oct_;
    std::shared_ptr<Visualizer> visualizer_;
    std::shared_ptr<KinoAstar> kinoastar_;
    std::shared_ptr<Gcopter> gcopter_;


    ros::Subscriber goal_sub_;
    ros::Subscriber current_state_sub_;
    ros::Timer main_thread_timer_;
    ros::Publisher cmd_pub_;
    ros::Publisher mpc_polynome_pub_;

    ros::Time current_time_;
    Eigen::Vector3d current_state_XYTheta_;
    Eigen::Vector3d current_state_VAJ_;
    Eigen::Vector3d current_state_OAJ_;

    double plan_start_time_;
    Eigen::Vector3d plan_start_state_XYTheta;
    Eigen::Vector3d plan_start_state_VAJ;
    Eigen::Vector3d plan_start_state_OAJ;

    Eigen::Vector3d goal_state_;

    ros::Time Traj_start_time_;
    double Traj_total_time_;

    bool have_geometry_;
    bool have_goal_;

    bool if_fix_final_;
    Eigen::Vector3d final_state_;

    bool if_mpc_;
    bool if_random_goal_;

    double replan_time_;
    
    double max_replan_time_;
    // std::chrono::time_point<std::chrono::steady_clock> startPlanTime, endPlanTime;

    StateMachine state_machine_ = StateMachine::INIT;

  public:
    PlanManager(ros::NodeHandle nh){
      nh_ = nh;
      
      world2oct_ = std::make_shared<World2oct>(nh);
      visualizer_ = std::make_shared<Visualizer>(nh);
      kinoastar_ = std::make_shared<KinoAstar>(nh,world2oct_);
      gcopter_ = std::make_shared<Gcopter>(Config(ros::NodeHandle("~")), nh_, world2oct_,kinoastar_);
      kinoastar_->init();

      goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,&PlanManager::goal_callback,this);
      current_state_sub_ = nh_.subscribe<carstatemsgs::CarState>("/simulation/PosePub",1,&PlanManager::GeometryCallback,this);
      main_thread_timer_ = nh_.createTimer(ros::Duration(0.001),&PlanManager::MainThread, this);
      cmd_pub_ = nh_.advertise<carstatemsgs::CarState>("/simulation/PoseSub",1);

      mpc_polynome_pub_ = nh_.advertise<mpc_controller::Polynome>("/planner/traj_poly", 1);

      have_geometry_ = false;
      have_goal_ = false;

      nh_.param<bool>("if_fix_final", if_fix_final_, false);
      if(if_fix_final_){
        nh_.param<double>("final_x", final_state_(0), 0.0);
        nh_.param<double>("final_y", final_state_(1), 0.0);
        nh_.param<double>("final_yaw", final_state_(2), 0.0);
      }

      nh_.param<bool>("if_mpc", if_mpc_, false);
      nh_.param<bool>("if_random_goal", if_random_goal_, false);

      nh_.param<double>("replan_time",replan_time_,10000.0);
      nh_.param<double>("max_replan_time", max_replan_time_, 1.0);

      int rand_num = time(nullptr);
      srand(rand_num);
      std::cout<<"    rand num for srand: "<<rand_num<<std::endl;

      state_machine_ = StateMachine::IDLE;

    // DEBUG 存储数据文件
    std::FILE *fp;
    char *fname = "SimpsonDiff.txt";  // 文件的相对路径
    fp = fopen(fname, "w+");  // 打开文件
    fclose(fp);  // 关闭文件

    }

    ~PlanManager(){ 
      world2oct_->~World2oct();
      visualizer_->~Visualizer();
      kinoastar_->~KinoAstar();
    }

    void init(){
      kinoastar_->init();
    }

    void printStateMachine(){
      if(state_machine_ == INIT) ROS_INFO("state_machine_ == INIT");
      if(state_machine_ == IDLE) ROS_INFO("state_machine_ == IDLE");
      if(state_machine_ == PLANNING) ROS_INFO("state_machine_ == PLANNING");
      if(state_machine_ == REPLAN) ROS_INFO("state_machine_ == REPLAN");
    }

    void GeometryCallback(const carstatemsgs::CarState::ConstPtr &msg){
      have_geometry_ = true;
      current_state_XYTheta_ << msg->x, msg->y, msg->yaw;
      current_state_VAJ_ << msg->v, msg->a, msg->js;
      current_state_OAJ_ << msg->omega, msg->alpha, msg->jyaw;
      current_time_ = msg->Header.stamp;
    }

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
      if(if_random_goal_){
        have_goal_ = !have_goal_;
      }
      else{
        std::cout<<"\n\n\n\n\n\n\n\n";
        std::cout<<"---------------------------------------------------------------"<<std::endl;
        std::cout<<"---------------------------------------------------------------"<<std::endl;

        std::cout<< "get goal!" <<std::endl;
        state_machine_ = StateMachine::IDLE;
        have_goal_ = true;
        goal_state_<<msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation);
        if(if_fix_final_) goal_state_ = final_state_;
        std::cout<<" goal state: "<<goal_state_.transpose()<<std::endl;

        std::cout<<"---------------------------------------------------------------"<<std::endl;
        std::cout<<"---------------------------------------------------------------"<<std::endl;
        std::cout<<"\n\n\n\n\n\n\n\n";
      }
    }

    void MainThread(const ros::TimerEvent& event){
      if(if_random_goal_ && have_geometry_ && have_goal_ && state_machine_ == StateMachine::IDLE){

        double finalx = rand()/double(RAND_MAX)*(world2oct_->x_upper - world2oct_->x_lower)+world2oct_->x_lower;
        double finaly = rand()/double(RAND_MAX)*(world2oct_->y_upper - world2oct_->y_lower)+world2oct_->y_lower;
        double finalyaw = rand()/double(RAND_MAX)*M_PI*2-M_PI;
        
        double goal_SDF = world2oct_->getDistWithGradBilinear(Eigen::Vector2d(finalx, finaly));  
        if( goal_SDF < 0.1 || goal_SDF > 1e8){
          return;
        }
        else{
          state_machine_ = StateMachine::IDLE;
          goal_state_<<finalx, finaly, finalyaw;
          std::cout<<"\n\n\n\n\n\n\n\n";
          std::cout<<"---------------------------------------------------------------"<<std::endl;
          std::cout<<"---------------------------------------------------------------"<<std::endl;
          std::cout<<" goal state: "<<goal_state_.transpose()<<std::endl;
          std::cout<<"---------------------------------------------------------------"<<std::endl;
          std::cout<<"---------------------------------------------------------------"<<std::endl;
          std::cout<<"\n\n\n\n\n\n\n\n";
        }
      }
      else if(!have_geometry_ || !have_goal_) return;

      if(state_machine_ == StateMachine::IDLE || 
          ((state_machine_ == StateMachine::PLANNING||state_machine_ == StateMachine::REPLAN) 
            && (ros::Time::now() - Traj_start_time_).toSec() > replan_time_)){
        // 停止状态的话进入规划
        if(state_machine_ == StateMachine::IDLE){
          state_machine_ = StateMachine::PLANNING;
          plan_start_time_ = -1;
          plan_start_state_XYTheta = current_state_XYTheta_;
          plan_start_state_VAJ = current_state_VAJ_;
          plan_start_state_OAJ = current_state_OAJ_;
          // startPlanTime = std::chrono::steady_clock::now();
        } 
        // 规划状态下使用预测距离进行重规划
        else if(state_machine_ == StateMachine::PLANNING || state_machine_ == StateMachine::REPLAN){
          
          state_machine_ = StateMachine::REPLAN;

          // startPlanTime = std::chrono::steady_clock::now();
          double current = ros::Time::now().toSec();
          gcopter_->get_the_predicted_state(current + max_replan_time_ - plan_start_time_, plan_start_state_XYTheta, plan_start_state_VAJ, plan_start_state_OAJ);
          plan_start_time_ = current + max_replan_time_;

          // 这里是不是改成时间判断终点会好一点
          if((plan_start_state_XYTheta - goal_state_).head(2).squaredNorm() + fmod(fabs((plan_start_state_XYTheta - goal_state_)[2]), 2.0 * M_PI)*0.3 < 0.1){
            state_machine_ = StateMachine::GOINGTOGOAL;
            return;
          }
        } 
        
        ROS_INFO("\033[32;40m \n\n\n\n\n-------------------------------------start new plan------------------------------------------ \033[0m");
        
        visualizer_->finalnodePub(plan_start_state_XYTheta, goal_state_);
        ROS_INFO("init_state_: %.10f  %.10f  %.10f", plan_start_state_XYTheta(0), plan_start_state_XYTheta(1), plan_start_state_XYTheta(2));
        ROS_INFO("goal_state_: %.10f  %.10f  %.10f", goal_state_(0), goal_state_(1), goal_state_(2));
        std::cout<<"<arg name=\"start_x_\" value=\""<< plan_start_state_XYTheta(0) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"start_y_\" value=\""<< plan_start_state_XYTheta(1) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"start_yaw_\" value=\""<< plan_start_state_XYTheta(2) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_x_\" value=\""<< goal_state_(0) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_y_\" value=\""<< goal_state_(1) <<"\"/>"<<std::endl;
        std::cout<<"<arg name=\"final_yaw_\" value=\""<< goal_state_(2) <<"\"/>"<<std::endl;

        std::cout<<"plan_start_state_VAJ: "<<plan_start_state_VAJ.transpose()<<std::endl;
        std::cout<<"plan_start_state_OAJ: "<<plan_start_state_OAJ.transpose()<<std::endl;


    std::FILE *fp;
    char *fname = "SimpsonDiff.txt";  // 文件的相对路径
    fp = fopen(fname, "a");  // 打开文件
    // 在这里写入读取/处理文件的代码
    fprintf(fp,"init_state_: %.10f  %.10f  %.10f\n", plan_start_state_XYTheta(0), plan_start_state_XYTheta(1), plan_start_state_XYTheta(2));
    fprintf(fp,"goal_state_: %.10f  %.10f  %.10f\n", goal_state_(0), goal_state_(1), goal_state_(2));
    fprintf(fp,"plan_start_state_VAJ: %.10f  %.10f  %.10f\n", plan_start_state_VAJ(0), plan_start_state_VAJ(1), plan_start_state_VAJ(2));
    fprintf(fp,"plan_start_state_OAJ: %.10f  %.10f  %.10f\n", plan_start_state_OAJ(0), plan_start_state_OAJ(1), plan_start_state_OAJ(2));
    fclose(fp);  // 关闭文件

        // 更新局部地图
        ros::Time current = ros::Time::now();
        world2oct_->updateLocalMap(current_state_XYTheta_);
        ROS_INFO("\033[41;37m updateLocalMap time:%f \033[0m", (ros::Time::now()-current).toSec());

        // 前端
        current = ros::Time::now();
        if(!findAstarRoad()) return;
        ROS_INFO("\033[41;37m all of kinoastar time:%f \033[0m", (ros::Time::now()-current).toSec());
        
        // 后端
        current = ros::Time::now();
        bool result = gcopter_->minco_plan(kinoastar_->flat_traj_);
        Traj_total_time_ = gcopter_->final_traj.getTotalDuration();

        //可视化
        visualizer_->mincoPathPub(gcopter_->final_traj, plan_start_state_XYTheta);

        // 存储轨迹开始时间
        if(plan_start_time_ < 0){
          Traj_start_time_ = ros::Time::now();
          plan_start_time_ = Traj_start_time_.toSec();
        }
        else{
          Traj_start_time_ = ros::Time(plan_start_time_);
        }

        // endPlanTime = std::chrono::steady_clock::now(); 
        // 走MPC
        if(if_mpc_){
          // auto elapsedMilliseconds = std::chrono::duration_cast<std::chrono::microseconds>(endPlanTime - startPlanTime).count();
          // int sleepTimeMs = int(max_replan_time_ * 1000000.0) - elapsedMilliseconds;
          // std::cout<<"sleepTimesMs: "<<sleepTimeMs<<std::endl;
          // if (sleepTimeMs > 0) {
          //     std::this_thread::sleep_for(std::chrono::microseconds(sleepTimeMs));
          // }
          MPCPathPub(plan_start_time_);
        }

        if(!result){
          exit(0);
        }

      }
      
      // 走真值
      if(!if_mpc_){
        mincoCarPub((ros::Time::now() - Traj_start_time_).toSec());
        if((ros::Time::now() - Traj_start_time_).toSec() >= Traj_total_time_){
          state_machine_ = StateMachine::IDLE;
          carstatemsgs::CarState carstate;
          carstate.Header.frame_id = "base";
          carstate.Header.stamp = ros::Time::now();
          carstate.omega = 0.0;
          carstate.v = 0.0;
          carstate.alpha = 0.0;
          carstate.a = 0.0;
          carstate.jyaw = 0.0;
          carstate.js = 0.0;
          cmd_pub_.publish(carstate);
          if(!if_random_goal_){
            have_goal_ = false;
          }
        }
      }

      if(if_mpc_){
        if((ros::Time::now() - Traj_start_time_).toSec() >= Traj_total_time_){
          state_machine_ = StateMachine::IDLE;
          if(!if_random_goal_){
            have_goal_ = false;
          }
        }
      }

    }


    bool findAstarRoad(){
      kinoastar_->reset();
      ros::Time current = ros::Time::now();
      kinoastar_->search(goal_state_,plan_start_state_XYTheta, plan_start_state_VAJ, plan_start_state_OAJ);
      // kinoastar_->search(goal_state_,current_state_XYTheta_, Eigen::Vector3d(0.542585,  -0.81197,  -2.63714), Eigen::Vector3d(0.389632, -0.301501,   -3.1911));
      ROS_INFO("\033[40;36m kinoastar search time:%lf  \033[0m", (ros::Time::now()-current).toSec());
      if(kinoastar_->has_path_) kinoastar_->getKinoNode();
std::FILE *fp;
char *fname = "SimpsonDiff.txt";  // 文件的相对路径
fp = fopen(fname, "a");  // 打开文件
// 在这里写入读取/处理文件的代码
fprintf(fp,"kinoastar search time: %.10f\n", (ros::Time::now()-current).toSec());
fclose(fp);  // 关闭文件
      if(kinoastar_->has_path_) visualizer_->kinoastarPathPub(kinoastar_);
      return kinoastar_->has_path_;
    }

    void mincoCarPub(const double& time_offset){
      double total_time = gcopter_->final_traj.getTotalDuration();     
      double current =  total_time>time_offset ? time_offset : total_time;
      if(!ros::ok()){
        carstatemsgs::CarState carstate;
        carstate.Header.frame_id = "base";
        carstate.Header.stamp = ros::Time::now();   
        cmd_pub_.publish(carstate);      
        return;
      }
      Eigen::Vector2d SY = gcopter_->final_traj.getPos(time_offset);
      Eigen::Vector2d VO = gcopter_->final_traj.getVel(time_offset);
      Eigen::Vector2d AA = gcopter_->final_traj.getAcc(time_offset);
      Eigen::Vector2d JJ = gcopter_->final_traj.getJer(time_offset);

      carstatemsgs::CarState carstate;
      carstate.Header.frame_id = "base";
      carstate.Header.stamp = ros::Time::now();
      carstate.yaw = SY.x();
      carstate.s = SY.y();
      carstate.omega = VO.x();
      carstate.v = VO.y();
      carstate.alpha = AA.x();
      carstate.a = AA.y();
      carstate.jyaw = JJ.x();
      carstate.js = JJ.y();
      cmd_pub_.publish(carstate);
    
    }

    void MPCPathPub(const double& traj_start_time){
      Eigen::MatrixXd initstate = gcopter_->get_current_iniState();
      Eigen::MatrixXd finState = gcopter_->get_current_finState();
      Eigen::MatrixXd finalInnerpoints = gcopter_->get_current_Innerpoints();
      Eigen::VectorXd finalpieceTime = gcopter_->get_current_finalpieceTime();
      Eigen::Vector3d iniStateXYTheta = gcopter_->get_current_iniStateXYTheta();

      mpc_controller::Polynome polynome;
      polynome.header.frame_id = "base";
      polynome.header.stamp = ros::Time::now();
      polynome.init_p.x = initstate.col(0).x();
      polynome.init_p.y = initstate.col(0).y();
      polynome.init_v.x = initstate.col(1).x();
      polynome.init_v.y = initstate.col(1).y();
      polynome.init_a.x = initstate.col(2).x();
      polynome.init_a.y = initstate.col(2).y();
      polynome.tail_p.x = finState.col(0).x();
      polynome.tail_p.y = finState.col(0).y();
      polynome.tail_v.x = finState.col(1).x();
      polynome.tail_v.y = finState.col(1).y();
      polynome.tail_a.x = finState.col(2).x();
      polynome.tail_a.y = finState.col(2).y();

      if(plan_start_time_ < 0) polynome.traj_start_time = ros::Time::now();
      else polynome.traj_start_time = ros::Time(plan_start_time_);

      for(u_int i=0; i<finalInnerpoints.cols(); i++){
        geometry_msgs::Vector3 point;
        point.x = finalInnerpoints.col(i).x();
        point.y = finalInnerpoints.col(i).y();
        point.z = 0.0;
        polynome.innerpoints.push_back(point);
      }
      for(u_int i=0; i<finalpieceTime.size(); i++){
        polynome.t_pts.push_back(finalpieceTime[i]);
      }
      polynome.start_position.x = iniStateXYTheta.x();
      polynome.start_position.y = iniStateXYTheta.y();
      polynome.start_position.z = iniStateXYTheta.z();
      
      mpc_polynome_pub_.publish(polynome);
    }

};


#endif