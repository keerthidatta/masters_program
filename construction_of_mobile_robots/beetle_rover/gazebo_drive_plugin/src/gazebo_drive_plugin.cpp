#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include <sdf/sdf.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
  class GazeboDrivePlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      this->model = _parent;
      this->sdf = _sdf;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GazeboDrivePlugin::OnUpdate, this));
      
      this->old_secs =ros::Time::now().toSec();

     if (this->sdf->HasElement("namespace_model"))
          this->namespace_model = this->sdf->Get<std::string>("namespace_model");
     if (this->sdf->HasElement("wheel_kp"))
          this->wheel_kp = this->sdf->Get<double>("wheel_kp");
      if (this->sdf->HasElement("wheel_ki"))
          this->wheel_ki = this->sdf->Get<double>("wheel_ki");
      if (this->sdf->HasElement("wheel_kd"))
          this->wheel_kd = this->sdf->Get<double>("wheel_kd");
      if (this->sdf->HasElement("Cornerwheel_kp"))
          this->Cornerwheel_kp = this->sdf->Get<double>("Cornerwheel_kp");
      if (this->sdf->HasElement("Cornerwheel_ki"))
          this->Cornerwheel_ki = this->sdf->Get<double>("Cornerwheel_ki");
      if (this->sdf->HasElement("Cornerwheel_kd"))
          this->Cornerwheel_kd = this->sdf->Get<double>("Cornerwheel_kd");

      if (this->sdf->HasElement("activate_pid_control"))
          this->activate_pid_control = (this->sdf->Get<std::string>("activate_pid_control") == "yes");

      ROS_INFO("gazebo_drive_plugin - gazebo_drive_plugin Plugin namespace.");
	

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_drive_plugin",
            ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("gazebo_drive_plugin_node"));



      if(this->activate_pid_control)
      {
          // Activated PID Speed Control
	  const auto &jointController = this->model->GetJointController();
          jointController->Reset();
	  
          //Wheel Joints
          jointController->AddJoint(model->GetJoint("beetle__front_right_wheel_joint"));
          jointController->AddJoint(model->GetJoint("beetle__front_left_wheel_joint"));
          jointController->AddJoint(model->GetJoint("beetle__rear_left_wheel_joint"));
          jointController->AddJoint(model->GetJoint("beetle__rear_right_wheel_joint"));
          jointController->AddJoint(model->GetJoint("beetle__center_left_wheel_joint"));
          jointController->AddJoint(model->GetJoint("beetle__center_right_wheel_joint"));

	  //Corner Joints
          jointController->AddJoint(model->GetJoint("beetle__front_right_corner_joint"));
          jointController->AddJoint(model->GetJoint("beetle__front_left_corner_joint"));
          jointController->AddJoint(model->GetJoint("beetle__rear_right_corner_joint"));
          jointController->AddJoint(model->GetJoint("beetle__rear_left_corner_joint"));

          this->right_wheel_name = model->GetJoint("beetle__front_right_wheel_joint")->GetScopedName();
          this->left_wheel_name = model->GetJoint("beetle__front_left_wheel_joint")->GetScopedName();
          this->rear_left_wheel_name = model->GetJoint("beetle__rear_left_wheel_joint")->GetScopedName();
          this->rear_right_wheel_name = model->GetJoint("beetle__rear_right_wheel_joint")->GetScopedName();
          this->center_left_wheel_name = model->GetJoint("beetle__center_left_wheel_joint")->GetScopedName();
          this->center_right_wheel_name = model->GetJoint("beetle__center_right_wheel_joint")->GetScopedName();

          this->front_right_corner_name = model->GetJoint("beetle__front_right_corner_joint")->GetScopedName();
          this->front_left_corner_name = model->GetJoint("beetle__front_left_corner_joint")->GetScopedName();
          this->rear_right_corner_name = model->GetJoint("beetle__rear_right_corner_joint")->GetScopedName();
          this->rear_left_corner_name = model->GetJoint("beetle__rear_left_corner_joint")->GetScopedName();
	
          jointController->SetVelocityPID(this->right_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->left_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));          
          jointController->SetVelocityPID(this->rear_left_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->rear_right_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->center_left_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->center_right_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          
	  jointController->SetPositionPID(this->front_right_corner_name, common::PID(this->Cornerwheel_kp, this->Cornerwheel_ki, this->Cornerwheel_kd));
	  jointController->SetPositionPID(this->front_left_corner_name, common::PID(this->Cornerwheel_kp, this->Cornerwheel_ki, this->Cornerwheel_kd));
	  jointController->SetPositionPID(this->rear_right_corner_name, common::PID(this->Cornerwheel_kp, this->Cornerwheel_ki, this->Cornerwheel_kd));
	  jointController->SetPositionPID(this->rear_left_corner_name, common::PID(this->Cornerwheel_kp, this->Cornerwheel_ki, this->Cornerwheel_kd));

          jointController->SetVelocityTarget(this->right_wheel_name, 0.0);
          jointController->SetVelocityTarget(this->left_wheel_name, 0.0);
          jointController->SetVelocityTarget(this->rear_left_wheel_name, 0.0);
          jointController->SetVelocityTarget(this->rear_right_wheel_name, 0.0);
          jointController->SetVelocityTarget(this->center_left_wheel_name, 0.0);
          jointController->SetVelocityTarget(this->center_right_wheel_name, 0.0);
          
	  /*jointController->SetPositionTarget(this->front_right_corner_name, 0.0);
          jointController->SetPositionTarget(this->front_left_corner_name, 0.0);
          jointController->SetPositionTarget(this->rear_right_corner_name, 0.0);
          jointController->SetPositionTarget(this->rear_left_corner_name, 0.0);*/
      }

      ros::SubscribeOptions so1 =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/beetle_left_wheel_speed",
            1,
            boost::bind(&GazeboDrivePlugin::OnRosMsg_left_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so1);
        
      ros::SubscribeOptions so2 =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/beetle_right_wheel_speed",
            1,
            boost::bind(&GazeboDrivePlugin::OnRosMsg_right_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub2 = this->rosNode->subscribe(so2);

ros::SubscribeOptions so3 =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/beetle_rear_left_wheel_speed",
            1,
            boost::bind(&GazeboDrivePlugin::OnRosMsg_rear_left_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub3 = this->rosNode->subscribe(so3);

ros::SubscribeOptions so4 =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/beetle_rear_right_wheel_speed",
            1,
            boost::bind(&GazeboDrivePlugin::OnRosMsg_rear_right_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub4 = this->rosNode->subscribe(so4);

ros::SubscribeOptions so5 =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/beetle_center_left_wheel_speed",
            1,
            boost::bind(&GazeboDrivePlugin::OnRosMsg_center_left_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub5 = this->rosNode->subscribe(so5);

ros::SubscribeOptions so6 =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/beetle_center_right_wheel_speed",
            1,
            boost::bind(&GazeboDrivePlugin::OnRosMsg_center_right_wheel_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub6 = this->rosNode->subscribe(so6);


     this->rosQueueThread =
        std::thread(std::bind(&GazeboDrivePlugin::QueueThread, this));
	
      ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());
      
    }

    public: void OnUpdate()
    {
      double new_secs =ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;
      
      double max_delta = 0.0;
      const auto &jointController = this->model->GetJointController();
	
      if (this->freq_update != 0.0)
      {
        max_delta = 1.0 / this->freq_update;
      }
      
      if (delta > max_delta && delta != 0.0)
      {

        this->old_secs = new_secs;
        if(this->activate_pid_control)
        {
          ROS_DEBUG("Update Wheel Speed PID...");
	  
	  jointController->SetVelocityTarget(this->right_wheel_name, this->right_wheel_speed_magn);
          jointController->SetVelocityTarget(this->left_wheel_name, this->left_wheel_speed_magn);
	  jointController->SetVelocityTarget(this->rear_left_wheel_name, this->rear_left_wheel_speed_magn);
          jointController->SetVelocityTarget(this->rear_right_wheel_name, this->rear_right_wheel_speed_magn);
	  jointController->SetVelocityTarget(this->center_left_wheel_name, this->center_left_wheel_speed_magn);
          jointController->SetVelocityTarget(this->center_right_wheel_name, this->center_right_wheel_speed_magn);
		
          /*jointController->SetPositionTarget(this->front_left_corner_name, this->left_wheel_pos_magn);
          jointController->SetPositionTarget(this->front_right_corner_name, this->right_wheel_pos_magn);
          jointController->SetPositionTarget(this->rear_left_corner_name, this->rear_left_wheel_pos_magn);
          jointController->SetPositionTarget(this->rear_right_corner_name, this->rear_right_wheel_pos_magn);*/
	  
	  jointController->SetJointPosition(this->front_left_corner_name, left_wheel_pos_magn);
          jointController->SetJointPosition(this->front_right_corner_name, -right_wheel_pos_magn);
          jointController->SetJointPosition(this->rear_left_corner_name, -rear_left_wheel_pos_magn);
          jointController->SetJointPosition(this->rear_right_corner_name, rear_right_wheel_pos_magn);
	  
        } 	
      }

      jointController->Update();
    }

    /*double constVelocity = 12.0;

    public: double ComputeSteeringTime(double PrevPosition, double TargetPosition)
    {
	return (TargetPosition - PrevPosition)/constVelocity;
    }*/

    public: double ComputeTargetPosition(const double &TurnAngle)
    {
	return TurnAngle;
    }
	
    public: double MotorReadingsConverter(const double &value)
    {
	return (value );
    }
    public: double metersToradians(double meters)
    {
      return meters / 0.0375 ;
    }
  
    //Set each wheel speed
    public: void SetLeftWheelSpeed(const double &vel)
    {

      this->left_wheel_speed_magn = metersToradians(MotorReadingsConverter(vel));
      ROS_WARN("left_wheel_speed_magn %lf", this->left_wheel_speed_magn);
    }
    
    public: void SetRightWheelSpeed(const double &vel)
    {
      this->right_wheel_speed_magn = metersToradians(MotorReadingsConverter(vel));
      ROS_WARN("right_wheel_speed_magn %lf", this->right_wheel_speed_magn);
    }

    public: void SetRearLeftWheelSpeed(const double &vel)
    {
      this->rear_left_wheel_speed_magn = metersToradians(MotorReadingsConverter(vel));
      ROS_WARN("rear_left_wheel_speed_magn %lf", this->rear_left_wheel_speed_magn);
    }

    public: void SetRearRightWheelSpeed(const double &vel)
    {
      this->rear_right_wheel_speed_magn = metersToradians(MotorReadingsConverter(vel));
      ROS_WARN("rear_right_wheel_speed_magn %lf", this->rear_right_wheel_speed_magn);
    }

    public: void SetCenterLeftWheelSpeed(const double &vel)
    {
      this->center_left_wheel_speed_magn = metersToradians(MotorReadingsConverter(vel));
      ROS_WARN("center_left_wheel_speed_magn %lf", this->center_left_wheel_speed_magn);
    }

    public: void SetCenterRightWheelSpeed(const double &vel)
    {
      this->center_right_wheel_speed_magn = metersToradians(MotorReadingsConverter(vel));
      ROS_WARN("center_right_wheel_speed_magn %lf", this->center_right_wheel_speed_magn);
    }

    //Set corner angles for corner wheels
    public: void SetLeftWheelCornerAngle(const double &angle)
    {	
	this->left_wheel_pos_magn = angle;
        ROS_WARN("left_wheel_pos_magn %lf", this->left_wheel_pos_magn);
    }

    public: void SetRightWheelCornerAngle(const double &angle)
    {
    	this->right_wheel_pos_magn = angle;
        ROS_WARN("right_wheel_pos_magn %lf", this->right_wheel_pos_magn);
    }

    public: void SetRearLeftWheelCornerAngle(const double &angle)
    {
    	this->rear_left_wheel_pos_magn = angle;
        ROS_WARN("rear_left_wheel_pos_magn %lf", this->rear_left_wheel_pos_magn);
    }

    public: void SetRearRightWheelCornerAngle(const double &angle)
    {
      	this->rear_right_wheel_pos_magn = angle;
        ROS_WARN("rear_right_wheel_pos_magn %lf", this->rear_right_wheel_pos_magn);
    }

    //Callback Twist messages for each wheel
    public: void OnRosMsg_left_wheel_speed(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->SetLeftWheelSpeed(_msg->linear.x);
      this->SetLeftWheelCornerAngle(_msg->angular.z);
    }
    
    public: void OnRosMsg_right_wheel_speed(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->SetRightWheelSpeed(_msg->linear.x);
      this->SetRightWheelCornerAngle(_msg->angular.z);
    }
    
    public: void OnRosMsg_rear_left_wheel_speed(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->SetRearLeftWheelSpeed(_msg->linear.x);
      this->SetRearLeftWheelCornerAngle(_msg->angular.z);
    }

    public: void OnRosMsg_rear_right_wheel_speed(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->SetRearRightWheelSpeed(_msg->linear.x);
      this->SetRearRightWheelCornerAngle(_msg->angular.z);
    }

    public: void OnRosMsg_center_left_wheel_speed(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->SetCenterLeftWheelSpeed(_msg->linear.x);
    }

    public: void OnRosMsg_center_right_wheel_speed(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->SetCenterRightWheelSpeed(_msg->linear.x);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

   
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;
    
    // Frequency 
    double freq_update = 10.0;

    double left_wheel_speed_magn = 0.0;
    double right_wheel_speed_magn = 0.0;
    double rear_left_wheel_speed_magn = 0.0;
    double rear_right_wheel_speed_magn = 0.0;
    double center_left_wheel_speed_magn = 0.0;
    double center_right_wheel_speed_magn = 0.0;
    

    double left_wheel_pos_magn = 0.0;
    double right_wheel_pos_magn = 0.0;
    double rear_left_wheel_pos_magn = 0.0;
    double rear_right_wheel_pos_magn = 0.0;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
    
    private: ros::Subscriber rosSub2;
    private: ros::Subscriber rosSub3;
    private: ros::Subscriber rosSub4;
    private: ros::Subscriber rosSub5;
    private: ros::Subscriber rosSub6;

    private: sdf::ElementPtr sdf;
    
    std::string right_wheel_name;
    std::string left_wheel_name;
    std::string rear_left_wheel_name;
    std::string rear_right_wheel_name;
    std::string center_left_wheel_name;
    std::string center_right_wheel_name;
    std::string front_right_corner_name;
    std::string front_left_corner_name;
    std::string rear_right_corner_name;
    std::string rear_left_corner_name;
    
    std::string namespace_model = "";
    bool activate_pid_control;

    double wheel_kp = 0.1;
    double wheel_ki = 0.0;
    double wheel_kd = 0.0;
    double Cornerwheel_kp = 0.1;
    double Cornerwheel_ki = 0.0;
    double Cornerwheel_kd = 0.0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboDrivePlugin)
}
