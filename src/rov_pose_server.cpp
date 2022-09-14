#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>

#include <mavros_msgs/ManualControl.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <rov_actionlib/PoseAction.h>

class PoseAction
{
public:
  PoseAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&PoseAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PoseAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/mavros/global_position/global", 1, &PoseAction::controlCB, this);
    sub_imu_ = nh_.subscribe("/mavros/imu/data", 1, &PoseAction::imuCB, this);
    pub_ = nh_.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 1);
    as_.start();
  }

  ~PoseAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    rov_actionlib::PoseGoal goal = *as_.acceptNewGoal();

    // pose_ = goal.pose;
    turning_ = true;
    prv_dis_error = 1000000;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
  {
    ROS_INFO("imu_data.orientation.x: %f", msg->orientation.x);
    ROS_INFO("imu_data.orientation.y: %f", msg->orientation.y);
    ROS_INFO("imu_data.orientation.z: %f", msg->orientation.z);
    ROS_INFO("imu_data.orientation.w: %f", msg->orientation.w);
  }

  void controlCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    command_.x = 0;
    command_.r = 0;
    altitude_ = msg->altitude;
    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
    
    ROS_INFO("altitude: %f", altitude_);
    ROS_INFO("latitude: %f", latitude_);
    ROS_INFO("longitude: %f", longitude_);
        
    // dis_error_ = fabs(sqrt((pose_.x-msg->x)*(pose_.x-msg->x) + (pose_.y-msg->y)*(pose_.y-msg->y)));    
    dis_error_ = 0;
    // heading_ = atan2(pose_.y-msg->y, pose_.x-msg->x); 
    // theta_error_ = angles::normalize_angle(heading_ - msg->theta);    
    theta_error_ = 0;
    // ROS_INFO("Dist error: %f", dis_error_);
    // ROS_INFO("Heading: %f", heading_);
    // ROS_INFO("Theta error: %f", theta_error_);
    if (turning_)
    {
      double a_scale = 6.0;
      double error_tol = 0.00001;
      if (fabs(theta_error_) > error_tol)
      {
        command_.x = 0;
        command_.r = 0; // a_scale*theta_error_;
      }
      else 
      {
        turning_ = false;
      }
    }
    else
    {
      double l_scale = 6.0;
      double error_tol = 0.00001;
      if ((dis_error_ > error_tol) && (dis_error_ < prv_dis_error))
      {
        command_.x = 0; // l_scale*dis_error_;
        command_.r = 0;
        prv_dis_error = dis_error_;
      }
      else
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        result_.dis_error = dis_error_;
        as_.setSucceeded(result_);
      }
    }
    pub_.publish(command_);

  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rov_actionlib::PoseAction> as_;
  std::string action_name_;
  double dis_error_, theta_error_, heading_, prv_dis_error;
  bool turning_;
  double altitude_, latitude_, longitude_;
  mavros_msgs::ManualControl command_;
  rov_actionlib::PoseResult result_;
  ros::Subscriber sub_, sub_imu_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rov_pose");

  PoseAction shape(ros::this_node::getName());
  ros::spin();

  return 0;
}
