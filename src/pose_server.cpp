#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>

#include <geometry_msgs/Twist.h>
#include <rov_actionlib/PoseAction.h>

// This class computes the command_velocities of the turtle to draw regular polygons 
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
    sub_ = nh_.subscribe("/turtle1/pose", 1, &PoseAction::controlCB, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    as_.start();
  }

  ~PoseAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    rov_actionlib::PoseGoal goal = *as_.acceptNewGoal();
    //save the goal as private variables
    // edges_ = goal.edges;
    // radius_ = goal.radius;
    pose_ = goal.pose;
    turning_ = true;
    prv_dis_error = 1000000;
    
    // reset helper variables
    // interior_angle_ = ((edges_-2)*M_PI)/edges_;
    // apothem_ = radius_*cos(M_PI/edges_);
    //compute the side length of the polygon
    // side_len_ = apothem_ * 2* tan( M_PI/edges_);
    // store the result values
    // result_.apothem = apothem_;
    // result_.interior_angle = interior_angle_;
    // edge_progress_ =0;
    // start_edge_ = true;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void controlCB(const turtlesim::Pose::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    command_.linear.x = 0;
    command_.angular.z = 0;
    dis_error_ = fabs(sqrt((pose_.x-msg->x)*(pose_.x-msg->x) + (pose_.y-msg->y)*(pose_.y-msg->y)));    
    heading_ = atan2(pose_.y-msg->y, pose_.x-msg->x); 
    theta_error_ = angles::normalize_angle(heading_ - msg->theta);    
    ROS_INFO("Dist error: %f", dis_error_);
    ROS_INFO("Heading: %f", heading_);
    ROS_INFO("Theta error: %f", theta_error_);
    if (turning_)
    {
      double a_scale = 6.0;
      double error_tol = 0.00001;
      if (fabs(theta_error_) > error_tol)
      {
        command_.linear.x = 0;
        command_.angular.z = a_scale*theta_error_;
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
        command_.linear.x = l_scale*dis_error_;
        command_.angular.z = 0;
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
  // double radius_, apothem_, interior_angle_, side_len_;
  double start_x_, start_y_, start_theta_;
  double dis_error_, theta_error_, heading_, prv_dis_error;
  // int edges_ , edge_progress_;
  // bool start_edge_;
  bool turning_;
  turtlesim::Pose pose_;
  geometry_msgs::Twist command_;
  rov_actionlib::PoseResult result_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_pose");

  PoseAction shape(ros::this_node::getName());
  ros::spin();

  return 0;
}
