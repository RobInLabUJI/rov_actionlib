#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <mavros_msgs/ManualControl.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
//#include <rov_actionlib/PoseAction.h>
#include <rov_actionlib/GPSLocationAction.h>

void LatLng2GlobalXY(double lat, double lng, double& x, double& y)
{
  double EarthRadius = 6371000;
  double lat0 = 40.0 * M_PI / 180.0;
  
  // Equirectangular projection
  // https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
  x = EarthRadius * lng * cos(lat0);
  y = EarthRadius * lat;
}

class GPSLocationAction
{
public:
  GPSLocationAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&GPSLocationAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&GPSLocationAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/mavros/global_position/global", 1, &GPSLocationAction::controlCB, this);
    sub_imu_ = nh_.subscribe("/mavros/imu/data", 1, &GPSLocationAction::imuCB, this);
    pub_ = nh_.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 1);
    
    as_.start();
  }

  ~GPSLocationAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    rov_actionlib::GPSLocationGoal goal = *as_.acceptNewGoal();

    // pose_ = goal.pose;
    double lat = goal.location.latitude;
    double lng = goal.location.longitude;
    LatLng2GlobalXY(lat, lng, goalX_, goalY_);

    turning_ = true;
    prv_dis_error_ = 1000000;
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
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    orientation_ = yaw;
    ROS_INFO("Yaw angle: %f", orientation_);    
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
    
    double x, y;
    LatLng2GlobalXY(latitude_, longitude_, x, y);
    
    dis_error_ = fabs(sqrt((goalX_-x)*(goalX_-x) + (goalY_-y)*(goalY_-y)));    
    heading_ = atan2(goalY_-y, goalX_-x); 
    theta_error_ = angles::normalize_angle(heading_ - orientation_);    

    ROS_INFO("Dist error: %f", dis_error_);
    ROS_INFO("Heading: %f", heading_);
    ROS_INFO("Theta error: %f", theta_error_);
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
      if ((dis_error_ > error_tol) && (dis_error_ < prv_dis_error_))
      {
        command_.x = 0; // l_scale*dis_error_;
        command_.r = 0;
        prv_dis_error_ = dis_error_;
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
  actionlib::SimpleActionServer<rov_actionlib::GPSLocationAction> as_;
  std::string action_name_;
  double dis_error_, theta_error_, heading_, prv_dis_error_;
  bool turning_;
  double altitude_, latitude_, longitude_;
  double orientation_;
  double goalX_, goalY_;
  mavros_msgs::ManualControl command_;
  rov_actionlib::GPSLocationResult result_;
  ros::Subscriber sub_, sub_imu_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rov_pose");

  GPSLocationAction gpsLoc(ros::this_node::getName());
  ros::spin();

  return 0;
}
