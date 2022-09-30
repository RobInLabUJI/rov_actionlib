#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <mavros_msgs/ManualControl.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <rov_actionlib/GPSLocationAction.h>

void LatLng2GlobalXY(double lat, double lng, double& x, double& y)
{
  // Input: lat, lng in degrees
  // Output: x, y in meters (longitude, latitude)
   
  double EarthRadius = 6371000;
  double lat0 = 40.0 * M_PI / 180.0;
  
  // Equirectangular projection
  // https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
  // https://stackoverflow.com/questions/1253499/simple-calculations-for-working-with-lat-lon-and-km-distance
  x = EarthRadius * lng * M_PI / 180.0 * cos(lat0);
  y = EarthRadius * lat * M_PI / 180.0;
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

    goalLat_ = goal.location.latitude;
    goalLng_ = goal.location.longitude;

    turning_ = true;
    prv_dis_error_ = 1000000;
    ROS_INFO("NEW GOAL");
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
  {
    //ROS_INFO("imu_data.orientation.x: %f", msg->orientation.x);
    //ROS_INFO("imu_data.orientation.y: %f", msg->orientation.y);
    //ROS_INFO("imu_data.orientation.z: %f", msg->orientation.z);
    //ROS_INFO("imu_data.orientation.w: %f", msg->orientation.w);
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    orientation_ = yaw;
    // ROS_INFO("Yaw angle: %f", orientation_);    
  }

  void controlCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    double G_ROV_DEFAULT_SPEED = 200.0;
    // make sure that the action hasn't been canceled
    //if (!as_.isActive())
    //  return;
    
    command_.x = 0;
    command_.r = 0;

    altitude_ = msg->altitude;
    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
    
    // ROS_INFO("altitude: %f", altitude_);
    ROS_INFO("latitude: %f", latitude_);
    ROS_INFO("longitude: %f", longitude_);
    ROS_INFO("Yaw angle: %.1f", orientation_*180.0/M_PI);    
    
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;

    double x, y;
    LatLng2GlobalXY(goalLat_-latitude_, goalLng_-longitude_, x, y);
    
    dis_error_ = fabs(sqrt(x*x + y*y));    
    heading_ = 0; // atan2(y, x); 
    theta_error_ = angles::normalize_angle(heading_ - orientation_);    

    ROS_INFO("Dist error: %f", dis_error_);
    ROS_INFO("Heading: %f", heading_);
    ROS_INFO("Theta error: %f", theta_error_);

    if (turning_)
    {
      double a_scale = 6.0;
      double error_tol = 0.1;
      if (fabs(theta_error_) > error_tol)
      {
        if (theta_error_ >= 0)
        {
          command_.r = -G_ROV_DEFAULT_SPEED;
        } else {
          command_.r = G_ROV_DEFAULT_SPEED;
        }
      }
      else 
      {
        turning_ = false;
        dis_error_threshold_ = dis_error_ / 2;
      }
    }
    else
    {
      double l_scale = 6.0;
      double error_tol = 10;
      if ((dis_error_ > error_tol) && 
          (dis_error_ < prv_dis_error_) &&
          (dis_error_ > dis_error_threshold_))
      {
        // command_.x = 0; // l_scale*dis_error_;
        command_.x = G_ROV_DEFAULT_SPEED; 
        prv_dis_error_ = dis_error_;
      }
      else
      {
        if (dis_error_ <= error_tol)
        {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          result_.dis_error = dis_error_;
          as_.setSucceeded(result_);
        } else {
          turning_ = true;
        }
      }
    }
    ROS_INFO("command_.x: %f", command_.x);
    ROS_INFO("command_.r: %f", command_.r);
    pub_.publish(command_);
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rov_actionlib::GPSLocationAction> as_;
  std::string action_name_;
  double dis_error_, theta_error_, heading_, prv_dis_error_, dis_error_threshold_;
  bool turning_;
  double altitude_, latitude_, longitude_;
  double orientation_;
  double goalLng_, goalLat_;
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
