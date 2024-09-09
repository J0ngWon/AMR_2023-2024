#include "mw_ahrs_ros/mw_ahrs_driver.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

const string eol("\n");
const size_t max_line_length(128);

MwAhrsDriver::MwAhrsDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh), nh_priv_(nh_priv), port_("/dev/AMR_AHRS"), baud_(115200), rate_(50.0), frame_id_("imu_link"), version_("v1")
{
  nh_priv_.getParam("port", port_);
  nh_priv_.getParam("baud", baud_);
  nh_priv_.getParam("frame_id", frame_id_);
  nh_priv_.getParam("version", version_);
}

MwAhrsDriver::~MwAhrsDriver()
{
  serial_.close();
}

bool MwAhrsDriver::init()
{
  srv_reset_ = nh_.advertiseService("reset", &MwAhrsDriver::reset, this);

  rp_imu_.init(nh_, "data", 10);
  rp_imu_.msg_.header.frame_id = frame_id_;
  rp_imu_.msg_.orientation_covariance = { 0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025 };
  rp_imu_.msg_.angular_velocity_covariance = { 0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02 };
  rp_imu_.msg_.linear_acceleration_covariance = { 0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04 };

  rp_rpy_.init(nh_, "rpy", 10);
  rp_rpy_.msg_.header.frame_id = frame_id_;

  rp_mag_.init(nh_, "mag", 10);
  rp_mag_.msg_.header.frame_id = frame_id_;

  // Initial setting for serial communication
  serial_.setPort(port_);
  serial_.setBaudrate(baud_);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(to);

  try
  {
    serial_.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM_ONCE("serial::IOException: " << e.what());
  }

  // Check the serial port
  if (serial_.isOpen())
  {
    ROS_INFO("MW AHRS driver connected to %s at %i baud", port_.c_str(), baud_);
  }
  else
  {
    ROS_ERROR("MW AHRS driver failed to connect to %s", port_.c_str());
    return false;
  }

  start();

  return true;
}

void MwAhrsDriver::read() {
  if (serial_.available()) {
    string msg = serial_.readline(max_line_length, eol);
    // ROS_INFO_STREAM("Message : " << msg);

    int cnt = 0;
    string data[1000];
    char* buff = new char[1000];
    strcpy(buff, msg.c_str());
    char* tok = strtok(buff, " ");

    while (tok != nullptr) {
      data[cnt++] = string(tok);
      tok = strtok(nullptr, " ");
    }

    int num = 0;
    if (version_ == "v1")
      num = 12;
    else if (version_ == "v2")
      num = 14;

    if (cnt == num) {
      rp_rpy_.msg_.vector.x = stod(data[6]) * M_PI / 180.0;
      rp_rpy_.msg_.vector.y = stod(data[7]) * M_PI / 180.0;
      rp_rpy_.msg_.vector.z = stod(data[8]) * M_PI / 180.0;

      rp_imu_.msg_.linear_acceleration.x = stod(data[0]) * 9.80665;
      rp_imu_.msg_.linear_acceleration.y = stod(data[1]) * 9.80665;
      rp_imu_.msg_.linear_acceleration.z = stod(data[2]) * 9.80665;

      rp_imu_.msg_.angular_velocity.x = stod(data[3]) * M_PI / 180.0;
      rp_imu_.msg_.angular_velocity.y = stod(data[4]) * M_PI / 180.0;
      rp_imu_.msg_.angular_velocity.z = stod(data[5]) * M_PI / 180.0;

      geometry_msgs::Quaternion quat =
          tf::createQuaternionMsgFromRollPitchYaw(rp_rpy_.msg_.vector.x, rp_rpy_.msg_.vector.y, rp_rpy_.msg_.vector.z);

      rp_imu_.msg_.orientation = quat;

      rp_mag_.msg_.magnetic_field.x = stod(data[9]) * 0.000001;
      rp_mag_.msg_.magnetic_field.y = stod(data[10]) * 0.000001;
      rp_mag_.msg_.magnetic_field.z = stod(data[11]) * 0.000001;

      // TF 브로드캐스트 추가
      /*
      static tf::TransformBroadcaster br;
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "base_link";
      transformStamped.child_frame_id = "imu_link";
      transformStamped.transform.translation.x = 0.0;
      transformStamped.transform.translation.y = 0.0;
      transformStamped.transform.translation.z = 0.0;
      transformStamped.transform.rotation = quat;
      br.sendTransform(transformStamped);
      */
    }
  }
}

void MwAhrsDriver::start()
{
  string start = "ss=15\n";
  serial_.write(start);

  string frequency = "sp=10\n";
  serial_.write(frequency);
}

bool MwAhrsDriver::reset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  string msg = "rst\n";
  serial_.write(msg);
  this_thread::sleep_for(chrono::milliseconds(1000));
  start();

  return true;
}

void MwAhrsDriver::publishData()
{
  while (ros::ok())
  {
    if (rp_rpy_.trylock())
    {
      rp_rpy_.msg_.header.stamp = ros::Time::now();
      rp_rpy_.unlockAndPublish();
    }

    if (rp_imu_.trylock())
    {
      rp_imu_.msg_.header.stamp = ros::Time::now();
      rp_imu_.unlockAndPublish();
    }

    if (rp_mag_.trylock())
    {
      rp_mag_.msg_.header.stamp = ros::Time::now();
      rp_mag_.unlockAndPublish();
    }

    ros::Rate(rate_).sleep();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "mw_ahrs_driver_node");
  ros::NodeHandle nh("imu");
  ros::NodeHandle nh_priv("~");

  auto driver = make_shared<MwAhrsDriver>(nh, nh_priv);

  if (driver->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create thread for publishing the IMU data
    thread publish([&driver]() -> void { driver->publishData(); });

    while (ros::ok())
      driver->read();

    spinner.stop();
  }

  return 0;
}
