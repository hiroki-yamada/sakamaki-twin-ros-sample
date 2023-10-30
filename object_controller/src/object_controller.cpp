#include <stdio.h>
#include <cmath>
#include <string>
#include <list>
#include <map>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <boost/algorithm/string.hpp>

class SIGVerseObjectController
{
private:
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;
  static const char KEYCODE_3 = 0x33;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_I = 0x69;
  static const char KEYCODE_R = 0x72;
  
public:
  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);
  
  void receiveMessageCallback(const std_msgs::String::ConstPtr& message);
  void sendGraspedMessage(const std::string &name);
  void sendReleasedMessage();
  void createTimer(const std::string &name, const float speed);
  void sendPositon(const std::string &name, const float posx, const float posy);
  void sendPositonCallback(const std::string &name, const double start_time, const float speed);

  void showHelp();
  int run(int argc, char **argv);

private:
  ros::NodeHandle node_handle_;

  ros::Subscriber sub_msg_;
  ros::Publisher  pub_msg_;
  ros::Publisher  pub_transform_;
  
  std::map<std::string, ros::Timer> pub_timer_map_;
};

void SIGVerseObjectController::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseObjectController::canReceive( int fd )
{
  fd_set fdset;
  int ret;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}

int SIGVerseObjectController::run(int argc, char **argv)
{
  char c;
  int  ret;
  char buf[1024];

  /////////////////////////////////////////////
  // get the console in raw mode
  int kfd = 0;
  struct termios cooked;

  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(50);

  sub_msg_       = node_handle_.subscribe<std_msgs::String>("/goods/message/from_sigverse", 100, &SIGVerseObjectController::receiveMessageCallback, this);
  pub_msg_       = node_handle_.advertise<std_msgs::String>("/goods/message/from_ros", 10);
  pub_transform_ = node_handle_.advertise<geometry_msgs::TransformStamped>("/goods/transform", 10);
  
  showHelp();
  
  createTimer("bear_doll",   0.5);
  createTimer("dog_doll",    0.6);
  createTimer("rabbit_doll", 0.7);

  while (ros::ok())
  {
    if(canReceive(kfd))
    {
      // get the next event from the keyboard
      if((ret = read(kfd, &buf, sizeof(buf))) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      c = buf[ret-1];
          
      switch(c)
      {
        case KEYCODE_1:
        {
          sendPositon("table", 0.0, 0.0);
          break;
        }
        case KEYCODE_2:
        {
          sendPositon("table", 0.5, 0.0);
          break;
        }
        case KEYCODE_3:
        {
          sendPositon("table", 0.0, 0.5);
          break;
        }
        case KEYCODE_G:
        {
          sendGraspedMessage("bear_doll");
          break;
        }
        case KEYCODE_H:
        {
          sendGraspedMessage("dog_doll");
          break;
        }
        case KEYCODE_I:
        {
          sendGraspedMessage("rabbit_doll");
          break;
        }
        case KEYCODE_R:
        {
          sendReleasedMessage();
          break;
        }
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return EXIT_SUCCESS;
}

void SIGVerseObjectController::showHelp()
{
  puts("---------------------------");
  puts("-- Object Controller --");
  puts("---------------------------");
  puts("1 : Move Table to Position1");
  puts("2 : Move Table to Position2");
  puts("3 : Move Table to Position3");
  puts("g : Send Grasped bear_doll");
  puts("h : Send Grasped dog_doll");
  puts("i : Send Grasped rabbit_doll");
  puts("r : Send Released");
  puts("---------------------------");
}


void SIGVerseObjectController::receiveMessageCallback(const std_msgs::String::ConstPtr& message)
{
  ROS_INFO("Subscribe Message: %s", message->data.c_str());
}


void SIGVerseObjectController::sendGraspedMessage(const std::string &name)
{
  std_msgs::String msg;
  msg.data = "grasped,"+name;
  ROS_INFO("Sent Message: %s", msg.data.c_str());
  pub_msg_.publish(msg);
}

void SIGVerseObjectController::sendReleasedMessage()
{
  std_msgs::String msg;
  msg.data = "released";
  ROS_INFO("Sent Message: %s", msg.data.c_str());
  pub_msg_.publish(msg);
}

void SIGVerseObjectController::createTimer(const std::string &name, const float speed)
{
  pub_timer_map_[name]=node_handle_.createTimer(ros::Duration(0.05), boost::bind(&SIGVerseObjectController::sendPositonCallback, this, name, ros::Time::now().toSec(), speed));
}


void SIGVerseObjectController::sendPositon(const std::string &name, const float posx, const float posy)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = name;   // Please set the name

  // test position
  geometry_msgs::Vector3 pos;
  pos.x = posx;
  pos.y = posy;
  pos.z = 0.0;
  
  // test rotation
  tf::Quaternion tfqua = tf::createQuaternionFromRPY(0, 0, 0);
  
  geometry_msgs::Quaternion qua;
  quaternionTFToMsg(tfqua, qua);
  
  transformStamped.transform.translation = pos;
//  transformStamped.transform.rotation    = qua; // If not set, it will be calculated automatically by SIGVerse.
  
  pub_transform_.publish(transformStamped);
}


void SIGVerseObjectController::sendPositonCallback(const std::string &name, const double start_time, const float speed)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = name;   // Please set the name

  double pos_val = start_time + speed * ros::Time::now().toSec();
  
  // test position
  geometry_msgs::Vector3 pos;
  pos.x = cos(pos_val);
  pos.y = sin(pos_val);
  pos.z = 0.0;
  
  // test rotation
  tf::Quaternion tfqua = tf::createQuaternionFromRPY(0, 0, pos_val);
  
  geometry_msgs::Quaternion qua;
  quaternionTFToMsg(tfqua, qua);
  
  transformStamped.transform.translation = pos;
//  transformStamped.transform.rotation    = qua; // If not set, it will be calculated automatically by SIGVerse.
  
  pub_transform_.publish(transformStamped);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_controller");
  SIGVerseObjectController object_controller;
  return object_controller.run(argc, argv);
}
  


