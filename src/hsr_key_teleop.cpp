#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class HSRKeyTeleop
{
private:
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEYCODE_A = 0x61;
  static const char KEYCODE_B = 0x62;
  static const char KEYCODE_C = 0x63;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_J = 0x6a;
  static const char KEYCODE_M = 0x6d;

  static const char KEYCODE_Q = 0x71;
  static const char KEYCODE_S = 0x73;
  static const char KEYCODE_U = 0x75;

  const std::string ARM_LIFT_JOINT_NAME = "arm_lift_joint";

  const std::string MSG_ARE_YOU_READY  = "Are_you_ready?";

  const std::string MSG_I_AM_READY      = "I_am_ready";
  const std::string MSG_IS_THIS_CORRECT = "Is_this_correct?";
  const std::string MSG_TASK_FINISHED   = "Task_finished";

public:
  HSRKeyTeleop();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void moveBase(ros::Publisher &publisher, double linear_x, double angular_z);
  void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const int duration_sec);
  void moveHand(ros::Publisher &publisher, bool grasp);

  void showHelp();
  int run(int argc, char **argv);

private:
  // Last position and previous position of arm_lift_joint
  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
};


HSRKeyTeleop::HSRKeyTeleop()
{
  arm_lift_joint_pos1_ = 0.0;
  arm_lift_joint_pos2_ = 0.0;
}


void HSRKeyTeleop::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int HSRKeyTeleop::canReceive( int fd )
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

void HSRKeyTeleop::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  for(int i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i]==ARM_LIFT_JOINT_NAME)
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
//      ROS_INFO("lift:%lf",armLiftJointPos_);
      return;
    }
  }
}

void HSRKeyTeleop::moveBase(ros::Publisher &publisher, double linear_x, double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = 0.0;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  publisher.publish(twist);
}

void HSRKeyTeleop::moveArm(ros::Publisher &publisher, const std::string &name, const double position, const int duration_sec)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  ros::Duration duration;
  duration.sec = duration_sec;

  trajectory_msgs::JointTrajectory joint_trajectory;

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher.publish(joint_trajectory);
}

void HSRKeyTeleop::moveHand(ros::Publisher &publisher, bool is_hand_open)
{
  std::vector<std::string> joint_names {"hand_l_proximal_joint", "hand_r_proximal_joint"};

  std::vector<double> positions;

  if(is_hand_open)
  {
    ROS_DEBUG("Grasp");
    positions.push_back(0.0);
    positions.push_back(0.0);
  }
  else
  {
    ROS_DEBUG("Open hand");
    positions.push_back(+0.611);
    positions.push_back(-0.611);
  }

  ros::Duration duration;
  duration.sec = 2;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = duration;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);

  publisher.publish(joint_trajectory);
}


void HSRKeyTeleop::showHelp()
{
  puts("Operate from keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR");
  puts("s          : Stop HSR");
  puts("---------------------------");
  puts("u : Move Arm Up");
  puts("j : Stop Arm");
  puts("m : Move Arm Down");
  puts("---------------------------");
  puts("a : Rotate Arm - Vertical");
  puts("b : Rotate Arm - Horizontal");
  puts("c : Rotate Arm - Low position");
  puts("---------------------------");
  puts("g : Grasp/Open hand");
  puts("---------------------------");
  puts(("1 : Send "+MSG_IS_THIS_CORRECT).c_str());
  puts(("2 : Send "+MSG_TASK_FINISHED).c_str());
  puts("---------------------------");
  puts("h : Show help");
}

int HSRKeyTeleop::run(int argc, char **argv)
{
  char c;

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

  showHelp();

  ros::init(argc, argv, "hsr_teleop_key");

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(40);

  std::string sub_msg_to_robot_topic_name;
  std::string pub_msg_to_moderator_topic_name;
  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_arm_trajectory_topic_name;
  std::string pub_gripper_trajectory_topic_name;

  node_handle.param<std::string>("sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/hsrb/joint_states");
  node_handle.param<std::string>("pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/opt_command_velocity");
  node_handle.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
  node_handle.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_trajectory_controller/command");

  ros::Subscriber sub_joint_state        = node_handle.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &HSRKeyTeleop::jointStateCallback, this);
  ros::Publisher  pub_base_twist         = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
  ros::Publisher  pub_arm_trajectory     = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
  ros::Publisher  pub_gripper_trajectory = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);


  bool is_hand_open = false;

  std::string arm_lift_joint_name   = "arm_lift_joint";
  std::string arm_flex_joint_name   = "arm_flex_joint";
  std::string wrist_flex_joint_name = "wrist_flex_joint";

  while (ros::ok())
  {
    if(canReceive(kfd))
    {
      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      switch(c)
      {
        case KEYCODE_UP:
        {
          ROS_DEBUG("Go forward");
          moveBase(pub_base_twist, +1.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go back");
          moveBase(pub_base_twist, -1.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Go right");
          moveBase(pub_base_twist, 0.0, -1.0);
          break;
        }
        case KEYCODE_LEFT:
        {
          ROS_DEBUG("Go left");
          moveBase(pub_base_twist, 0.0, +1.0);
          break;
        }
        case KEYCODE_S:
        {
          ROS_DEBUG("Stop");
          moveBase(pub_base_twist, 0.0, 0.0);
          break;
        }
        case KEYCODE_U:
        {
          ROS_DEBUG("Move Arm Up");
          moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_J:
        {
          ROS_DEBUG("Stop Arm");
          moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
          break;
        }
        case KEYCODE_M:
        {
          ROS_DEBUG("Move Arm Down");
          moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_A:
        {
          ROS_DEBUG("Rotate Arm - Vertical");
          moveArm(pub_arm_trajectory, arm_flex_joint_name, 0.0, 1);
          moveArm(pub_arm_trajectory, wrist_flex_joint_name, -1.57, 1);
          break;
        }
        case KEYCODE_B:
        {
          ROS_DEBUG("Rotate Arm - Horizontal");
          moveArm(pub_arm_trajectory, arm_flex_joint_name, -1.57, 1);
          moveArm(pub_arm_trajectory, wrist_flex_joint_name, 0.0, 1);
          break;
        }
        case KEYCODE_C:
        {
          ROS_DEBUG("Rotate Arm - Low position");
          moveArm(pub_arm_trajectory, arm_flex_joint_name, -2.2, 1);
          moveArm(pub_arm_trajectory, wrist_flex_joint_name, 0.35, 1);
          break;
        }
        case KEYCODE_G:
        {
          moveHand(pub_gripper_trajectory, is_hand_open);

          is_hand_open = !is_hand_open;
          break;
        }
        case KEYCODE_H:
        {
          showHelp();
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


int main(int argc, char** argv)
{
  HSRKeyTeleop hsr_key_teleop;
  return hsr_key_teleop.run(argc, argv);
}



