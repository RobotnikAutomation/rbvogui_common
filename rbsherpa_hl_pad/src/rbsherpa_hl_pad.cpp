/*
 * rbsherpa_pad
 * Copyright (c) 2016, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the roboy controller, sending the messages
 * received from the joystick device
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <robotnik_msgs/ptz.h>
// Not yet catkinized 9/2013
// #include <sound_play/sound_play.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <robotnik_msgs/home.h>
#include <robotnik_msgs/ptz.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/set_mode.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <vector>

#define DEFAULT_NUM_OF_BUTTONS 16
#define DEFAULT_AXIS_LINEAR_X 1
#define DEFAULT_AXIS_LINEAR_Y 0
#define DEFAULT_AXIS_ANGULAR 2
#define DEFAULT_AXIS_LINEAR_Z 3
#define DEFAULT_SCALE_LINEAR 1.0
#define DEFAULT_SCALE_ANGULAR 2.0
#define DEFAULT_SCALE_LINEAR_Z 1.0

// Used only with ps4
#define AXIS_PTZ_TILT_UP 0
#define AXIS_PTZ_TILT_DOWN 1
#define AXIS_PTZ_PAN_LEFT 2
#define AXIS_PTZ_PAN_RIGHT 3

#define PAN_MIN -3.13
#define TILT_MIN -1.5708
#define PAN_MAX 3.13
#define TILT_MAX 1.5708

class RBSherpaPad {
public:
  RBSherpaPad();
  void Update();

private:
  void padCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  int linear_x_, linear_y_, linear_z_, angular_;
  double l_scale_, a_scale_, l_scale_z_;
  //! It will publish into command velocity (for the robot) and the ptz_state
  //! (for the pantilt)
  ros::Publisher vel_pub_;
  //! It will be suscribed to the joystick
  ros::Subscriber pad_sub_;
  //! Name of the topic where it will be publishing the velocity
  std::string cmd_topic_vel_;
  //! Name of the service where it will be modifying the digital outputs
  std::string cmd_service_io_;
  double current_vel;
  //! Pad type
  std::string pad_type_;
  //! Number of the DEADMAN button
  int dead_man_button_;
  //! Number of the button for increase or decrease the speed max of the
  //! joystick
  int speed_up_button_, speed_down_button_;
  int button_output_1_, button_output_2_;
  int output_1_, output_2_;
  bool bOutput1, bOutput2;
  //! button to change kinematic mode
  int button_kinematic_mode_;
  //! kinematic mode
  int kinematic_mode_;
  //! robot model
  std::string robot_model_;
  //! Service to modify the kinematic mode
  ros::ServiceClient setKinematicMode;
  //! Name of the service to change the mode
  std::string cmd_set_mode_;
  //! button to start the homing service
  int button_home_;
  //! Service to start homing
  ros::ServiceClient doHome;
  //! Name of the service to do the homing
  std::string cmd_home_;
  //! Service to modify the digital outputs
  ros::ServiceClient set_digital_outputs_client_;
  //! Number of buttons of the joystick
  int num_of_buttons_;
  //! Pointer to a vector for controlling the event when pushing the buttons
  bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
  //! Pointer to a vector for controlling the event when pushing directional
  //! arrows (UNDER AXES ON PX4!)
  bool bRegisteredDirectionalArrows[4];

  // DIAGNOSTICS
  //! Diagnostic to control the frequency of the published command velocity
  //! topic
  diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq;
  //! Diagnostic to control the reception frequency of the subscribed joy topic
  diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq;
  //! General status diagnostic updater
  diagnostic_updater::Updater updater_pad;
  //! Diagnostics min freq
  double min_freq_command, min_freq_joy; //
  //! Diagnostics max freq
  double max_freq_command, max_freq_joy; //
  //! Flag to enable/disable the communication with the publishers topics
  bool bEnable;
  //! Flag to track the first reading without the deadman's button pressed.
  bool last_command_;
  //! Client of the sound play service
  //  sound_play::SoundClient sc;
  //! Pan & tilt increment (degrees)
  //! Add a dead zone to the joystick that controls scissor and robot rotation
  //! (only useful for xWam)
  std::string joystick_dead_zone_;

  //! Joint state subscriber (this kind of controller works in position)
  ros::Subscriber joint_state_sub_;

  //! Robot Joint States
  sensor_msgs::JointState joint_state_;
  // bool bReadState_;
  bool bPtuOK_;
};

RBSherpaPad::RBSherpaPad()
    : linear_x_(1), linear_y_(0), angular_(2), linear_z_(3), nh_(), pnh_("~") {
  current_vel = 0.1;

  // JOYSTICK PAD TYPE
  pnh_.param<std::string>("pad_type", pad_type_, "ps3");
  //
  pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
  // MOTION CONF
  pnh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
  pnh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
  pnh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
  pnh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
  pnh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
  pnh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
  pnh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
  pnh_.param("cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_);
  pnh_.param("button_dead_man", dead_man_button_, dead_man_button_);
  pnh_.param("button_speed_up", speed_up_button_,
             speed_up_button_); // 4 Thrustmaster
  pnh_.param("button_speed_down", speed_down_button_,
             speed_down_button_); // 5 Thrustmaster
  pnh_.param<std::string>("joystick_dead_zone", joystick_dead_zone_, "true");

  // DIGITAL OUTPUTS CONF
  pnh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
  pnh_.param("button_output_1", button_output_1_, button_output_1_);
  pnh_.param("button_output_2", button_output_2_, button_output_2_);
  pnh_.param("output_1", output_1_, output_1_);
  pnh_.param("output_2", output_2_, output_2_);

  // KINEMATIC MODE
  kinematic_mode_ = 1;
  pnh_.param("kinematic_mode", kinematic_mode_, kinematic_mode_);
  pnh_.param("button_kinematic_mode", button_kinematic_mode_,
             button_kinematic_mode_);
  pnh_.param("cmd_service_set_mode", cmd_set_mode_, cmd_set_mode_);
  pnh_.param("cmd_service_home", cmd_home_, cmd_home_);

  robot_model_ = "";
  pnh_.param("robot_model", robot_model_, robot_model_);
  if (robot_model_ == "") {
    ROS_ERROR("No robot model defined");
  }
  ROS_INFO("RBSherpaPad num_of_buttons_ = %d", num_of_buttons_);
  for (int i = 0; i < num_of_buttons_; i++) {
    bRegisteredButtonEvent[i] = false;
    ROS_INFO("bREG %d", i);
  }

  for (int i = 0; i < 3; i++) {
    bRegisteredDirectionalArrows[i] = false;
  }

  /*ROS_INFO("Service I/O = [%s]", cmd_service_io_.c_str());
    ROS_INFO("Topic PTZ = [%s]", cmd_topic_ptz_.c_str());
    ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());
    ROS_INFO("Axis linear = %d", linear_);
    ROS_INFO("Axis angular = %d", angular_);
    ROS_INFO("Scale angular = %d", a_scale_);
    ROS_INFO("Deadman button = %d", dead_man_button_);
    ROS_INFO("OUTPUT1 button %d", button_output_1_);
    ROS_INFO("OUTPUT2 button %d", button_output_2_);
    ROS_INFO("OUTPUT1 button %d", button_output_1_);
    ROS_INFO("OUTPUT2 button %d", button_output_2_);*/

  // Publish through the node handle Twist type messages to the
  // guardian_controller/command topic
  vel_pub_ = pnh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
  // Listen through the node handle sensor_msgs::Joy messages from joystick
  // (these are the references that we will sent to rbsherpa_controller/command)
  pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
                                             &RBSherpaPad::padCallback, this);

  // Request service to activate / deactivate digital I/O
  set_digital_outputs_client_ =
      nh_.serviceClient<robotnik_msgs::set_digital_output>(cmd_service_io_);
  bOutput1 = bOutput2 = false;

  // Request service to set kinematic mode
  setKinematicMode = nh_.serviceClient<robotnik_msgs::set_mode>(cmd_set_mode_);

  // Request service to start homing
  doHome = nh_.serviceClient<robotnik_msgs::home>(cmd_home_);

  // Diagnostics
  updater_pad.setHardwareID("None");
  // Topics freq control
  min_freq_command = min_freq_joy = 5.0;
  max_freq_command = max_freq_joy = 50.0;
  sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(
      "/joy", updater_pad,
      diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy,
                                               0.1, 10));

  pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(
      cmd_topic_vel_.c_str(), updater_pad,
      diagnostic_updater::FrequencyStatusParam(&min_freq_command,
                                               &max_freq_command, 0.1, 10));

  bEnable = false; // Communication flag disabled by default
  last_command_ = true;
}

/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void RBSherpaPad::Update() { updater_pad.update(); }

void RBSherpaPad::padCallback(const sensor_msgs::Joy::ConstPtr &joy) {
  geometry_msgs::Twist vel;

  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;

  bEnable = (joy->buttons[dead_man_button_] == 1);

  // Actions dependant on dead-man button
  if (joy->buttons[dead_man_button_] == 1) {
    // ROS_ERROR("RBSherpaPad::padCallback: DEADMAN button %d",
    // dead_man_button_); Set the current velocity level
    if (joy->buttons[speed_down_button_] == 1) {
      if (!bRegisteredButtonEvent[speed_down_button_])
        if (current_vel > 0.1) {
          current_vel = current_vel - 0.1;
          bRegisteredButtonEvent[speed_down_button_] = true;
          ROS_INFO("Velocity: %f%%", current_vel * 100.0);
          char buf[50] = "\0";
          int percent = (int)(current_vel * 100.0);
          sprintf(buf, " %d percent", percent);
          // sc.say(buf);
        }
    } else {
      bRegisteredButtonEvent[speed_down_button_] = false;
    }

    if (joy->buttons[speed_up_button_] == 1) {
      if (!bRegisteredButtonEvent[speed_up_button_])
        if (current_vel < 0.9) {
          current_vel = current_vel + 0.1;
          bRegisteredButtonEvent[speed_up_button_] = true;
          ROS_INFO("Velocity: %f%%", current_vel * 100.0);
          char buf[50] = "\0";
          int percent = (int)(current_vel * 100.0);
          sprintf(buf, " %d percent", percent);
          // sc.say(buf);
        }
    } else {
      bRegisteredButtonEvent[speed_up_button_] = false;
    }

    vel.linear.x = current_vel * l_scale_ * joy->axes[linear_x_];

    if (kinematic_mode_ == 2) {
      vel.linear.y = current_vel * l_scale_ * joy->axes[linear_y_];
    }

    if (robot_model_ == "ackermann") {
      // in ackermann mode, angular velocity means steering_angle,
      // so we must not scale it to the "current_vel", otherwise
      // it will not be natural to control it
      vel.angular.z = (a_scale_ * joy->axes[angular_]);
    } else {

      if (kinematic_mode_ == 1) {
        double angular_vel = (a_scale_ * joy->axes[angular_]);
        vel.angular.z = (vel.linear.x / 0.612) * std::tan(angular_vel);
      } else {
        vel.angular.z = current_vel * (a_scale_ * joy->axes[angular_]);
      }
    }

    //    if (joystick_dead_zone_ == "true")
    //    {
    //      // limit scissor movement or robot turning (they are in the same
    //      joystick) if (joy->axes[angular_] == 1.0 || joy->axes[angular_] ==
    //      -1.0)  // if robot turning
    //      {
    //        // Same angular velocity for the three axis
    //        vel.angular.z = current_vel * (a_scale_ * joy->axes[angular_]);
    //      }
    //      else
    //      {
    //        // Same angular velocity for the three axis
    //        vel.angular.z = current_vel * (a_scale_ * joy->axes[angular_]);
    //      }
    //    }
    //    else  // no dead zone
    //    {
    //    }

    if (joy->buttons[button_kinematic_mode_] == 1 and robot_model_ == "omni") {
      if (!bRegisteredButtonEvent[button_kinematic_mode_]) {
        // Define mode (inc) - still coupled
        kinematic_mode_ += 1;
        if (kinematic_mode_ > 2)
          kinematic_mode_ = 1;
        ROS_INFO("RBSherpaPad::joyCallback: Kinematic Mode %d ",
                 kinematic_mode_);
        bRegisteredButtonEvent[button_kinematic_mode_] = true;
      }
    } else {
      bRegisteredButtonEvent[button_kinematic_mode_] = false;
    }
  } else {
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
  }

  sus_joy_freq->tick(); // Ticks the reception of joy events

  // Publish
  // Only publishes if it's enabled
  if (bEnable) {
    vel_pub_.publish(vel);
    pub_command_freq->tick();
    last_command_ = true;
  }

  if (!bEnable && last_command_) {
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel_pub_.publish(vel);

    pub_command_freq->tick();
    last_command_ = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rbsherpa_pad");
  RBSherpaPad rbsherpa_pad;

  // ros::Rate r(200.0);  // 200 for reading different joint_states topic
  // sources
  ros::Rate r(50.0);

  while (ros::ok()) {
    // UPDATING DIAGNOSTICS
    rbsherpa_pad.Update();
    ros::spinOnce();
    r.sleep();
  }
}
