#include <math.h>
#include <SBUS.h>
#include <ODriveArduino.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDriveStamped.h"
#include "ros_lib/geometry_msgs/Twist.h"
#include "src/AckermannGeometry.h"
#include "src/RotorArm.h"
#include "src/Steering.h"
#include "src/Throttle.h"

#define FOLD_SW 14
#define UNFOLD_SW 15
#define EN1 9
#define EN2 10
#define PWM1 11

int Motor_speed = 100;

RotorArm arm_FR(11, 12, 2,  14, 24);
RotorArm arm_RR(26, 25, 5,  14, 27);
RotorArm arm_FL(18, 19, 6,  14, 28);
RotorArm arm_RL(15, 16, 29, 14, 17);

ros::NodeHandle nh;

double ack_steer =  0.0;
double ack_throt =  0.0;

void ackCb(const ackermann_msgs::AckermannDriveStamped& ack_msg) {
  ack_steer = ack_msg.drive.steering_angle;
  ack_throt = ack_msg.drive.speed / 3.6;
}

ros::Subscriber<ackermann_msgs::AckermannDriveStamped> sub("/Ackermann/command/joy", &ackCb );
//ros::Subscriber<geometry_msgs::Twist> sub("ackermann_msgs", &ackCb );

#define DEBUG_SERIAL  Serial
#define SBUS_SERIAL   Serial1
#define ODRIVE_SERIAL Serial3

#define DEBUG_SERIAL_BAUDRATE 115200
#define ODRIVE_SERIAL_BAUDRATE 115200

#define STEERING_BIAS 1.5
#define THROTTLE_BIAS -6.3

ODriveArduino odrive(ODRIVE_SERIAL);
Steering steering(Serial2, 115200, 23, 2.0, 1, 2);
SBUS x8r(SBUS_SERIAL);
AckermannGeometry ackermann_geometry;

float channels[16];
#define filter_size 3
float filter[filter_size][16] = {0};
int  switches[16] = {0};
const float filter_LB = -0.25;
const float filter_HB = +0.25;


bool failSafe;
bool lostFrame;

float target_steering_degree;
double target_wheel_rpm;
double target_wheel_rps;

int requested_state;

void setup() {
  //    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
  //    DEBUG_SERIAL.println("Setting parameters...");

  nh.initNode();
  nh.subscribe(sub);

  ODRIVE_SERIAL.begin(ODRIVE_SERIAL_BAUDRATE);
  for (int axis = 0; axis < 2; axis++) {
    ODRIVE_SERIAL << "w axis" << axis << ".controller.config.vel_limit " << 5000.0f << '\n';
    ODRIVE_SERIAL << "w axis" << axis << ".motor.config.current_lim " << 100.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }
  //    DEBUG_SERIAL.println("ODriveArduino");

  requested_state = AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
  //    DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
  //    DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
  odrive.run_state(0, requested_state, false); // don't wait
  odrive.run_state(1, requested_state, false); // don't wait

  x8r.begin();
}


//bool getSBUS()
//{
//  static int filt_count = 0;
//  static float val = 0;
//  for (int sample = 0; sample < filter_size; sample++)
//  {
//    if (!x8r.readCal(&channels[0], &failSafe, &lostFrame))
//      return false;
//    else
//    {
//      for (int filt_idx = 7; filt_idx < 15; filt_idx++)
//      {
//        filter[filt_count][filt_idx] = channels[filt_idx];
//      }
//      val = 0;
//      for (int sample = 0; sample < filter_size; sample++)
//        val += filter[sample][filt_idx];
//      val = val / filter_size;
//
//      if (val < filter_LB)
//        switches[filt_idx] = -1;
//      else if (filter_LB < val && val < filter_HB)
//        switches[filt_idx] = 0;
//      else
//        switches[filt_idx] = 1;
//    }
//    filt_count++;
//  }
//  return true;
//}

void loop() {
    if (x8r.readCal(&channels[0], &failSafe, &lostFrame)) {
//  if (getSBUS()){
    for (int i = 0; i < 16; i++)
    {
      DEBUG_SERIAL << "idx:" << i << "\t" << channels[i] << '\n';
    }


    nh.spinOnce();
    if (channels[12] > 0) {
      target_steering_degree = ack_steer;
      target_wheel_rpm = ack_throt;
      if (target_wheel_rpm < 2.0 && target_wheel_rpm > -2.0) target_wheel_rpm = 0.0;

      ackermann_geometry.calculate(target_steering_degree, target_wheel_rpm);
      target_wheel_rps = target_wheel_rpm/60;
      odrive.SetVelocity(0, target_wheel_rps);
      odrive.SetVelocity(1, -target_wheel_rps);
      steering.rotateAckermannAngle(ackermann_geometry);

      //      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      //      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      //      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      //      odrive.run_state(0, requested_state, false);
      //      odrive.run_state(1, requested_state, true);
      //
      //      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      //      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      //      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      //      odrive.run_state(0, requested_state, false);
      //      odrive.run_state(1, requested_state, true);

      //requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      //      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      //      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      //      odrive.run_state(0, requested_state, false); // don't wait
      //      odrive.run_state(1, requested_state, false); // don't wait
      //      delay(100);
    }
    else {
      target_steering_degree = *(channels + 0) * 40.0 + STEERING_BIAS;
      if (target_steering_degree < 2.0 && target_steering_degree > -2.0) target_steering_degree = 0.0;

      target_wheel_rpm = ((*(channels + 1) * 150) + THROTTLE_BIAS);
      if (target_wheel_rpm < 2.0 && target_wheel_rpm > -2.0) target_wheel_rpm = 0.0;
      DEBUG_SERIAL << "target_wheel_rpm" << target_wheel_rpm << '\n';

      ackermann_geometry.calculate(target_steering_degree, target_wheel_rpm);

      //      DEBUG_SERIAL << "target Angle: " << target_steering_degree << '\n';
      //            DEBUG_SERIAL << "Left Angle  : " << ackermann_geometry.left_steer_degree << '\n';
      //            DEBUG_SERIAL << "Right Angle : " << ackermann_geometry.right_steer_degree << "\n\n";
      //
      //            DEBUG_SERIAL << "target RPM: " << target_wheel_rpm << '\n';
      //            DEBUG_SERIAL << "Left RPM  : " << ackermann_geometry.left_rear_rpm << '\n';
      //      DEBUG_SERIAL << "Right RPM : " << ackermann_geometry.right_rear_rpm << "\n\n";

      target_wheel_rps = target_wheel_rpm/60;
      odrive.SetVelocity(0, -target_wheel_rps);
      odrive.SetVelocity(1, target_wheel_rps);
      steering.rotateAckermannAngle(ackermann_geometry);

      if (channels[11] > 0.5) {     // Folding Tx
        arm_FR.fold();
        arm_RR.fold();
        arm_FL.fold();
        arm_RL.fold();
      }
      else if (channels[11] < -0.5) { //Unfolding Tx
        arm_FR.unfold();
        arm_RR.unfold();
        arm_FL.unfold();
        arm_RL.unfold();
      }
      else {
        arm_FR.stopMotor();
        arm_RR.stopMotor();
        arm_FL.stopMotor();
        arm_RL.stopMotor();
      }
    }
  }
  else
  {
  }
  delay(10);
}
