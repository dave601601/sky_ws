/**

* @file offb_node.cpp

* @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight

* Stack and tested in Gazebo Classic SITL

*/


#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/PolygonStamped.h> // Keep if you use this elsewhere, commented out in logic below

#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

#include <mavros_msgs/CommandBool.h>

#include <mavros_msgs/SetMode.h>

#include <mavros_msgs/State.h>

#include <mavros_msgs/Altitude.h>

#include <mavros_msgs/PositionTarget.h>

#include <tf/tf.h> // For Quaternion conversions

#include <iostream>

#include <fstream>

#include <unistd.h>

#include <cstdio>

#include <cmath> // For std::abs and std::max/min


#include <iq_gnc/ObbDetection.h>

#include <iq_gnc/ObbDetections.h>

#include <exception>


// --- Constants ---

#define CAM_TOPIC "/vtolcam/usb_cam/image_raw"

const double TARGET_HOVER_ALTITUDE = 2.0; // Desired hovering altitude above the target in meters

const double ALTITUDE_DEADBAND = 0.1; // Meters, deadband for altitude control to reduce oscillations

const double XY_VEL_LIMIT = 1.0; // Max horizontal velocity (m/s)

const double Z_VEL_LIMIT = 1.0; // Max vertical velocity (m/s)


const double DURATION = 20.0;


const double ALPHAXY = 0.5;

const double ALPHAZ = 0.1;

const double ALPHAYAW = 0.1;

const double ALPHAERR = 0.4;

// --- Global Variables (for simplicity in callbacks, updated by subscribers) ---

mavros_msgs::State current_state;

iq_gnc::ObbDetections latest_obb_detections;

mavros_msgs::Altitude current_drone_altitude;


// --- Callbacks for ROS Subscribers ---

void state_cb(const mavros_msgs::State::ConstPtr& msg){

current_state = *msg;

}


// Keeping this callback and variable, but the `center()` function using it is not used

// in the primary OBB tracking logic based on xywhr.detections.

// If you intend to use this data, ensure it's integrated properly.

geometry_msgs::PolygonStamped xy4;

void change_xy4(const geometry_msgs::PolygonStamped::ConstPtr& msg){

xy4 = *msg;

}


void obb_detections_cb(const iq_gnc::ObbDetections::ConstPtr& msg){

latest_obb_detections = *msg;

}


void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){

current_drone_altitude = *msg;

}


// --- Helper Class to Capture Camera Resolution ---

class ResolutionCapturer

{

public:

ResolutionCapturer() : nh_("~"), height_(0), width_(0), captured_(false)

{

camsub_ = nh_.subscribe(CAM_TOPIC, 1, &ResolutionCapturer::camCallback, this);

ROS_INFO("ResolutionCapturer: Waiting for camera resolution on topic %s...", CAM_TOPIC);

}

void camCallback(const sensor_msgs::ImageConstPtr& msg)

{

if (!captured_) { // Only capture once

height_ = msg->height;

width_ = msg->width;

captured_ = true;

ROS_INFO("ResolutionCapturer: Captured camera resolution: Width=%.0f, Height=%.0f. Shutting down subscriber.", width_, height_);

camsub_.shutdown(); // Unsubscribe after capturing

}

}


inline float height() const { return height_; }

inline float width() const { return width_; }

inline bool isCaptured() const { return captured_; }


private:

ros::NodeHandle nh_;

ros::Subscriber camsub_;

float height_, width_;

bool captured_;

};



// --- Main ROS Node Function ---

int main(int argc, char **argv)

{

ros::init(argc, argv, "offb_node");

ros::NodeHandle nh;


// --- ROS Subscribers ---

ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>

("mavros/state", 10, state_cb);

ros::Subscriber altitude_sub = nh.subscribe<mavros_msgs::Altitude>("/mavros/altitude", 10, altitude_cb);

ros::Subscriber obb_polygon_sub = nh.subscribe<geometry_msgs::PolygonStamped>("/yolo_obb/detected_polygons", 10, change_xy4); // Subscribing, but not used in tracking logic below

ros::Subscriber obb_xywhr_sub = nh.subscribe<iq_gnc::ObbDetections>("/yolo_obb/xywhr", 10, obb_detections_cb);

// --- ROS Publishers ---

ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>

("mavros/setpoint_position/local", 10);

ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

// ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 10);

ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);


// --- ROS Service Clients ---

ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>

("mavros/cmd/arming");

ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>

("mavros/set_mode");


// --- Helper Objects ---

ResolutionCapturer res_capturer;


// --- Setpoint Publishing Rate ---

ros::Rate rate(DURATION); // MAVROS requires setpoints at >2Hz for offboard mode


// --- Wait for FCU Connection ---

ROS_INFO("Waiting for FCU connection...");

while(ros::ok() && !current_state.connected){

ros::spinOnce();

rate.sleep();

}

ROS_INFO("FCU connected.");


// --- Wait for Camera Resolution to be Captured ---

ROS_INFO("Waiting for camera resolution...");

while(ros::ok() && !res_capturer.isCaptured()){

ros::spinOnce();

rate.sleep();

}

ROS_INFO("Camera resolution captured: Width=%.0f, Height=%.0f", res_capturer.width(), res_capturer.height());


// --- Initial Pose for Offboard Mode ---

// This pose is sent repeatedly to enable offboard mode before actual control begins.

geometry_msgs::PoseStamped initial_pose;

initial_pose.pose.position.x = 0;

initial_pose.pose.position.y = 0;

initial_pose.pose.position.z = TARGET_HOVER_ALTITUDE;

initial_pose.pose.orientation.w = 1.0; // Facing forward (no rotation)


// Send a few setpoints before trying to switch to offboard mode.

// This is crucial for MAVROS to establish a stream of setpoints, preventing rejection of offboard command.

ROS_INFO("Sending initial setpoints to establish stream...");

for(int i = 0; ros::ok() && i < 100; ++i){ // Send 100 setpoints (5 seconds at 20Hz)

initial_pose.header.stamp = ros::Time::now();

local_pos_pub.publish(initial_pose);

ros::spinOnce();

rate.sleep();

}

ROS_INFO("Initial setpoints sent. Ready for offboard mode.");


mavros_msgs::SetMode offb_set_mode;

offb_set_mode.request.custom_mode = "OFFBOARD";


mavros_msgs::CommandBool arm_cmd;

arm_cmd.request.value = true;


ros::Time last_mode_request = ros::Time::now();

ros::Time last_arming_request = ros::Time::now();

ros::Time last_print_time = ros::Time::now(); // For rate-limited ROS_INFO messages


double prevx = 0;double prevy = 0;double prevz = 0; double prevyaw = 0;

double prevpixelx = 0; double prevpixely = 0;

double alpha = 0.09;

double preverrx = 0;

double preverry = 0;

double integerrx = 0;

double integerry = 0;


// --- Main Control Loop ---

while(ros::ok()){

// --- Offboard Mode and Arming Logic ---

// Continuously request offboard mode and arming if not already set.

if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_mode_request > ros::Duration(5.0))){

if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){

ROS_INFO("Offboard mode enabled.");

}

last_mode_request = ros::Time::now();

} else {

if( !current_state.armed && (ros::Time::now() - last_arming_request > ros::Duration(5.0))){

if( arming_client.call(arm_cmd) && arm_cmd.response.success){

ROS_INFO("Vehicle armed.");

}

last_arming_request = ros::Time::now();

}

}


// --- Read Control Parameters from File ---

// Reads target pose (for mode 0), control mode, and proportional gains.

double x_target, y_target, z_target, roll_target, pitch_target, yaw_target;

double xvel, yvel, zvel, yawvel;

double Kp_xy, Kp_z ,Kp_yaw; // Proportional gains for XY and Z control

int control_mode;

double targetalt = .2;

double kpxy, kixy, kdxy;

double dt = 1 / DURATION;


std::ifstream posefile("/home/d/icarus/catkin_ws/src/test/src/pos.txt");

if (!posefile.is_open()) {

ROS_ERROR_THROTTLE(1.0, "Could not open pos.txt! Please ensure the file exists at /home/d/icarus/catkin_ws/src/test/src/pos.txt. Defaulting to safe hover mode.");

// Default to safe hover mode with moderate gains if file is missing.

control_mode = 1; // Default to velocity control (hover)

Kp_xy = 0.2; // Safe default Kp for XY

Kp_z = 0.3; // Safe default Kp for Z

// Target position/orientation for mode 0 will remain default if file unreadable.

} else {

// Format: x y z roll pitch yaw mode Kp_xy (Kp_z will be same as Kp_xy, or add another value to file)

posefile >> x_target >> y_target >> z_target >> roll_target >> pitch_target >> yaw_target >>

control_mode >>

Kp_xy >> Kp_yaw >>

xvel >> yvel>> zvel >> yawvel

>> targetalt

>> kpxy >> kixy >> kdxy;

Kp_z = Kp_xy; // For simplicity, use the same 'scale' from file for both XY and Z Kp.

// You can extend pos.txt if you need separate Kp_xy and Kp_z values.

posefile.close();

}

geometry_msgs::TwistStamped cmd_vel; // For publishing velocity commands

mavros_msgs::PositionTarget vel_msg;

cmd_vel.header.stamp = ros::Time::now();

cmd_vel.header.frame_id = "base_link"; // Velocities are typically in the drone's body (FLU) frame


// --- Control Logic based on `control_mode` ---

if (control_mode == 1) // Mode 1: Velocity control to hover above detected OBB

{

vel_msg.header.stamp = ros::Time::now();

vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |

mavros_msgs::PositionTarget::IGNORE_PY |

mavros_msgs::PositionTarget::IGNORE_PZ |

mavros_msgs::PositionTarget::IGNORE_AFX |

mavros_msgs::PositionTarget::IGNORE_AFY |

mavros_msgs::PositionTarget::IGNORE_AFZ |

mavros_msgs::PositionTarget::IGNORE_YAW;


if (latest_obb_detections.detections.empty()) {

// No OBB detections received. Send zero velocity to hold position.

cmd_vel.twist.linear.x = 0;

cmd_vel.twist.linear.y = 0;

cmd_vel.twist.linear.z = 0;

cmd_vel.twist.angular.z = 0; // Maintain current yaw

cmd_vel_pub.publish(cmd_vel);

ROS_WARN_THROTTLE(1.0, "Mode 1: No OBB detections received. Drone is holding position (sending zero velocity).");

}

else if (!res_capturer.isCaptured()){

// Camera resolution not captured yet, cannot calculate pixel errors correctly.

cmd_vel.twist.linear.x = 0;

cmd_vel.twist.linear.y = 0;

cmd_vel.twist.linear.z = 0;

cmd_vel.twist.angular.z = 0;

cmd_vel_pub.publish(cmd_vel);

ROS_WARN_THROTTLE(1.0, "Mode 1: Camera resolution not captured. Cannot perform OBB tracking. Holding position.");

}

else {

// Assuming we track the first detected object (you might want more sophisticated selection)

const auto& detection = latest_obb_detections.detections[0];

float image_center_x = res_capturer.width() / 2.0;

float image_center_y = res_capturer.height() / 2.0;


// Calculate error in pixels from image center

float error_x_pixels = detection.x_center - image_center_x; // Positive if object is to the right of center

float error_y_pixels = detection.y_center - image_center_y; // Positive if object is below center


// Proportional control for XY velocity (assuming a DOWNWARD-FACING CAMERA)

// - Object to the right (positive error_x_pixels) -> Drone needs to move RIGHT.

// In FLU (Forward-Left-Up) body frame: moving right is NEGATIVE Y velocity.

cmd_vel.twist.linear.y = -Kp_xy * (error_x_pixels / image_center_x); // Normalized error to prevent huge velocities

vel_msg.velocity.y = -Kp_xy * (error_x_pixels / image_center_x);


// - Object below (positive error_y_pixels) -> Drone needs to move FORWARD.

// In FLU body frame: moving forward is POSITIVE X velocity.

cmd_vel.twist.linear.x = Kp_xy * (error_y_pixels / image_center_y); // Normalized error

vel_msg.velocity.x = Kp_xy * (error_y_pixels / image_center_y);


// Clamp XY velocities to defined limits for safety

cmd_vel.twist.linear.x = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, cmd_vel.twist.linear.x));

cmd_vel.twist.linear.y = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, cmd_vel.twist.linear.y));

vel_msg.velocity.x = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, vel_msg.velocity.x));

vel_msg.velocity.y = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, vel_msg.velocity.y));


// Altitude control (Proportional)

double altitude_error = TARGET_HOVER_ALTITUDE - current_drone_altitude.local; // Positive if drone is too low

if (altitude_error > ALTITUDE_DEADBAND) { // Drone is too low, move up

cmd_vel.twist.linear.z = Kp_z * altitude_error;

vel_msg.velocity.z = Kp_z * altitude_error;

} else if (altitude_error < -ALTITUDE_DEADBAND) { // Drone is too high, move down

cmd_vel.twist.linear.z = Kp_z * altitude_error; // error is negative, so velocity will be negative

vel_msg.velocity.z = Kp_z * altitude_error;

} else {

cmd_vel.twist.linear.z = 0; // Maintain current altitude within deadband

vel_msg.velocity.z = 0;

}




// Clamp Z velocity

cmd_vel.twist.linear.z = std::max(-Z_VEL_LIMIT, std::min(Z_VEL_LIMIT, cmd_vel.twist.linear.z));

vel_msg.velocity.z = std::max(-Z_VEL_LIMIT, std::min(Z_VEL_LIMIT, vel_msg.velocity.z));

vel_msg.velocity.z = 0;

// Yaw control can be added here if 'detection.rotation_angle' needs to be used for alignment

cmd_vel.twist.angular.z = 0; // No yaw control for now, drone maintains current yaw

cmd_vel.twist.angular.x = 0;

cmd_vel.twist.angular.y = 0;

double temp;

temp = vel_msg.velocity.x;

vel_msg.velocity.x = vel_msg.velocity.y;

vel_msg.velocity.y = -temp;


// if (error_x_pixels > 10)

// //vel_msg.velocity.y = -Kp_xy;//*std::abs(error_x_pixels);

// vel_msg.velocity.y = -std::min(std::abs(Kp_xy), std::abs(error_x_pixels)/pow(detection.height, 1.15)*0.15);


// else if (error_x_pixels < -10)

// //vel_msg.velocity.y = Kp_xy;//*std::abs(error_x_pixels);

// vel_msg.velocity.y = std::min(std::abs(Kp_xy), std::abs(error_x_pixels)/pow(detection.height, 1.15)*0.15);

// else

// vel_msg.velocity.y = 0;

// error_x_pixels = ALPHAERR*error_x_pixels + preverrx;

// error_y_pixels = ALPHAERR*error_y_pixels + preverry;

double currerrx = error_x_pixels / pow(detection.height, 1.0)*0.15;

double currerry = error_y_pixels / pow(detection.height, 1.0)*0.15;

double ptermx = kpxy * currerrx;

double ptermy = kpxy * currerry;


if (detection.height > 100) integerrx += dt * currerrx;

if (detection.height > 100) integerry += dt * currerry;


const double INTEGMAX = 500;

integerrx = std::max(-INTEGMAX, std::min(INTEGMAX, integerrx));

integerry = std::max(-INTEGMAX, std::min(INTEGMAX, integerry));

double itermx = kixy*integerrx;

double itermy = kixy*integerry;

double dtermx = kdxy * (currerrx - preverrx) / dt; preverrx = currerrx;

double dtermy = kdxy * (currerry - preverry) / dt; preverry = currerry;


double abserr;

double m = 1.05;

if ((abserr = abs(error_x_pixels)) < 50) {ptermx *= pow(m, abserr-50); itermx*=pow(m, abserr-50); dtermx *=pow(m, abserr-50); integerrx = 0;}

// else if (abs(error_x_pixels) < 10) {ptermx*=0.001; itermx*=0.001;}

if ((abserr = abs(error_y_pixels)) < 50) {ptermy*=pow(m, abserr-50); itermy*=pow(m, abserr-50); dtermx*=pow(m, abserr-50); integerry = 0;}

// else if (abs(error_y_pixels) < 10) {ptermy*=0.001; itermy*=0.001;}


double velx = -(ptermy + itermy + dtermy);

double vely = -(ptermx + itermx + dtermx);

velx = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, velx));

vely = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, vely));

vel_msg.velocity.x = velx;

vel_msg.velocity.y = vely;

// vel_msg.velocity.y = alpha*vel_msg.velocity.y + (1-alpha)*prevy;

// prevy = vel_msg.velocity.y;


// if (error_y_pixels > 10)

// // vel_msg.velocity.x = -Kp_xy;//*std::abs(error_y_pixels);

// vel_msg.velocity.x = -std::min(std::abs(Kp_xy), std::abs(error_y_pixels)/pow(detection.height, 1.15)*0.15);

// else if (error_y_pixels < -10)

// // vel_msg.velocity.x = Kp_xy;//*std::abs(error_y_pixels);

// vel_msg.velocity.x = std::min(std::abs(Kp_xy), std::abs(error_y_pixels)/pow(detection.height, 1.15)*0.15);

// else

// vel_msg.velocity.x = 0;


// vel_msg.velocity.x = alpha*vel_msg.velocity.x + (1-alpha)*prevx;

// prevx = vel_msg.velocity.x;


// double anglerad = 0;

if (current_drone_altitude.local > targetalt + 4) {vel_msg.velocity.z = -0.8; }

else if (current_drone_altitude.local < targetalt - .2) {vel_msg.velocity.z = 0.1;}

else {vel_msg.velocity.z = (targetalt - current_drone_altitude.local) * 0.05;}

if (std::abs(error_x_pixels) < 80 && std::abs(error_y_pixels) < 80){

// if (detection.width >= detection.height) anglerad = detection.angle_rad;

// else anglerad = detection.angle_rad + M_PI/2;


// if (current_drone_altitude.local > 4)

// vel_msg.velocity.z = -0.8;

// else if (current_drone_altitude.local < 1.8)

// vel_msg.velocity.z = 0.1;

// else

// {

// vel_msg.velocity.z = (2 - current_drone_altitude.local)*0.005;

// if (std::abs(detection.angle_rad) < 0.01 || std::abs(detection.angle_rad - M_PI) < 0.01)

// vel_msg.yaw_rate = 0;

// if (detection.angle_rad < M_PI )

// vel_msg.yaw_rate = -yawvel;

// else if (detection.angle_rad > M_PI)

// vel_msg.yaw_rate = yawvel;

// }

// Altitude Control (Landing)

// Gentle descent


if (current_drone_altitude.local <= 4.0){// Yaw Control

double current_angle = detection.angle_rad;

double target_angle = (detection.width >= detection.height) ? 0.0 : M_PI / 2.0;

double yaw_error = target_angle - current_angle;


// Handle angle wrap-around for shortest rotation

if (yaw_error > M_PI / 2.0) yaw_error -= M_PI;

else if (yaw_error < -M_PI / 2.0) yaw_error += M_PI;

double target_yaw_rate = 0;

if (std::abs(yaw_error) > 0.05) {

target_yaw_rate = Kp_yaw * yaw_error;

}


// Clamp yaw rate

target_yaw_rate = std::max(-0.5, std::min(0.5, target_yaw_rate));

vel_msg.yaw_rate = ALPHAYAW * target_yaw_rate + (1 - ALPHAYAW) * prevyaw;

}

}



// vel_msg.velocity.x = ALPHAXY*vel_msg.velocity.x + (1-ALPHAXY)*prevx;

// vel_msg.velocity.x = ALPHAXY*vel_msg.velocity.y + (1-ALPHAXY)*prevy;

prevyaw = vel_msg.yaw_rate;

prevx = vel_msg.velocity.x;

prevy = vel_msg.velocity.y;

prevz = vel_msg.velocity.z;


prevpixelx = error_x_pixels;

prevpixely = error_y_pixels;


vel_pub.publish(vel_msg);

// cmd_vel_pub.publish(cmd_vel); // Publish the calculated velocity commands


// Log control data for debugging (rate-limited)

if (ros::Time::now() - last_print_time > ros::Duration(1.0)){ // Print every 0.5 seconds

ROS_INFO("Mode 1 (Velocity Control): Target Detected. Pixel Error (X,Y): (%.0f, %.0f). (X,Y,Z yaw): (%.3f, %.3f, %.3f, %.3f). Current Alt: %.2f, w:%.2f h:%.2f r:%.2f",

error_x_pixels, error_y_pixels, vel_msg.velocity.x, vel_msg.velocity.y, vel_msg.velocity.z, vel_msg.yaw_rate, current_drone_altitude.local, detection.width, detection.height, detection.angle_rad*180/M_PI);

ROS_INFO("p %.2f %.2f i %.2f %.2f d %.2f %.2f", ptermx, ptermy, itermx, itermy, dtermx, dtermy);

last_print_time = ros::Time::now();

}

}

}

else if (control_mode == 0) // Mode 0: Position control to a fixed point read from `pos.txt`

{

// Convert RPY (Roll, Pitch, Yaw) to Quaternion for pose.orientation

tf::Quaternion q;

q.setRPY(roll_target, pitch_target, yaw_target);


geometry_msgs::PoseStamped target_pose; // Define target_pose for this mode

target_pose.header.stamp = ros::Time::now();

target_pose.header.frame_id = "map"; // Or "odom" depending on your TF setup. "map" is common for global setpoints.

target_pose.pose.position.x = x_target;

target_pose.pose.position.y = y_target;

target_pose.pose.position.z = z_target;


target_pose.pose.orientation.w = q.w();

target_pose.pose.orientation.x = q.x();

target_pose.pose.orientation.y = q.y();

target_pose.pose.orientation.z = q.z();


local_pos_pub.publish(target_pose); // Publish the position setpoint


// Log position data (rate-limited)

if (ros::Time::now() - last_print_time > ros::Duration(1.0)){

ROS_INFO("Mode 0 (Position Control): Targeting X: %.2f, Y: %.2f, Z: %.2f",

target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

last_print_time = ros::Time::now();

}

}

else if (control_mode==3)

{

vel_msg.header.stamp = ros::Time::now();

vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |

mavros_msgs::PositionTarget::IGNORE_PY |

mavros_msgs::PositionTarget::IGNORE_PZ |

mavros_msgs::PositionTarget::IGNORE_AFX |

mavros_msgs::PositionTarget::IGNORE_AFY |

mavros_msgs::PositionTarget::IGNORE_AFZ |

mavros_msgs::PositionTarget::IGNORE_YAW;

vel_msg.velocity.x = xvel;

vel_msg.velocity.y = yvel;

vel_msg.velocity.z = zvel;

vel_msg.yaw_rate = yawvel;


vel_pub.publish(vel_msg);

// Log position data (rate-limited)

if (ros::Time::now() - last_print_time > ros::Duration(1.0)){

ROS_INFO("Mode 3: xyz vel: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f",

xvel, yvel, zvel, yawvel);

last_print_time = ros::Time::now();

}


}

else {

// Invalid mode detected from pos.txt

ROS_WARN_THROTTLE(1.0, "Invalid control mode detected from pos.txt: %d. Expected 0 or 1. Drone holding position.", control_mode);

// Send zero velocity commands to ensure drone doesn't drift uncontrollably

cmd_vel.twist.linear.x = 0;

cmd_vel.twist.linear.y = 0;

cmd_vel.twist.linear.z = 0;

cmd_vel.twist.angular.z = 0;

cmd_vel_pub.publish(cmd_vel);

}


ros::spinOnce(); // Process incoming ROS messages

rate.sleep(); // Sleep to maintain the desired loop rate

}


return 0;

}