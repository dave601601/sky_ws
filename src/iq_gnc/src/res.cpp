
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

#include <gnc_functions.hpp>


#define CAM_TOPIC "/sky_vision/down_cam/img_raw"

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







int mode = 0;
double setpointX = 0;
double setpointY = 0;
double setpointZ = 0;
double setpointROLL = 0;
double setpointPITCH = 0;
double setpointYAW = 0;
double kpXY = 0.0;
double kiXY = 0.0;
double kdXY = 0.0;
double kpZ = 0.0;
double kiZ = 0.0;
double kdZ = 0.0;
double kpYAW = 0.0;
double kiYAW = 0.0;
double kdYAW = 0.0;
double targetALT = 0.0;
double velx = 0;
double vely = 0;
double velz = 0;
double velyaw = 0;


const double DURATION = 20.0;
const double dt = 1 / DURATION;



ResolutionCapturer res_capturer;

ros::Time last_print_time;

mavros_msgs::Altitude current_drone_altitude;
void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    current_drone_altitude = *msg;
}


iq_gnc::ObbDetections latest_obb_detections;
void obb_detections_cb(const iq_gnc::ObbDetections::ConstPtr& msg){
    latest_obb_detections = *msg;
}



ros::Publisher vel_pub;
ros::Subscriber obb_xywhr_sub;
int my_init_publisher_subscriber(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
	local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10);
	global_lla_pos_pub = controlnode.advertise<geographic_msgs::GeoPoseStamped>((ros_namespace + "/mavros/setpoint_position/global").c_str(), 10);
	global_lla_pos_pub_raw = controlnode.advertise<mavros_msgs::GlobalPositionTarget>((ros_namespace + "/mavros/setpoint_raw/global").c_str(), 10);
	currentPos = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
	state_sub = controlnode.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
    
	arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
	land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
	set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
	takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
	command_client = controlnode.serviceClient<mavros_msgs::CommandLong>((ros_namespace + "/mavros/cmd/command").c_str());
	auto_waypoint_pull_client = controlnode.serviceClient<mavros_msgs::WaypointPull>((ros_namespace + "/mavros/mission/pull").c_str());
	auto_waypoint_push_client = controlnode.serviceClient<mavros_msgs::WaypointPush>((ros_namespace + "/mavros/mission/push").c_str());
	auto_waypoint_set_current_client = controlnode.serviceClient<mavros_msgs::WaypointSetCurrent>((ros_namespace + "/mavros/mission/set_current").c_str());
	
    vel_pub = controlnode.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    obb_xywhr_sub = controlnode.subscribe<iq_gnc::ObbDetections>("/yolo_obb/xywhr", 10, obb_detections_cb);
    return 0;
}


tf::Quaternion q;
geometry_msgs::PoseStamped target_pose; // Define target_pose for this mode
int mode0(){
    tf::Quaternion q;
    double roll_target,  pitch_target, yaw_target, x_target, y_target, z_target;
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
        ROS_INFO("Mode 0 (Position Control): Targeting X: %.2lf, Y: %.2lf, Z: %.2lf",
        target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        last_print_time = ros::Time::now();
    }


    return 0;
}



double prevx = 0; double prevy = 0; double prevz = 0; double prevyaw = 0;
const double XY_VEL_LIMIT = 1.0;
double integerrx = 0; double integerry = 0;
double prevpixelx = 0; double prevpixely = 0;
const double ALTITUDE_DEADBAND = 0.1;
const double ALPHAYAW = 0.1;
const double Z_VEL_LIMIT = 1.0; 
double preverrx = 0;
double preverry = 0;

int mode1(){
    mavros_msgs::PositionTarget vel_msg;
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
        // cmd_vel.twist.linear.x = 0;
        // cmd_vel.twist.linear.y = 0;
        // cmd_vel.twist.linear.z = 0;
        // cmd_vel.twist.angular.z = 0; // Maintain current yaw
        // cmd_vel_pub.publish(cmd_vel);
        
        vel_msg.velocity.x = 0;
        vel_msg.velocity.y = 0;
        vel_msg.velocity.z = 0;
        vel_msg.yaw_rate = 0;
        vel_pub.publish(vel_msg);
        ROS_WARN_THROTTLE(1.0, "Mode 1: No OBB detections received. Drone is holding position (sending zero velocity).");
    }
    else if (!res_capturer.isCaptured()){
        // Camera resolution not captured yet, cannot calculate pixel errors correctly.
        // cmd_vel.twist.linear.x = 0;
        // cmd_vel.twist.linear.y = 0;
        // cmd_vel.twist.linear.z = 0;
        // cmd_vel.twist.angular.z = 0;
        // cmd_vel_pub.publish(cmd_vel);
        vel_msg.velocity.x = 0;
        vel_msg.velocity.y = 0;
        vel_msg.velocity.z = 0;
        vel_msg.yaw_rate = 0;
        vel_pub.publish(vel_msg);
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
        // cmd_vel.twist.linear.y = -kpXY * (error_x_pixels / image_center_x); // Normalized error to prevent huge velocities
        vel_msg.velocity.y = -kpXY * (error_x_pixels / image_center_x);

        // - Object below (positive error_y_pixels) -> Drone needs to move FORWARD.
        // In FLU body frame: moving forward is POSITIVE X velocity.
        // cmd_vel.twist.linear.x = kpXY * (error_y_pixels / image_center_y); // Normalized error
        vel_msg.velocity.x = kpXY * (error_y_pixels / image_center_y);

        // Clamp XY velocities to defined limits for safety
        // cmd_vel.twist.linear.x = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, cmd_vel.twist.linear.x));
        // cmd_vel.twist.linear.y = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, cmd_vel.twist.linear.y));
        vel_msg.velocity.x = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, vel_msg.velocity.x));
        vel_msg.velocity.y = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, vel_msg.velocity.y));

        // Altitude control (Proportional)
        double altitude_error = targetALT - current_drone_altitude.local; // Positive if drone is too low
        if (altitude_error > ALTITUDE_DEADBAND) { // Drone is too low, move up
            // cmd_vel.twist.linear.z = kpZ * altitude_error;
            vel_msg.velocity.z = kpZ * altitude_error;
        } else if (altitude_error < -ALTITUDE_DEADBAND) { // Drone is too high, move down
            // cmd_vel.twist.linear.z = kpZ * altitude_error; // error is negative, so velocity will be negative
            vel_msg.velocity.z = kpZ * altitude_error;
        } else {
            // cmd_vel.twist.linear.z = 0; // Maintain current altitude within deadband
            vel_msg.velocity.z = 0;
        }

        // Clamp Z velocity
        // cmd_vel.twist.linear.z = std::max(-Z_VEL_LIMIT, std::min(Z_VEL_LIMIT, cmd_vel.twist.linear.z));
        vel_msg.velocity.z = std::max(-Z_VEL_LIMIT, std::min(Z_VEL_LIMIT, vel_msg.velocity.z));
        vel_msg.velocity.z = 0;

        // Yaw control can be added here if 'detection.rotation_angle' needs to be used for alignment
        // cmd_vel.twist.angular.z = 0; // No yaw control for now, drone maintains current yaw
        // cmd_vel.twist.angular.x = 0;
        // cmd_vel.twist.angular.y = 0;

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
        double ptermx = kpXY * currerrx;
        double ptermy = kpXY * currerry;

        if (detection.height > 100) integerrx += dt * currerrx;
        if (detection.height > 100) integerry += dt * currerry;

        const double INTEGMAX = 500;
        integerrx = std::max(-INTEGMAX, std::min(INTEGMAX, integerrx));
        integerry = std::max(-INTEGMAX, std::min(INTEGMAX, integerry));

        double itermx = kiXY*integerrx;
        double itermy = kiXY*integerry;

        double dtermx = kdXY * (currerrx - preverrx) / dt; preverrx = currerrx;
        double dtermy = kdXY * (currerry - preverry) / dt; preverry = currerry;

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

        if (current_drone_altitude.local > targetALT + 4) {vel_msg.velocity.z = -0.8; }
        else if (current_drone_altitude.local < targetALT - .2) {vel_msg.velocity.z = 0.1;}
        else {vel_msg.velocity.z = (targetALT - current_drone_altitude.local) * 0.05;}

        if (std::abs(error_x_pixels) < 80 && std::abs(error_y_pixels) < 80){

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
                    target_yaw_rate = kpYAW * yaw_error;
                }

                // Clamp yaw rate
                target_yaw_rate = std::max(-0.5, std::min(0.5, target_yaw_rate));
                vel_msg.yaw_rate = ALPHAYAW * target_yaw_rate + (1 - ALPHAYAW) * prevyaw;
            }
        }

        prevyaw = vel_msg.yaw_rate;
        prevx = vel_msg.velocity.x;
        prevy = vel_msg.velocity.y;
        prevz = vel_msg.velocity.z;

        prevpixelx = error_x_pixels;
        prevpixely = error_y_pixels;

        vel_pub.publish(vel_msg);

        // Log control data for debugging (rate-limited)
        if (ros::Time::now() - last_print_time > ros::Duration(1.0)){ // Print every 0.5 seconds
            ROS_INFO("Mode 1 (Velocity Control): Target Detected. Pixel Error (X,Y): (%.0f, %.0f). (X,Y,Z yaw): (%.3f, %.3f, %.3f, %.3f). Current Alt: %.2lf, w:%.2lf h:%.2lf r:%.2lf",
                        error_x_pixels, error_y_pixels, vel_msg.velocity.x, vel_msg.velocity.y, vel_msg.velocity.z, vel_msg.yaw_rate, current_drone_altitude.local, detection.width, detection.height, detection.angle_rad*180/M_PI);
            ROS_INFO("p %.2lf %.2lf i %.2lf %.2lf d %.2lf %.2lf", ptermx, ptermy, itermx, itermy, dtermx, dtermy);
            last_print_time = ros::Time::now();
            }
        }

        return 0;
}

int mode2(){
    mavros_msgs::PositionTarget vel_msg;
    vel_msg.header.stamp = ros::Time::now();
    vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW;
    vel_msg.velocity.x = velx;
    vel_msg.velocity.y = vely;
    vel_msg.velocity.z = velz;
    vel_msg.yaw_rate = velyaw;

    vel_pub.publish(vel_msg);

    // Log position data (rate-limited)
    if (ros::Time::now() - last_print_time > ros::Duration(1.0)){
        ROS_INFO("Mode 3: xyz vel: %.2lf, Y: %.2lf, Z: %.2lf, Yaw: %.2lf",
                    velx, vely, velz, velyaw);
        last_print_time = ros::Time::now();
    }
    return 0;
}

int mode3(){
    return 0;
}

int main(int argc, char** argv){
    
    std::string filename = "/home/d/icarus/catkin_ws/src/iq_gnc/src/config.txt";
    std::ifstream config(filename); // Open the file for reading, renamed to 'config'

    // Check if the file was successfully opened
    if (!config.is_open()) {
        std::cerr << "Error: Could not open config file: " << filename << std::endl;
        return false; // Return false if file opening failed
    }

    
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle mission_node;
    ros::Rate rate(DURATION);
    
    my_init_publisher_subscriber(mission_node);

    wait4connect();
    wait4start();
    initialize_local_frame();

    arm();
    
    while(ros::ok()){
        std::string d; 
        config >> d >> mode;
        config >> d >> setpointX;
        config >> d >> setpointY;
        config >> d >> setpointZ;
        config >> d >> setpointROLL;
        config >> d >> setpointPITCH;
        config >> d >> setpointYAW;
        config >> d >> kpXY;
        config >> d >> kiXY;
        config >> d >> kdXY;
        config >> d >> kpZ;
        config >> d >> kiZ;
        config >> d >> kdZ;
        config >> d >> kpYAW;
        config >> d >> kiYAW;
        config >> d >> kdYAW;
        config >> d >> targetALT;
        config >> d >> velx;
        config >> d >> vely;
        config >> d >> velz;
        config >> d >> velyaw;
        ROS_INFO("mode: %d, setpointX: %.2lf, setpointY: %.2lf, setpointZ: %.2lf, setpointROLL: %.2lf, setpointPITCH: %.2lf, setpointYAW: %.2lf, kpXY: %.2lf, kiXY: %.2lf, kdXY: %.2lf, kpYAW: %.2lf, kiYAW: %.2lf, kdYAW: %.2lf, targetALT: %.2lf, velx: %.2lf, vely: %.2lf, velz: %.2lf\n",
                mode, setpointX, setpointY, setpointZ, setpointROLL, setpointPITCH, setpointYAW,
                kpXY, kiXY, kdXY, kpYAW, kiYAW, kdYAW, targetALT, velx, vely, velz);

        switch (mode) {
            case 0:
                mode0();
                break;
            case 1:
                mode1();
                break;
            case 2:
                mode2();
                break;
            case 3:
                mode3();
                break;
            default:
                ROS_INFO("invalid mode");
                break;

        }

        ros::spinOnce();
        rate.sleep();

    }

    
    

    

    return 0;
}