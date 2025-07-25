
#include <ros/ros.h>
#include <fstream>
#include <cmath>

// ROS Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>

// Custom Messages
#include <iq_gnc/ObbDetection.h>
#include <iq_gnc/ObbDetections.h>

// GNC functions
#include <gnc_functions.hpp>

// Constants
const double LOOP_FREQUENCY = 20.0;
const char* CAMERA_TOPIC = "/sky_vision/down_cam/img_raw";
const char* CONFIG_FILE_PATH = "/home/d/sky_ws/src/iq_gnc/src/config.txt";

// A struct to hold all configuration parameters
struct MissionConfig {
    int mode = 0;
    double setpointX = 0, setpointY = 0, setpointZ = 0;
    double setpointROLL = 0, setpointPITCH = 0, setpointYAW = 0;
    double kpXY = 0, kiXY = 0, kdXY = 0;
    double kpZ = 0, kiZ = 0, kdZ = 0;
    double kpYAW = 0, kiYAW = 0, kdYAW = 0;
    double targetALT = 0;
    double velX = 0, velY = 0, velZ = 0, velYaw = 0;
};

// A simple PID controller state
struct PIDController {
    double integral = 0;
    double prev_error = 0;
};

class MissionController {
public:
    MissionController(ros::NodeHandle& nh) : 
        node_handle_(nh), 
        camera_info_received_(false),
        last_log_time_(0)
    {
        setupSubscribersAndPublishers();
        loadConfig();
    }

    void run() {
        ros::Rate rate(LOOP_FREQUENCY);
        initialize_local_frame();

        wait4connect();
        wait4start();
        
        arm();

        while (ros::ok()) {
            // Reload config every loop to allow for dynamic changes
            loadConfig();

            switch (config_.mode) {
                case 0:
                    executePositionControlMode();
                    break;
                case 1:
                    executeObjectTrackingMode();
                    break;
                case 2:
                    executeVelocityControlMode();
                    break;
                case 3:
                    executeIdleMode();
                    break;
                default:
                    ROS_WARN_THROTTLE(1.0, "Invalid mode: %d", config_.mode);
                    break;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void setupSubscribersAndPublishers() {
        std::string ros_namespace;
        node_handle_.getParam("namespace", ros_namespace);

        // Publishers
        local_pos_pub = node_handle_.advertise<geometry_msgs::PoseStamped>(ros_namespace + "/mavros/setpoint_position/local", 10);
        velocity_publisher_ = node_handle_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // Subscribers
        state_sub = node_handle_.subscribe<mavros_msgs::State>(ros_namespace + "/mavros/state", 10, state_cb);
        altitude_subscriber_ = node_handle_.subscribe<mavros_msgs::Altitude>("/mavros/altitude", 10, &MissionController::altitudeCallback, this);
        sub_ = node_handle_.subscribe("/mavros/global_position/rel_alt", 10, &MissionController::relativeAltitudeCallback, this);

        obb_subscriber_ = node_handle_.subscribe<iq_gnc::ObbDetections>("/yolo_obb/xywhr", 10, &MissionController::obbCallback, this);
        camera_info_subscriber_ = node_handle_.subscribe<sensor_msgs::Image>(CAMERA_TOPIC, 1, &MissionController::camInfoCallback, this);

        // Service Clients
        arming_client = node_handle_.serviceClient<mavros_msgs::CommandBool>(ros_namespace + "/mavros/cmd/arming");
        set_mode_client = node_handle_.serviceClient<mavros_msgs::SetMode>(ros_namespace + "/mavros/set_mode");
    
    
    }

    void loadConfig() {
        std::ifstream config_file(CONFIG_FILE_PATH);
        if (!config_file.is_open()) {
            ROS_ERROR_STREAM("Could not open config file: " << CONFIG_FILE_PATH);
            return;
        }
        std::string key;
        config_file >> key >> config_.mode;
        config_file >> key >> config_.setpointX >> key >> config_.setpointY >> key >> config_.setpointZ;
        config_file >> key >> config_.setpointROLL >> key >> config_.setpointPITCH >> key >> config_.setpointYAW;
        config_file >> key >> config_.kpXY >> key >> config_.kiXY >> key >> config_.kdXY;
        config_file >> key >> config_.kpZ >> key >> config_.kiZ >> key >> config_.kdZ;
        config_file >> key >> config_.kpYAW >> key >> config_.kiYAW >> key >> config_.kdYAW;
        config_file >> key >> config_.targetALT;
        config_file >> key >> config_.velX >> key >> config_.velY >> key >> config_.velZ >> key >> config_.velYaw;
    }

    void executePositionControlMode() {
        // geometry_msgs::PoseStamped target_pose;
        // target_pose.header.stamp = ros::Time::now();
        // target_pose.header.frame_id = "map";
        // target_pose.pose.position.x = config_.setpointX;
        // target_pose.pose.position.y = config_.setpointY;
        // target_pose.pose.position.z = config_.setpointZ;

        // tf::Quaternion q;
        // q.setRPY(config_.setpointROLL, config_.setpointPITCH, config_.setpointYAW);
        // target_pose.pose.orientation.w = q.w();
        // target_pose.pose.orientation.x = q.x();
        // target_pose.pose.orientation.y = q.y();
        // target_pose.pose.orientation.z = q.z();

        // local_pos_pub.publish(target_pose);
        set_destination(config_.setpointX, config_.setpointY, config_.setpointZ, config_.setpointYAW);

        if (ros::Time::now() - last_log_time_ > ros::Duration(1.0)) {
            ROS_INFO("Mode 0 (Position Control): Targeting X: %.2f, Y: %.2f, Z: %.2f",
                     config_.setpointX, config_.setpointY, config_.setpointZ);
            last_log_time_ = ros::Time::now();
        }
    }

    void executeObjectTrackingMode() {
        if (!camera_info_received_) {
            ROS_WARN_THROTTLE(1.0, "Mode 1: Waiting for camera resolution.");
            return;
        }
        if (latest_detections_.detections.empty()) {
            ROS_WARN_THROTTLE(1.0, "Mode 1: No object detections. Holding position.");
            publishVelocity(0, 0, 0, 0);
            return;
        }

        const auto& detection = latest_detections_.detections[0];
        float image_center_x = camera_width_ / 2.0;
        float image_center_y = camera_height_ / 2.0;

        // Pixel errors
        float error_x_pixels = detection.x_center - image_center_x;
        float error_y_pixels = detection.y_center - image_center_y;

        // PID control for XY velocity
        double dt = 1.0 / LOOP_FREQUENCY;
        
        // Normalized errors
        double err_x_norm = error_x_pixels / pow(detection.height, 1.0) * 0.15;
        double err_y_norm = error_y_pixels / pow(detection.height, 1.0) * 0.15;

        // P-term
        double p_term_x = config_.kpXY * err_x_norm;
        double p_term_y = config_.kpXY * err_y_norm;

        // I-term
        if (detection.height > 100) { // Integrate only when close
            pid_x_.integral += dt * err_x_norm;
            pid_y_.integral += dt * err_y_norm;
        }
        const double INTEGRAL_MAX = 500;
        pid_x_.integral = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, pid_x_.integral));
        pid_y_.integral = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, pid_y_.integral));
        double i_term_x = config_.kiXY * pid_x_.integral;
        double i_term_y = config_.kiXY * pid_y_.integral;

        // D-term
        double d_term_x = config_.kdXY * (err_x_norm - pid_x_.prev_error) / dt;
        double d_term_y = config_.kdXY * (err_y_norm - pid_y_.prev_error) / dt;
        pid_x_.prev_error = err_x_norm;
        pid_y_.prev_error = err_y_norm;

        // Attenuate control effort when error is small
        double m = 1.05;
        if (std::abs(error_x_pixels) < 50) {
            p_term_x *= pow(m, std::abs(error_x_pixels) - 50);
            i_term_x *= pow(m, std::abs(error_x_pixels) - 50);
            d_term_x *= pow(m, std::abs(error_x_pixels) - 50);
            pid_x_.integral = 0;
        }
        if (std::abs(error_y_pixels) < 50) {
            p_term_y *= pow(m, std::abs(error_y_pixels) - 50);
            i_term_y *= pow(m, std::abs(error_y_pixels) - 50);
            d_term_y *= pow(m, std::abs(error_y_pixels) - 50);
            pid_y_.integral = 0;
        }

        // In FLU body frame: moving forward is +X, moving right is -Y.
        // A positive y-pixel error (object below center) should move drone forward (+X vel).
        // A positive x-pixel error (object right of center) should move drone right (-Y vel).
        double vel_x = -(p_term_y + i_term_y + d_term_y);
        double vel_y = -(p_term_x + i_term_x + d_term_x);

        const double XY_VEL_LIMIT = 1.0;
        vel_x = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, vel_x));
        vel_y = std::max(-XY_VEL_LIMIT, std::min(XY_VEL_LIMIT, vel_y));

        // Altitude control
        double vel_z = 0;

        if (ros::Time::now() - last_log_time_ > ros::Duration(1.0)) {
            ROS_INFO("current_alt %lf rel alt %lf", current_altitude_.local, rel_alt_);
            last_log_time_ = ros::Time::now();
        }
        // if (current_altitude_.local > config_.targetALT + 4.0) {
        //     vel_z = -0.8;
        // } else if (current_altitude_.local < config_.targetALT - 0.2) {
        //     vel_z = 0.1;
        // } else {
        //     vel_z = (config_.targetALT - current_altitude_.local) * 0.05;
        // }
        if (rel_alt_ > config_.targetALT + 4.0) {
            vel_z = -0.8;
        } else if (rel_alt_ < config_.targetALT - 0.2) {
            vel_z = 0.1;
        } else {
            vel_z = (config_.targetALT - rel_alt_) * 0.05;
        }

        // Yaw control for alignment when close
        double yaw_rate = 0;
        if (std::abs(error_x_pixels) < 80 && std::abs(error_y_pixels) < 80 && current_altitude_.local <= 4.0) {
            double current_angle = detection.angle_rad;
            double target_angle = (detection.width >= detection.height) ? 0.0 : M_PI / 2.0;
            double yaw_error = target_angle - current_angle;

            if (yaw_error > M_PI / 2.0) yaw_error -= M_PI;
            else if (yaw_error < -M_PI / 2.0) yaw_error += M_PI;

            if (std::abs(yaw_error) > 0.05) {
                yaw_rate = config_.kpYAW * yaw_error;
            }
            yaw_rate = std::max(-0.5, std::min(0.5, yaw_rate));
        }

        publishVelocity(vel_x, vel_y, vel_z, yaw_rate);

        if (ros::Time::now() - last_log_time_ > ros::Duration(1.0)) {
            ROS_INFO("Mode 1 (Tracking): Vel(x,y,z,yaw): (%.2f, %.2f, %.2f, %.2f), Alt: %.2f",
                     vel_x, vel_y, vel_z, yaw_rate, current_altitude_.local);
            last_log_time_ = ros::Time::now();
        }
    }

    void executeVelocityControlMode() {
        publishVelocity(config_.velX, config_.velY, config_.velZ, config_.velYaw);
        if (ros::Time::now() - last_log_time_ > ros::Duration(1.0)) {
            ROS_INFO("Mode 2 (Velocity Control): Vel(x,y,z,yaw): (%.2f, %.2f, %.2f, %.2f)",
                     config_.velX, config_.velY, config_.velZ, config_.velYaw);
            last_log_time_ = ros::Time::now();
        }
    }

    void executeIdleMode() {
        publishVelocity(0, 0, 0, 0);
        if (ros::Time::now() - last_log_time_ > ros::Duration(1.0)) {
            ROS_INFO("Mode 3 (Idle): Holding position.");
            last_log_time_ = ros::Time::now();
        }
    }

    void publishVelocity(double vx, double vy, double vz, double vyaw) {
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
        vel_msg.velocity.x = vx;
        vel_msg.velocity.y = vy;
        vel_msg.velocity.z = vz;
        vel_msg.yaw_rate = vyaw;
        velocity_publisher_.publish(vel_msg);
    }

    // Callbacks
    void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg) {
        current_altitude_ = *msg;
    }
    void relativeAltitudeCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        rel_alt_ = msg->data;
    }


    void obbCallback(const iq_gnc::ObbDetections::ConstPtr& msg) {
        latest_detections_ = *msg;
    }

    void camInfoCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!camera_info_received_) {
            camera_height_ = msg->height;
            camera_width_ = msg->width;
            camera_info_received_ = true;
            ROS_INFO("Captured camera resolution: Width=%.0f, Height=%.0f", camera_width_, camera_height_);
            camera_info_subscriber_.shutdown(); // Unsubscribe after capturing
        }
    }

    // Node handle, publishers, subscribers
    ros::NodeHandle node_handle_;
    ros::Publisher velocity_publisher_;
    ros::Subscriber altitude_subscriber_;
    ros::Subscriber sub_;
    ros::Subscriber obb_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Time last_log_time_;

    // State
    MissionConfig config_;
    mavros_msgs::Altitude current_altitude_; double rel_alt_;
    iq_gnc::ObbDetections latest_detections_;
    float camera_height_;
    float camera_width_;
    bool camera_info_received_;

    // PID states
    PIDController pid_x_;
    PIDController pid_y_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_node_clear");
    ros::NodeHandle nh;
    try {
        MissionController controller(nh);
        controller.run();
    } catch (const std::exception& e) {
        ROS_FATAL("Unhandled exception: %s", e.what());
        return 1;
    }
    return 0;
}
