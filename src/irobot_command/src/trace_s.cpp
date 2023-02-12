#include <cstdio>
#include <functional>
#include <memory>
#include <future>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Util functions (should be in a .h but these are quick)
double get_diff_between_angles(double a1, double a2){
    /* Returns smallest a1-a2, assuming both in radians */
    double a = a1-a2;
    if (a > M_PI){
        a -= M_PI*2;
    } else if (a < -M_PI) {
        a += M_PI*2;
    }
    return a;
}

double get_yaw_from_quat(double x, double y, double z, double w){
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

namespace irobot_command{

class TraceSClient : public rclcpp::Node
{
public:
    using NavToPos = irobot_create_msgs::action::NavigateToPosition;
    using GoalHandleNavToPos = rclcpp_action::ClientGoalHandle<NavToPos>;

    explicit TraceSClient(
        const rclcpp::NodeOptions &node_options=rclcpp::NodeOptions()
    ) : Node("trace_s_client", node_options)
    {
        // Action client needs the action type `NavToPos`, the ROS node to add the client to (this), and the action name `/NavToPos`
        this->client_ptr_ = rclcpp_action::create_client<NavToPos>(
            this, "navigate_to_position"
        );

        this->start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "start_trace_s", 10,
            std::bind(&TraceSClient::get_start, this, std::placeholders::_1)
        );

        this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&TraceSClient::get_odom, this, std::placeholders::_1)
        );

        this->coord_idx_ = 0;   // Which coordinate in the "S" to move to
        this->goal_done_ = false;
        this->start_flag_ = false;

        this->viz_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("viz_on", 10);
        this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        this->dock_pub_ = this->create_publisher<std_msgs::msg::Bool>("start_dock", 10);

        // Declare parameters
        this->declare_parameter<double>("WAYPOINT_TOL");
        this->declare_parameter<double>("V_K");
        this->declare_parameter<double>("V_MAX");
        this->declare_parameter<double>("W_MAX");
    }

    bool is_goal_done()
    {
        return this->goal_done_;
    }

private:
    rclcpp_action::Client<NavToPos>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr viz_flag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trace_seven_end_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dock_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    bool start_flag_;
    bool goal_done_;
    size_t coord_idx_;
    // Hard-coded waypoints to get the robot to trace out an S
    const std::vector<double> x_coords{-1.25, -1.323, -1.35, -1.35, -1.323, -1.25, -1.15, -1.05, -0.95,  -0.877, -0.85, -0.85, -0.823, -0.75, -0.65, -0.55, -0.45, -0.377, -0.35, -0.35, -0.377, -0.45, -0.55};
    const std::vector<double> y_coords{ 0.973, 0.9,    0.8,   0.4,   0.3,    0.227, 0.2,   0.2,   0.227,  0.3,    0.4,   0.8,   0.9,    0.973, 1.0,   1.0,   0.973, 0.9,    0.8,   0.4,   0.3,    0.227, 0.2};
    const size_t START_VIZ_IDX = 0;             // Waypoint index to start visualization
    const size_t END_VIZ_IDX = x_coords.size(); // Waypoint index to end visualization

    // Respond to the start flag for us to start moving
    void get_start(const std_msgs::msg::Bool &msg){
        if (msg.data) {
            start_flag_ = true;
            std_msgs::msg::Bool msg_true;
            msg_true.data = true;
            this->viz_flag_pub_->publish(msg_true);     // Start tracing breadcrumbs
        }
    }

    // In response to odometry messages, publish new things onto cmd_vel
    void get_odom(const nav_msgs::msg::Odometry &msg){
        if (!start_flag_) return;   // Only start when we are cleared to do so

        // Get parameter values
        double V_K = this->get_parameter("V_K").get_parameter_value().get<double>();
        double V_MAX = this->get_parameter("V_MAX").get_parameter_value().get<double>();
        double W_MAX = this->get_parameter("W_MAX").get_parameter_value().get<double>();
        double WAYPOINT_TOL = this->get_parameter("WAYPOINT_TOL").get_parameter_value().get<double>();

        double gx = x_coords[coord_idx_];
        double gy = y_coords[coord_idx_];    // goal x and goal_y
        double dx = gx-msg.pose.pose.position.x;
        double dy = gy-msg.pose.pose.position.y;
        double gtheta = atan2( dy, dx );

        double ctheta = get_yaw_from_quat(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        );

        // Add numerical epsilon to avoid div by 0
        double d_theta = get_diff_between_angles(gtheta, ctheta) + 1e-6;
        // RCLCPP_INFO(this->get_logger(), "ctheta: %0.2fdeg, gtheta: %0.2fdeg, d_theta: %0.2fdeg",
        //     (ctheta*180.0/M_PI), (gtheta*180.0/M_PI), (d_theta*180.0/M_PI));

        double V = std::max(0.0, -V_K*abs(d_theta)+V_MAX);      // Linear relation with lin velocity and angle 
        // try using SIN instead of linear (smoother function)
        double W;
        if (d_theta > M_PI_2) {
            W = W_MAX;
        } else if (d_theta < -M_PI_2) {
            W = -W_MAX;
        } else {
            W = sin(d_theta);
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = V;
        cmd.angular.z = W;

        // If we are close enough to the target, go to the next one
        if (sqrt(dx*dx+dy*dy) < WAYPOINT_TOL) {
            RCLCPP_INFO(this->get_logger(), "gxy: (%0.2f, %0.2f), cxy: (%0.2f, %0.2f), dx: %0.2f, dy: %0.2f, distance to waypoint: %0.2f",
                gx, gy, msg.pose.pose.position.x, msg.pose.pose.position.y, dx, dy, sqrt(dx*dx+dy*dy)
            );

            coord_idx_++;
            RCLCPP_INFO(this->get_logger(), "Next waypoint idx: %ld", coord_idx_);
            if (coord_idx_ == x_coords.size()){
                // We have reached the goal and can stop
                RCLCPP_INFO(this->get_logger(), "Reached last waypoint. Done.");

                goal_done_ = true;
                cmd.linear.x = 0.0;
                cmd.linear.z = 0.0;
                std_msgs::msg::Bool msg_tx;
                msg_tx.data = false;
                viz_flag_pub_->publish(msg_tx);
                msg_tx.data = true;
                dock_pub_->publish(msg_tx);     // just use the same obj for convenience
            }
        }

        // Publish command velocity
        this->cmd_vel_pub_->publish(cmd);
    }
};  // class TraceSClient

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<irobot_command::TraceSClient>();

    while(!action_client->is_goal_done()){
        rclcpp::spin_some(action_client);
    }

    rclcpp::shutdown();
    return 0;
}