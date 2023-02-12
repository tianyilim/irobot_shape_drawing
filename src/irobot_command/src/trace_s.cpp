#include <cstdio>
#include <functional>
#include <memory>
#include <future>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace irobot_command{

class TraceSClient : public rclcpp::Node
{
public:
    using NavToPos = irobot_create_msgs::action::NavigateToPosition;
    using GoalHandleNavToPos = rclcpp_action::ClientGoalHandle<NavToPos>;

    explicit TraceSClient(
        const rclcpp::NodeOptions &node_options=rclcpp::NodeOptions()
    ) : Node("trace_seven_client", node_options)
    {
        // Action client needs the action type `NavToPos`, the ROS node to add the client to (this), and the action name `/NavToPos`
        this->client_ptr_ = rclcpp_action::create_client<NavToPos>(
            this, "navigate_to_position"
        );

        this->start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "start_trace_seven", 10,
            std::bind(&TraceSClient::send_goal, this, std::placeholders::_1)
        );

        this->coord_idx_ = 0;   // Which coordinate in the "7" to move to
        this->goal_done_ = false;

        this->viz_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("viz_on", 10);
        this->trace_seven_end_pub_ = this->create_publisher<std_msgs::msg::Bool>("start_trace_s", 10);
    }

    bool is_goal_done()
    {
        return this->goal_done_;
    }

    void send_goal(const std_msgs::msg::Bool & _)
    {
        using namespace std::placeholders;  // for defining callbacks later
        _;      // So we don't have warning for unused argument

        // Wait for action client to start up
        if (!this->client_ptr_)
        {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        // Error! Action server might have crashed.
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        // Start/stop leaving breadcrumbs
        std_msgs::msg::Bool msg;
        if (this->coord_idx_ == this->START_VIZ_IDX){
            RCLCPP_INFO(this->get_logger(), "Set viz ON");
            msg.data = true;
            this->viz_flag_pub_->publish(msg);
        }
        if (this->coord_idx_ == this->END_VIZ_IDX){
            RCLCPP_INFO(this->get_logger(), "Set viz OFF");
            msg.data = false;
            this->viz_flag_pub_->publish(msg);
        }

        auto goal_msg = NavToPos::Goal();
        // goal_msg.goal_pose.header.stamp;
        // goal_msg.goal_pose.header.frame_id = "";
        goal_msg.achieve_goal_heading = true;
        goal_msg.goal_pose.pose.position.x = this->x_coords[this->coord_idx_];
        goal_msg.goal_pose.pose.position.y = this->y_coords[this->coord_idx_];
        // Init quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 
            this->t_coords[this->coord_idx_]*M_PI/180.0
        );
        goal_msg.goal_pose.pose.orientation.w = q.getW();
        goal_msg.goal_pose.pose.orientation.x = q.getX();
        goal_msg.goal_pose.pose.orientation.y = q.getY();
        goal_msg.goal_pose.pose.orientation.z = q.getZ();

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavToPos>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&TraceSClient::goal_response_callback, this, _1);   // the placeholders refer to the function arguments in the callback
        send_goal_options.feedback_callback =
            std::bind(&TraceSClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&TraceSClient::result_callback, this, _1);

        // Send goal to the action server
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    }

private:
    rclcpp_action::Client<NavToPos>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr viz_flag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trace_seven_end_pub_;

    bool goal_done_;
    size_t coord_idx_;
    const size_t START_VIZ_IDX = 1;     // Waypoint index to start visualization
    const size_t END_VIZ_IDX = 3;       // Waypoint index to end visualization
    std::vector<float> x_coords{-1.35, -1.35, -0.35, -1.15};
    std::vector<float> y_coords{-1.0, -0.2, -0.7, 1.0};
    std::vector<float> t_coords{90.0, -22.565, 115.201, -180.0};
    // Hard-coded waypoints to get the robot to go

    void goal_response_callback(GoalHandleNavToPos::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleNavToPos::SharedPtr,
        const std::shared_ptr<const NavToPos::Feedback> feedback)
    {
        std::ostringstream status;
        if (feedback->navigate_state == 2) {
            status << feedback->remaining_travel_distance << "m";
        } else {
            status << feedback->remaining_angle_travel << "rad";
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Nav state: %d, %s",
            feedback->navigate_state,
            status.str().c_str()
        );
    }

    void result_callback(const GoalHandleNavToPos::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            this->coord_idx_++;
            // We know we are done when we have reached the last element in the coordinates
            std_msgs::msg::Bool msg;    // Dummy data to chuck into a message
            msg.data = true;
            if (this->coord_idx_ == this->x_coords.size()) {
                this->goal_done_ = true;
                this->trace_seven_end_pub_->publish(msg);
            } else {
                this->send_goal(msg);
            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        // Get euler angle from result callback
        tf2::Quaternion q(
            result.result->pose.pose.orientation.x,
            result.result->pose.pose.orientation.y,
            result.result->pose.pose.orientation.z,
            result.result->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Result received: x:%0.2f, y:%0.2f, th:%0.2f",
            result.result->pose.pose.position.x,
            result.result->pose.pose.position.y,
            yaw);
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
