#include <cstdio>
#include <functional>
#include <memory>
#include <future>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "irobot_create_msgs/action/dock_servo.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace irobot_command{

class DockClient : public rclcpp::Node
{
public:
    using NavToPos = irobot_create_msgs::action::NavigateToPosition;
    using GoalHandleNavToPos = rclcpp_action::ClientGoalHandle<NavToPos>;
    using Dock = irobot_create_msgs::action::DockServo;
    using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;

    explicit DockClient(
        const rclcpp::NodeOptions &node_options=rclcpp::NodeOptions()
    ) : Node("dock_client", node_options)
    {
        // Action client needs the action type `NavToPos`, the ROS node to add the client to (this), and the action name `/NavToPos`
        this->nav_to_pos_client_ptr_ = rclcpp_action::create_client<NavToPos>(
            this, "navigate_to_position"
        );
        this->dock_client_ptr_ = rclcpp_action::create_client<Dock>(
            this, "dock"
        );

        this->start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "start_dock", 10,
            std::bind(&DockClient::send_nav_to_pos, this, std::placeholders::_1)
        );

        this->goal_done_ = false;
    }

    bool is_goal_done()
    { return this->goal_done_; }

    void send_nav_to_pos(const std_msgs::msg::Bool & start_msg)
    {
        if (!start_msg.data) return;
        using namespace std::placeholders;  // for defining callbacks later

        // Wait for action client to start up
        if (!this->nav_to_pos_client_ptr_)
        { RCLCPP_ERROR(this->get_logger(), "Action client not initialized"); }

        // Error! Action server might have crashed.
        if (!this->nav_to_pos_client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = NavToPos::Goal();
        goal_msg.achieve_goal_heading = true;
        goal_msg.goal_pose.pose.position.x = this->gx;
        goal_msg.goal_pose.pose.position.y = this->gy;
        // Init quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 
            this->gtheta*M_PI/180.0
        );
        goal_msg.goal_pose.pose.orientation.w = q.getW();
        goal_msg.goal_pose.pose.orientation.x = q.getX();
        goal_msg.goal_pose.pose.orientation.y = q.getY();
        goal_msg.goal_pose.pose.orientation.z = q.getZ();

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavToPos>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&DockClient::nav_to_pos_goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&DockClient::nav_to_pos_result_callback, this, _1);

        // Send goal to the action server
        auto goal_handle_future = this->nav_to_pos_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void send_dock_command(){
        using namespace std::placeholders;  // for defining callbacks later

        // Wait for action client to start up
        if (!this->dock_client_ptr_)
        {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized, trying again");
            return;
        }

        // Error! Action server might have crashed.
        if (!this->dock_client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Dock Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = Dock::Goal();     // This Action has no content.
        RCLCPP_INFO(this->get_logger(), "Sending Dock goal");

        auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&DockClient::dock_goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&DockClient::dock_result_callback, this, _1);

        // Send goal to the action server
        auto goal_handle_future = this->dock_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavToPos>::SharedPtr nav_to_pos_client_ptr_;
    rclcpp_action::Client<Dock>::SharedPtr dock_client_ptr_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;

    bool goal_done_;
    const float gx = -0.05;
    const float gy = 0.0;
    const float gtheta = 0.0;

    void nav_to_pos_goal_response_callback(GoalHandleNavToPos::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void nav_to_pos_result_callback(const GoalHandleNavToPos::WrappedResult & result)
    {
        if (result.code==rclcpp_action::ResultCode::SUCCEEDED){
            this->send_dock_command();
            return;
        } else {
            RCLCPP_ERROR(this->get_logger(), "NavToPos met an error.");
        }
    }

    void dock_goal_response_callback(GoalHandleDock::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void dock_result_callback(const GoalHandleDock::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            this->goal_done_ = true;
            return;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        }
        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        }
    }

};  // class DockClient

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<irobot_command::DockClient>();

    while(!action_client->is_goal_done()){
        rclcpp::spin_some(action_client);
    }

    rclcpp::shutdown();
    return 0;
}
