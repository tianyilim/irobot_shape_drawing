#include <cstdio>
#include <functional>
#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "std_msgs/msg/bool.hpp"

namespace irobot_command{

class iRobotUndockClient : public rclcpp::Node
{
public:
    using Undock = irobot_create_msgs::action::Undock;
    using GoalHandleUndock = rclcpp_action::ClientGoalHandle<Undock>;

    explicit iRobotUndockClient(
        const rclcpp::NodeOptions &node_options=rclcpp::NodeOptions()
    ) : Node("irobot_undock_client", node_options)
    {
        // Action client needs the action type `Undock`, the ROS node to add the client to (this), and the action name `/undock`
        this->client_ptr_ = rclcpp_action::create_client<Undock>(
            this, "undock"
        );

        this->timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::milliseconds(2000),    // Don't call too often.
            std::bind(&iRobotUndockClient::send_goal, this) );

        this->start_pub_ = this->create_publisher<std_msgs::msg::Bool>("start_trace_seven", 10);
    }

    bool is_goal_done() const {return this->goal_done_;}

    void send_goal()
    {
        using namespace std::placeholders;  // for defining callbacks later
        this->goal_done_ = false;

        // Wait for action client to start up
        if (!this->client_ptr_)
        {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized, trying again");
            return;
        }

        // Error! Action server might have crashed.
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = Undock::Goal();     // This Action has no content.

        RCLCPP_INFO(this->get_logger(), "Sending Undock goal");

        auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&iRobotUndockClient::goal_response_callback, this, _1);   // the placeholders refer to the function arguments in the callback
        // send_goal_options.feedback_callback =
        //     std::bind(&iRobotUndockClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&iRobotUndockClient::result_callback, this, _1);

        // Send goal to the action server
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    }

private:
    rclcpp_action::Client<Undock>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;

    bool goal_done_;

    void goal_response_callback(GoalHandleUndock::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            this->timer_->cancel();             // Cancel timer so action only called once
        }
    }

    void result_callback(const GoalHandleUndock::WrappedResult & result)
    {
        this->goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            // Let the other server know it's time to start
            std_msgs::msg::Bool start_msg;
            start_msg.data = true;
            start_pub_->publish(start_msg);
            break;
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

        RCLCPP_INFO(this->get_logger(), "Result received: is_docked=%s", result.result->is_docked ? "true" : "false");
    }
};  // class iRobotUndockClient

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<irobot_command::iRobotUndockClient>();

    while(!action_client->is_goal_done()){
        rclcpp::spin_some(action_client);
    }

    rclcpp::shutdown();
    return 0;
}
