#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ocp_cpp/visibility_control.h"

namespace action_server
{
    class ActionServer: public rclcpp::Node
    {
        public:
            using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
            using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

            ACTION_SERVER_PUBLIC
            explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("action_server", options)
            {
                using namespace std::placeholders;

                this->action_server_ = rclcpp_action::create_server<Fibonacci>(
                    this,
                    "fibonacci",
                    std::bind(&ActionServer::handle_goal, this, _1, _2);
                    std::bind(&ActionServer::handle_cancel, this, _1);
                    std::bind(&ActionServer::handle_accepted, this, _1)
                );
            }

        private:
            rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const Fibonacci::Goal> goal
            )
            {
                RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleFibonacci> goal_handle
            )
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }
            
            
            void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                using namespace std::placeholders;
                // this needs to return quickly to avoid blocking the executor, 
                std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
            }

            
            void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Executing goal");
                rclcpp::Rate loop_rate(1);
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<Fibonacci::Feedback>();
                auto & sequence  = feedback->partial_sequence;
                sequence.push_back(0);
                sequence.push_back(1);

                auto result = std::make_shared<Fibonacci::Result>();

                for (int i = 0; (i < goal->order) && (rclcpp::ok()); ++i)
                {
                    if (goal_handle->is_canceling())
                    {
                        result->sequence = sequence;
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal cancaled");
                        return;
                    }
                    // Update sequence if not canceling
                    sequence.push_back(sequence[i] + sequence[i-1]);
                    // Publish Feedback
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Publish feedback");

                    loop_rate.sleep();
                }
                // Check if goal is done
                if (rclcpp::ok())
                {
                    result->sequence = sequence;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                }
            }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_server::ActionServer)