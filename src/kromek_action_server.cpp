#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_planning_interfaces/action/kromek.hpp"
#include "kromek_ros/Kromek.hpp"
#include "kromek_ros/Helpers.hpp"
#include "kromek_ros/Kromek.cpp"
#include "kromek_ros/Helpers.cpp"

// Serial numbers
const unsigned GR1 = 1819;
const unsigned SIGMA50 = 8155;
Kromek::Driver _kromek;
std::shared_ptr<Kromek::Sensor> _sensor = _kromek.device(SIGMA50);

class KromekActionServer : public rclcpp::Node
{
public:
  using Kromek = robot_planning_interfaces::action::Kromek;
  using GoalHandleKromek = rclcpp_action::ServerGoalHandle<Kromek>;

    explicit KromekActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("kromek_action_server", options)
    {
        this->declare_parameter<int>("sensor", SIGMA50);
        this->get_parameter("sensor", sensor_num_);

        using namespace std::placeholders;
        
        this->action_server_ = rclcpp_action::create_server<Kromek>(
            this,
            "kromek",
            std::bind(&KromekActionServer::handle_goal, this, _1, _2),
            std::bind(&KromekActionServer::handle_cancel, this, _1),
            std::bind(&KromekActionServer::handle_accepted, this, _1));
    }

private:
rclcpp_action::Server<Kromek>::SharedPtr action_server_;
int sensor_num_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Kromek::Goal> goal)
  {
    if (_sensor)
    {
        _sensor->start();
        _sensor->clear();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Could not find sensor: %d. Cancelling goal", SIGMA50);
    }

    RCLCPP_INFO(this->get_logger(), "Received goal request with scan time %d", goal->time);
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleKromek> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleKromek> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&KromekActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleKromek> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Scanning");
    rclcpp::Rate loop_rate(8);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Kromek::Feedback>();
    auto & current_count = feedback->count;
    auto & time_remaining = feedback->sec_remain;
    auto result = std::make_shared<Kromek::Result>();
    rclcpp::Time start_time = this->get_clock()->now();
    rclcpp::Time current_time = this->get_clock()->now();

    while((current_time.seconds() - start_time.seconds()) <= static_cast<double>(goal->time) && rclcpp::ok())
    {
        current_time = this->get_clock()->now();
        current_count = _sensor->counts();
        feedback->count = current_count;
        RCLCPP_INFO(this->get_logger(), "Current count: %d", current_count);

        // Check if there is an external cancel request
        if (goal_handle->is_canceling()) {
        result->count = current_count;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Scan canceled");
        return;
        }
        
        time_remaining = static_cast<double>(goal->time) - (current_time.seconds() - start_time.seconds());
        //RCLCPP_INFO(this->get_logger(), "Time remaining: %f", time_remaining);
        feedback->sec_remain = time_remaining;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->count = current_count / goal->time;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Scan finished, final count: %d", result->count);
    }
  } // function execute

};  // class KromekActionServer


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<KromekActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}