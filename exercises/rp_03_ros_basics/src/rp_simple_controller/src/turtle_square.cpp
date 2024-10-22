#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

// Allows us to use literals for time units
using namespace std::chrono_literals;
// Just to be sure that we have pi available on every platform [required up
// c++20]
constexpr float pi = 3.14159265358979323846;

class TurtleSquareNode : public rclcpp::Node {
public:
    TurtleSquareNode() : Node("turtle_square_node") {
        // Create the publisher for velocity commands [binded to
        // geometry_msgs::msg::Twist message] For simplicity, publish on
        // "/turtle1/cmd_vel"
        // TODO here!
        velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Create the subscriber for the turtle pose [binded to turtlesim::msg::Pose
        // message] For simplicity, subscribe to "/turtle1/pose"
        // TODO here!
        position_sub = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleSquareNode::pose_callback, this, std::placeholders::_1));

        // Create the timer for the control loop
        // The timer is set to call the timer_callback function every 1millisecond
        // [1ms]
        timer = this->create_wall_timer(
            1ms, std::bind(&TurtleSquareNode::timer_callback, this));
    }

private:
    /** 
     * Timer callback.
     * Increase the current time by dt and execute the control loop.
     * The control loop is a simple state machine that makes the turtle move in a
     * square trajectory. The machine has two states:
     * - 0: Move forward for side_length meters
     * - 1: Turn right for angle_to_turn radians
     * The completion of each state is verified as follows:
     * - [0-FORWARD] If the current time excedes [side_length / trans_vel] then
     * the state is changed to 1 [TURN]
     * - [1-TURN] If the current time excedes [angle_to_turn / rot_vel] then the
     * state is changed to 0 [FORWARD]
     * In both cases, reset current_time to 0.
     *
     * The translational velocity [v] and angular velocity [w] are set based on
     * the current state:
     * - [0-FORWARD] : v = trans_vel | w = 0
     * - [1-TURN] : v = 0 | w = rot_vel
     *
     * Once the velocities are set, create a geometry_msgs::msg::Twist message and
     * store the computed velocities accordingly. Remember that, to control the
     * turtle, [v] should be written in the linear.x field and [w] in the
     * angular.z field. Finally, publish the message on the velocity_pub
     * publisher.
     */
    void timer_callback() {
        current_time += dt;

        // TODO here!
        auto velocity_message = geometry_msgs::msg::Twist();
        if (state == 0) {               // Currently moving forward
            // Check if the turtle moved forward for the required distance
            if (current_time > (side_length / trans_vel)) {
                state = 1;              // Switch to turn state
                current_time = 0.0f;    // Reset time
            } else {
                velocity_message.linear.x = trans_vel;
                velocity_message.angular.z = 0.0;
            }
        } else {                        // Currently turning
            // Check if the turtle has turned the required angle
            if (current_time > (angle_to_turn / rot_vel)) {
                state = 0;              // Switch to forward state
                current_time = 0.0f;    // Reset time
            } else {
                velocity_message.linear.x = 0.0;
                velocity_message.angular.z = rot_vel;
            }
        }

        velocity_pub->publish(velocity_message);
    }

    /**
     * Pose callback.
     * Once the turtle pose is received, print the turtle pose on the console
     * using RCLCPP_INFO macro. If you have problems, you might also use std::cout
     */
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        // TODO here!
        RCLCPP_INFO(this->get_logger(), "Turtle Pose (x, y, theta) = (%f, %f, %f)", msg->x, msg->y, msg->theta);
    }
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer;
    // Velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
    // Position subscriber
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr position_sub;

    // Simulation parameters
    const float dt = 1e-3;  // timer_callback is called every 0.001 seconds [if
                          // changed, remember to change the timer period]
    float current_time = 0.0f;
    bool state = 0;  // 0 forward - 1 turn
    // Trajectory parameters
    float side_length = 2;
    float angle_to_turn = pi * 0.5;  // To turn of 90 degrees
    // Pre-set velocities.
    float trans_vel = 1.0;
    float rot_vel = 1.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleSquareNode>());
    rclcpp::shutdown();
    return 0;
}
