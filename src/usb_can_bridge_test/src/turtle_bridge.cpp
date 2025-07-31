#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <std_msgs/msg/string.hpp>

class TurtleBridge : public rclcpp::Node {
public:
    TurtleBridge() : Node("turtle_bridge") {
        // Parameters
        this->declare_parameter("turtle_name", "motor_turtle");
        this->declare_parameter("trace_enabled", true);
        this->declare_parameter("velocity_scale", 1.0);
        
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        trace_enabled_ = this->get_parameter("trace_enabled").as_bool();
        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Starting Turtle Bridge for motor visualization");
        
        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&TurtleBridge::jointStateCallback, this, std::placeholders::_1));
            
        motor_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "motor_status", 10,
            std::bind(&TurtleBridge::motorStatusCallback, this, std::placeholders::_1));
            
        // Publishers
        turtle_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            turtle_name_ + "/cmd_vel", 10);
            
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "turtle_bridge_status", 10);
        
        // Service clients for turtle setup
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        pen_client_ = this->create_client<turtlesim::srv::SetPen>(turtle_name_ + "/set_pen");
        
        // Initialize turtle state
        turtle_x_ = 5.5;
        turtle_y_ = 5.5;
        turtle_theta_ = 0.0;
        last_position_ = 0.0;
        
        // Wait for turtlesim and spawn custom turtle
        setupTurtle();
        
        // Timer for status updates
        status_timer_ = this->create_timer(
            std::chrono::milliseconds(500),
            std::bind(&TurtleBridge::publishStatus, this));
    }

private:
    std::string turtle_name_;
    bool trace_enabled_;
    double velocity_scale_;
    
    // Turtle state
    double turtle_x_, turtle_y_, turtle_theta_;
    double last_position_;
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    void setupTurtle() {
        // Wait for turtlesim services
        while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for turtlesim spawn service...");
        }
        
        // Spawn custom turtle
        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_request->x = turtle_x_;
        spawn_request->y = turtle_y_;
        spawn_request->theta = turtle_theta_;
        spawn_request->name = turtle_name_;
        
        auto spawn_future = spawn_client_->async_send_request(spawn_request);
        
        // Wait for spawn result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), spawn_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Successfully spawned turtle: %s", turtle_name_.c_str());
            
            // Configure pen for tracing
            setupPen();
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to spawn turtle, using default turtle1");
            turtle_name_ = "turtle1";
            turtle_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        }
    }
    
    void setupPen() {
        // Wait for pen service
        while (!pen_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for pen service...");
        }
        
        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        pen_request->r = 0;    // Red for motor movement
        pen_request->g = 255;  // Green 
        pen_request->b = 0;    // Blue
        pen_request->width = 3;
        pen_request->off = trace_enabled_ ? 0 : 1;
        
        auto pen_future = pen_client_->async_send_request(pen_request);
        RCLCPP_INFO(this->get_logger(), "Configured pen for turtle tracing");
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.empty() || msg->velocity.empty()) {
            return;
        }
        
        // Find traction joint
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "traction_joint") {
                double velocity = msg->velocity[i];
                double position = msg->position[i];
                
                // Convert motor motion to turtle movement
                updateTurtleFromMotor(velocity, position);
                break;
            }
        }
    }
    
    void motorStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Log motor status for debugging
        RCLCPP_DEBUG(this->get_logger(), "Motor Status: %s", msg->data.c_str());
    }
    
    void updateTurtleFromMotor(double motor_velocity, double motor_position) {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Convert motor velocity to turtle linear velocity
        // Scale motor velocity (rad/s) to reasonable turtle speed (m/s)
        twist_msg.linear.x = motor_velocity * velocity_scale_;
        
        // Add some angular velocity based on position change for more interesting movement
        double position_change = motor_position - last_position_;
        twist_msg.angular.z = position_change * 0.1; // Small angular component
        
        last_position_ = motor_position;
        
        // Limit velocities to reasonable turtle ranges
        twist_msg.linear.x = std::max(-2.0, std::min(2.0, twist_msg.linear.x));
        twist_msg.angular.z = std::max(-2.0, std::min(2.0, twist_msg.angular.z));
        
        turtle_cmd_pub_->publish(twist_msg);
        
        // Update estimated turtle position (for status)
        double dt = 0.02; // Assume ~50Hz update rate
        turtle_x_ += twist_msg.linear.x * cos(turtle_theta_) * dt;
        turtle_y_ += twist_msg.linear.x * sin(turtle_theta_) * dt;
        turtle_theta_ += twist_msg.angular.z * dt;
        
        // Keep theta in [-pi, pi]
        while (turtle_theta_ > M_PI) turtle_theta_ -= 2 * M_PI;
        while (turtle_theta_ < -M_PI) turtle_theta_ += 2 * M_PI;
    }
    
    void publishStatus() {
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "Turtle: " + turtle_name_ + 
                         ", Pos: (" + std::to_string(turtle_x_) + ", " + std::to_string(turtle_y_) + ")" +
                         ", Theta: " + std::to_string(turtle_theta_) +
                         ", Motor Pos: " + std::to_string(last_position_);
        
        status_pub_->publish(status_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TurtleBridge>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
