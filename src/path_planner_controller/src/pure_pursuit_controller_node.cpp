// pure_pursuit_controller_node
//
// ROS2-PurePursuitControl-PathPlanning-Tracking projesindeki nav_controller/control.py
// dosyasinin pure_pursuit() ve timer_callback() kismini C++'a tasir. path_planner_node'un
// yayinladigi "plan" (nav_msgs/Path) uzerinde Pure Pursuit ile ilerler ve "cmd_vel" uretir.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

using std::placeholders::_1;

class PurePursuitControllerNode : public rclcpp::Node
{
public:
    PurePursuitControllerNode() : Node("pure_pursuit_controller_node")
    {
        this->declare_parameter<double>("lookahead_distance", 0.15);
        this->declare_parameter<double>("linear_speed", 0.1);
        this->declare_parameter<double>("goal_tolerance", 0.05);
        this->declare_parameter<double>("max_steering_angle", M_PI / 6.0);
        this->declare_parameter<double>("control_period", 0.05);

        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        const double control_period = this->get_parameter("control_period").as_double();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PurePursuitControllerNode::odomCallback, this, _1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10, std::bind(&PurePursuitControllerNode::pathCallback, this, _1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period),
            std::bind(&PurePursuitControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "pure_pursuit_controller_node baslatildi, yol bekleniyor...");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        have_odom_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_.clear();
        path_.reserve(msg->poses.size());
        for (const auto &pose : msg->poses)
        {
            path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
        path_index_ = 0;
        goal_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "Yeni yol alindi (%zu nokta), hedefe ilerleniyor...", path_.size());
    }

    void controlLoop()
    {
        if (!have_odom_ || path_.empty() || goal_reached_)
        {
            return;
        }

        const auto target = findLookaheadPoint();
        const double target_heading = std::atan2(target.second - y_, target.first - x_);
        double steering = normalizeAngle(target_heading - yaw_);

        double linear = linear_speed_;
        if (std::abs(steering) > max_steering_angle_)
        {
            const double sign = steering > 0 ? 1.0 : -1.0;
            steering = sign * (M_PI / 4.0);
            linear = 0.0;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = steering;

        const auto &goal = path_.back();
        if (std::abs(x_ - goal.first) < goal_tolerance_ && std::abs(y_ - goal.second) < goal_tolerance_)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Hedefe ulasildi.");
        }

        cmd_vel_pub_->publish(cmd);
    }

    // path_index_'ten baslayarak robota lookahead_distance_'tan daha uzak ilk noktayi bulur
    // (python versiyonundaki pure_pursuit() ile ayni mantik). Bulunamazsa yolun son noktasi donulur.
    std::pair<double, double> findLookaheadPoint()
    {
        for (size_t i = path_index_; i < path_.size(); ++i)
        {
            const double dx = x_ - path_[i].first;
            const double dy = y_ - path_[i].second;
            if (std::hypot(dx, dy) > lookahead_distance_)
            {
                path_index_ = i;
                return path_[i];
            }
        }
        path_index_ = path_.size() - 1;
        return path_.back();
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::pair<double, double>> path_;
    size_t path_index_ = 0;
    bool have_odom_ = false;
    bool goal_reached_ = false;

    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    double lookahead_distance_;
    double linear_speed_;
    double goal_tolerance_;
    double max_steering_angle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitControllerNode>());
    rclcpp::shutdown();
    return 0;
}
