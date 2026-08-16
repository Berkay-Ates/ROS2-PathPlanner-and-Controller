// path_planner_node
//
// Ports the planning part (costmap inflation + A* + path smoothing) of
// nav_controller/control.py (from the ROS2-PurePursuitControl-PathPlanning-Tracking
// project) to C++. The Pure Pursuit tracking part lives in a separate node
// (pure_pursuit_controller_node); the bridge between them is the nav_msgs/Path
// message (the "plan" topic).
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

using std::placeholders::_1;

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node")
    {
        this->declare_parameter<int>("expansion_size", 6);
        this->declare_parameter<int>("smoothing_samples_per_segment", 5);

        expansion_size_ = this->get_parameter("expansion_size").as_int();
        smoothing_samples_ = this->get_parameter("smoothing_samples_per_segment").as_int();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&PathPlannerNode::mapCallback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathPlannerNode::odomCallback, this, _1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::QoS(10), std::bind(&PathPlannerNode::goalCallback, this, _1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);

        RCLCPP_INFO(this->get_logger(), "path_planner_node started, waiting for a goal...");
    }

private:
    // --- callbacks ---------------------------------------------------------

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        have_odom_ = true;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        have_goal_ = true;
        need_plan_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_x_, goal_y_);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!need_plan_ || !have_goal_ || !have_odom_)
        {
            return;
        }

        const auto &info = msg->info;
        const double resolution = info.resolution;
        const double origin_x = info.origin.position.x;
        const double origin_y = info.origin.position.y;
        const int width = static_cast<int>(info.width);
        const int height = static_cast<int>(info.height);

        if (width <= 0 || height <= 0 || resolution <= 0.0)
        {
            return;
        }

        // OccupancyGrid -> occupied-cell grid. Only confirmed walls (100) get inflated;
        // unknown (-1) cells are counted as obstacles directly, without inflation. If we
        // inflated unknown cells too, the small not-yet-scanned free area around the robot
        // would be swallowed entirely and the robot would get trapped in its own start cell.
        std::vector<bool> occupied(static_cast<size_t>(width) * height, false);
        for (size_t i = 0; i < occupied.size(); ++i)
        {
            occupied[i] = (msg->data[i] == 100);
        }
        inflate(occupied, width, height, expansion_size_);
        for (size_t i = 0; i < occupied.size(); ++i)
        {
            if (msg->data[i] < 0)
            {
                occupied[i] = true;
            }
        }

        const int start_col = static_cast<int>((robot_x_ - origin_x) / resolution);
        const int start_row = static_cast<int>((robot_y_ - origin_y) / resolution);
        const int goal_col = static_cast<int>((goal_x_ - origin_x) / resolution);
        const int goal_row = static_cast<int>((goal_y_ - origin_y) / resolution);

        if (!inBounds(start_row, start_col, width, height) || !inBounds(goal_row, goal_col, width, height))
        {
            RCLCPP_WARN(this->get_logger(), "Start or goal is outside the map bounds.");
            return;
        }

        occupied[static_cast<size_t>(start_row) * width + start_col] = false;  // the robot's own cell is always passable

        const auto grid_path = aStar(occupied, width, height, {start_row, start_col}, {goal_row, goal_col});
        if (grid_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "A* could not find a path.");
            return;
        }

        std::vector<std::pair<double, double>> world_path;
        world_path.reserve(grid_path.size());
        for (const auto &cell : grid_path)
        {
            world_path.emplace_back(cell.second * resolution + origin_x, cell.first * resolution + origin_y);
        }

        const auto smoothed = catmullRomSmooth(world_path, smoothing_samples_);
        publishPath(smoothed, msg->header.frame_id);

        need_plan_ = false;
        RCLCPP_INFO(this->get_logger(), "Path published (%zu points), heading to goal...", smoothed.size());
    }

    // --- helpers ---------------------------------------------------------

    static bool inBounds(int row, int col, int width, int height)
    {
        return row >= 0 && row < height && col >= 0 && col < width;
    }

    // Inflates obstacle cells by 'expansion' cells (to leave a safety margin around walls).
    static void inflate(std::vector<bool> &occupied, int width, int height, int expansion)
    {
        if (expansion <= 0)
        {
            return;
        }
        const std::vector<bool> original = occupied;
        for (int row = 0; row < height; ++row)
        {
            for (int col = 0; col < width; ++col)
            {
                if (!original[static_cast<size_t>(row) * width + col])
                {
                    continue;
                }
                for (int dr = -expansion; dr <= expansion; ++dr)
                {
                    for (int dc = -expansion; dc <= expansion; ++dc)
                    {
                        const int nr = row + dr;
                        const int nc = col + dc;
                        if (inBounds(nr, nc, width, height))
                        {
                            occupied[static_cast<size_t>(nr) * width + nc] = true;
                        }
                    }
                }
            }
        }
    }

    // 8-connected A*. Grid index = row * width + col. If the goal is unreachable,
    // returns the visited cell closest to the goal (same behavior as the "closest_node"
    // fallback in the Python version).
    static std::vector<std::pair<int, int>> aStar(
        const std::vector<bool> &occupied, int width, int height,
        std::pair<int, int> start, std::pair<int, int> goal)
    {
        static const int neighbors[8][2] = {
            {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        auto heuristic = [](std::pair<int, int> a, std::pair<int, int> b) {
            return std::hypot(a.first - b.first, a.second - b.second);
        };
        auto index = [width](int row, int col) { return static_cast<size_t>(row) * width + col; };

        const size_t n = static_cast<size_t>(width) * height;
        const size_t start_idx = index(start.first, start.second);
        const size_t goal_idx = index(goal.first, goal.second);

        std::vector<double> gscore(n, std::numeric_limits<double>::infinity());
        std::vector<long> came_from(n, -1);
        std::vector<bool> visited(n, false);

        using QItem = std::pair<double, size_t>;  // (fscore, index)
        std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

        gscore[start_idx] = 0.0;
        open.push({heuristic(start, goal), start_idx});

        while (!open.empty())
        {
            const size_t current = open.top().second;
            open.pop();
            if (visited[current])
            {
                continue;
            }
            visited[current] = true;
            if (current == goal_idx)
            {
                break;
            }

            const int cr = static_cast<int>(current / width);
            const int cc = static_cast<int>(current % width);
            for (const auto &d : neighbors)
            {
                const int nr = cr + d[0];
                const int nc = cc + d[1];
                if (!inBounds(nr, nc, width, height))
                {
                    continue;
                }
                const size_t n_idx = index(nr, nc);
                if (occupied[n_idx] || visited[n_idx])
                {
                    continue;
                }
                const double step = (d[0] != 0 && d[1] != 0) ? std::sqrt(2.0) : 1.0;
                const double tentative = gscore[current] + step;
                if (tentative < gscore[n_idx])
                {
                    gscore[n_idx] = tentative;
                    came_from[n_idx] = static_cast<long>(current);
                    open.push({tentative + heuristic({nr, nc}, goal), n_idx});
                }
            }
        }

        size_t end_idx = goal_idx;
        if (came_from[goal_idx] == -1 && goal_idx != start_idx)
        {
            double best_dist = std::numeric_limits<double>::infinity();
            long best = -1;
            for (size_t i = 0; i < n; ++i)
            {
                if (!visited[i])
                {
                    continue;
                }
                const int r = static_cast<int>(i / width);
                const int c = static_cast<int>(i % width);
                const double d = heuristic({r, c}, goal);
                if (d < best_dist)
                {
                    best_dist = d;
                    best = static_cast<long>(i);
                }
            }
            if (best == -1)
            {
                return {};
            }
            end_idx = static_cast<size_t>(best);
        }

        std::vector<std::pair<int, int>> path;
        long cur = static_cast<long>(end_idx);
        while (cur != -1)
        {
            path.emplace_back(static_cast<int>(cur / width), static_cast<int>(cur % width));
            if (static_cast<size_t>(cur) == start_idx)
            {
                break;
            }
            cur = came_from[cur];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Produces a smooth path through the corner points using a Catmull-Rom cubic spline
    // (a dependency-free C++ equivalent of the scipy-based bspline_planning()).
    static std::vector<std::pair<double, double>> catmullRomSmooth(
        const std::vector<std::pair<double, double>> &pts, int samples_per_segment)
    {
        if (pts.size() < 3 || samples_per_segment < 1)
        {
            return pts;
        }

        auto at = [&](long i) -> const std::pair<double, double> & {
            if (i < 0) return pts.front();
            if (static_cast<size_t>(i) >= pts.size()) return pts.back();
            return pts[static_cast<size_t>(i)];
        };

        std::vector<std::pair<double, double>> result;
        result.reserve(pts.size() * samples_per_segment);

        for (size_t i = 0; i + 1 < pts.size(); ++i)
        {
            const auto &p0 = at(static_cast<long>(i) - 1);
            const auto &p1 = at(static_cast<long>(i));
            const auto &p2 = at(static_cast<long>(i) + 1);
            const auto &p3 = at(static_cast<long>(i) + 2);

            for (int s = 0; s < samples_per_segment; ++s)
            {
                const double t = static_cast<double>(s) / samples_per_segment;
                const double t2 = t * t;
                const double t3 = t2 * t;

                const double x = 0.5 * ((2 * p1.first) + (-p0.first + p2.first) * t +
                                         (2 * p0.first - 5 * p1.first + 4 * p2.first - p3.first) * t2 +
                                         (-p0.first + 3 * p1.first - 3 * p2.first + p3.first) * t3);
                const double y = 0.5 * ((2 * p1.second) + (-p0.second + p2.second) * t +
                                         (2 * p0.second - 5 * p1.second + 4 * p2.second - p3.second) * t2 +
                                         (-p0.second + 3 * p1.second - 3 * p2.second + p3.second) * t3);
                result.emplace_back(x, y);
            }
        }
        result.push_back(pts.back());
        return result;
    }

    void publishPath(const std::vector<std::pair<double, double>> &points, const std::string &frame_id)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = frame_id.empty() ? "map" : frame_id;

        for (const auto &p : points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = p.first;
            pose.pose.position.y = p.second;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
    }

    // --- members ---------------------------------------------------------

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    bool have_odom_ = false;
    bool have_goal_ = false;
    bool need_plan_ = false;

    double robot_x_ = 0.0, robot_y_ = 0.0;
    double goal_x_ = 0.0, goal_y_ = 0.0;

    int expansion_size_;
    int smoothing_samples_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
