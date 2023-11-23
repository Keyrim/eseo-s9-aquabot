#include <iostream>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

const int DELTA = 2;

struct Point {
    int x, y;

    Point(int x, int y) : x(x), y(y) {}

    double distanceTo(const Point &other) {
        return std::hypot(double(x - other.x), double(y - other.y));
    }
};

struct Obstacle {
    Point position;
    int diametre;

    Obstacle(const Point &pos, int diam) : position(pos), diametre(diam) {}
};

struct Node {
    Point position;
    Point velocity;
    double restant_score;

    Node(const Point &pos, double f) : position(pos), restant_score(f), velocity(*new Point(0, 0)) {}

    bool operator>(const Node &other) const {
        return restant_score > other.restant_score;
    }
};

Point gps_to_Point(double latitude, double longitude) {
    double lat_rad = M_PI / 180.0 * latitude;
    double lon_rad = M_PI / 180.0 * longitude;

    double R = 6371000.0;

    double x = R * lon_rad;
    double y = R * log(tan(M_PI/4 + lat_rad/2));

    return *new Point(x, y);
}


bool isValidPoint(Point &point, const std::vector<Obstacle *> &obstacles) {
    for (auto obs : obstacles) {
        if (point.x >= (obs->position.x - obs->diametre) 
            && point.x <= (obs->position.x + obs->diametre) 
            && point.y >= (obs->position.y - obs->diametre) 
            && point.y <= (obs->position.y + obs->diametre)) {
            return false;
        }    
    }
    return true;
}

bool isInPath(const Point &neighbor, std::vector<Node *> path) {
    for (auto node : path) {
        if (neighbor.x == node->position.x && neighbor.y == node->position.y) {
            return true;
        }
    }
    return false;
}

std::vector<Point> planificationTrajectoire(Point &start, Point &initial_velocity, const Point &goal,
                                            std::vector<Obstacle *> &obstacles, std::vector<Obstacle *> &obstaclesBateau) {
    std::vector<Node*> path;
    std::vector<Point> path_point;
    bool traj_found = false;
    bool is_depart = true;
    int i = 0;

    Node* initial_node = new Node(start, start.distanceTo(goal));
    initial_node->velocity = initial_velocity;
    path.push_back(initial_node);

    while (!traj_found && i < 2000) {

        if (i >= 5) {
            is_depart = false;
        }

        Node* current = path.back();

        if (current->position.distanceTo(goal) <= 26) {
            // Objectif atteint, reconstruire la trajectoire
            while (!path.empty()) {
                path_point.push_back(path.back()->position);
                path.pop_back();
            }
            std::reverse(path_point.begin(), path_point.end());
            return path_point;
        }

        Point best_neighbor = Point(-1000, -1000);

        for (const Point& neighbor : {
                // Calculez les positions des voisins en fonction de la vitesse initiale
                Point(current->position.x + DELTA + current->velocity.x, current->position.y + current->velocity.y),
                Point(current->position.x - DELTA + current->velocity.x, current->position.y + current->velocity.y),
                Point(current->position.x + current->velocity.x, current->position.y + DELTA + current->velocity.y),
                Point(current->position.x + current->velocity.x, current->position.y - DELTA + current->velocity.y),
                Point(current->position.x + DELTA + current->velocity.x, current->position.y + DELTA + current->velocity.y),
                Point(current->position.x + DELTA + current->velocity.x, current->position.y - DELTA + current->velocity.y),
                Point(current->position.x - DELTA + current->velocity.x, current->position.y + DELTA + current->velocity.y),
                Point(current->position.x - DELTA + current->velocity.x, current->position.y - DELTA + current->velocity.y)
            }) {
            Point mutable_neighbor = neighbor;
            if (isValidPoint(mutable_neighbor, obstacles) && isValidPoint(mutable_neighbor, obstaclesBateau)&& !isInPath(neighbor, path)) {  
                
                double restant_score_tentative = mutable_neighbor.distanceTo(goal);

                
                    if (path.size() >= 2) {

                        if (current->position.distanceTo(neighbor) < path.at(path.size()-2)->position.distanceTo(neighbor)) {
                            if (best_neighbor.x == -1000 && best_neighbor.y == -1000) {
                                best_neighbor = neighbor;
                            }
                            else if (restant_score_tentative < best_neighbor.distanceTo(goal)) {
                                best_neighbor = neighbor;
                            }
                        }
                    }
                    else {
                        if (best_neighbor.x == -1000 && best_neighbor.y == -1000) {
                            best_neighbor = neighbor;
                        }
                        else if (restant_score_tentative < best_neighbor.distanceTo(goal)) {
                            best_neighbor = neighbor;
                        }
                    }
                    
                
            }
        }
        if (best_neighbor.x == 90 && best_neighbor.y == -50) {
            std::cout << "putain de merde ";
        }
        path.push_back(new Node(best_neighbor, best_neighbor.distanceTo(goal)));
        i++;
    }
    return path_point;
}

std::vector<Point> minimizePath(std::vector<Point> &path) {
    std::vector<Point> path_point;
    path_point.push_back(path.at(0));

    for (int i = 0; i < path.size()-3; i++ ) {
        Point first_point = path.at(i);
        Point second_point = path.at(i+1);
        Point third_point = path.at(i+2);

        if ((second_point.x - first_point.x) != (third_point.x - second_point.x) || (second_point.y - first_point.y) != (third_point.y - second_point.y)) {
            path_point.push_back(second_point);
        }

    }

    return path_point;
}

class TrajectoryPlanner : public rclcpp::Node {
private :

    // Configuration
    bool isObstacles = false;
    std::vector<Obstacle *> obstacles = {new Obstacle(*new Point(120, -50), 25),
                                        new Obstacle(*new Point(-152, -6), 50),
                                        new Obstacle(*new Point(110, 130), 50),
                                        new Obstacle(*new Point(12, -102), 25),
                                        new Obstacle(*new Point(92, 170), 25),
                                        new Obstacle(*new Point(-92, 176), 30),
                                        new Obstacle(*new Point(-40, 220), 30),
                                        new Obstacle(*new Point(-44, -95), 30),
                                        new Obstacle(*new Point(-30, -150), 30)};
    Point initialPoint = null;
    bool isPursuit = true;

    // Infos Bateau
    Point start = null;
    Point initialVelocity = Point(0,0);
    Point goal = null;
    std::vector<Obstacle *> obstaclesBateau = {};

    //Topics
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr start_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr traj_publisher_;

    //Service 
    rclcpp::Service<geometry_msgs::srv::GetPlan>::SharedPtr start_service_;

    void startCallback(const geometry_msgs::msg::Point::SharedPtr msg) {

        Point tmpPoint = gps_to_Point(msg->x,msg->y);
        this.start.x = tmpPoint.x + this.initialPoint.x;
        this.start.y = tmpPoint.y + this.initialPoint.y;
        calculateAndPublishTrajectory();
    }

    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        Point tmpPoint = gps_to_Point(msg->x,msg->y);
        this.goal.x = tmpPoint.x + this.initialPoint.x;
        this.goal.y = tmpPoint.y + this.initialPoint.y;
    }

    void obstaclesCallback(const geometry_msgs::msg::std::vector<Point*> msg) {
        std::vector<Obstacle *> l_obstaclesBateau = {};
        for (auto position : msg){
            Point tmpPoint = gps_to_Point(*position.x, *position.y);
            l_obstaclesBateau.push_back(new Obstacle(*new Point(tmpPoint.x + this.initialPoint.x, tmpPoint.y + this.initialPoint.y), 20));
        }
        this.obstaclesBateau = l_obstaclesBateau;
    }

    void getStartPosition(const std::shared_ptr<geometry_msgs::srv::GetPlan::Request> request,
                          std::shared_ptr<geometry_msgs::srv::GetPlan::Response> response) {
        response->start.x = start.x;
        response->start.y = start.y;
    }

    void calculateAndPublishTrajectory() {

        if(!isPursuit){
            // Algo pour goal patrouille
        }

        std::vector<Point> path = planificationTrajectoire(this.start, this.initialVelocity, this.goal, this.obstacles, this.obstaclesBateau);
        std::vector<Point> min_path = minimizePath(path);

        // Publish the trajectory
        custom_msg::msg::Trajectory traj_msg;
        for (const auto &point : min_path) {
            traj_msg.x_points.push_back(point.x);
            traj_msg.y_points.push_back(point.y);
        }

        traj_publisher_->publish(traj_msg);
    }

public:
    TrajectoryPlanner()
        : Node("trajectory_planner_node") {
            this.start_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
                "/start", 10, std::bind(&TrajectoryPlanner::startCallback, this, std::placeholders::_1));

            this.goal_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
                "/goal", 10, std::bind(&TrajectoryPlanner::goalCallback, this, std::placeholders::_1));

            this.traj_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/traj", 10);

            this.start_service_ = this->create_service<geometry_msgs::srv::GetPlan>(
                "/get_start_position", std::bind(&TrajectoryPlanner::getStartPosition, this, std::placeholders::_1, std::placeholders::_2));
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}
