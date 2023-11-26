#include <iostream>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "environment_interfaces/msg/boat_trajectory.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

class TrajectoryPlanner : public rclcpp::Node {
private :
    const int DELTA = 2;

    struct Point {
    int x, y;

    Point(int x, int y) : x(x), y(y) {}


    double distanceTo(const Point &other) {
        return std::hypot(double(x - other.x), double(y - other.y));
    }

    bool operator!=(const Point &other) const {
        return (x != other.x && y != other.y);
    }
};

    struct Obstacle {
        Point position;
        int diametre;

        Obstacle(const Point &pos, int diam) : position(pos), diametre(diam) {}
    };

    struct NodeTraj {
        Point position;
        double restant_score;
        Point velocity;

        NodeTraj(Point& pos, double f) : position(pos), restant_score(f), velocity(*new Point(0, 0)) {}

        bool operator>(const NodeTraj &other) const {
            return restant_score > other.restant_score;
        }
    };

    // Configuration
    bool isObstacles = true;
    std::vector<Obstacle *> obstacles = {new Obstacle(*new Point(120, -50), 25),
                                        new Obstacle(*new Point(-152, -6), 50),
                                        new Obstacle(*new Point(110, 130), 50),
                                        new Obstacle(*new Point(12, -102), 25),
                                        new Obstacle(*new Point(92, 170), 25),
                                        new Obstacle(*new Point(-92, 176), 30),
                                        new Obstacle(*new Point(-40, 220), 30),
                                        new Obstacle(*new Point(-44, -95), 30),
                                        new Obstacle(*new Point(-30, -150), 30)};
    bool isPursuit = true;
    bool isFirstTrajPat = true;

    // Infos Bateau
    Point start = Point(-1000,-1000);
    Point initialVelocity = Point(0,0);
    Point goal = Point(-1000,-1000);
    std::vector<Obstacle *> obstaclesBateau = {};

    //Topics
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr start_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_subscription_;
    rclcpp::Publisher<environment_interfaces::msg::BoatTrajectory>::SharedPtr traj_publisher_;

    void startCallback(geometry_msgs::msg::Point msg) {

        this->start.x = msg.x;
        this->start.y = msg.y;
        // calculateAndPublishTrajectory();
    }

    void goalCallback(geometry_msgs::msg::Point msg) {

        this->goal.x = msg.x;
        this->goal.y = msg.y;
    }

    // void obstaclesCallback(const std::vector<Point*> msg) {
    //     std::vector<Obstacle *> l_obstaclesBateau = {};
    //     for (auto position : msg){
    //         Point tmpPoint = gps_to_Point(*position.x, *position.y);
    //         l_obstaclesBateau.push_back(new Obstacle(*new Point(tmpPoint.x + this.initialPoint.x, tmpPoint.y + this.initialPoint.y), 20));
    //     }
    //     this.obstaclesBateau = l_obstaclesBateau;
    // }

    void calculateAndPublishTrajectory() {

        if(this->start != Point(-1000,-1000) && this->goal != Point(-1000,-1000)){
            if(!isPursuit){
                if(isFirstTrajPat){
                    this->goal = Point(200,-200);
                }else{
                    if(this->start.x > 150 && this->start.y < -150){
                        this->goal = Point(-200,-200);
                    }else if(this->start.x < -150 && this->start.y < -150){
                        this->goal = Point(-200,200);
                    }else if(this->start.x < -150 && this->start.y > 150){
                        this->goal = Point(0,0);
                    }else if(this->start.x > -100 && this->start.x < 100 && this->start.y > -100 && this->start.y < 100){
                        this->goal = Point(200,200);
                    }else if(this->start.x > 150 && this->start.y > 150){
                        this->goal = Point(200,-200);
                    } 
                }
            }

            std::vector<Point> path = planificationTrajectoire(this->start, this->initialVelocity, this->goal, this->obstacles, this->obstaclesBateau);
            std::vector<Point> min_path = minimizePath(path);

            // Publish the trajectory
            environment_interfaces::msg::BoatTrajectory traj_msg;
            for (int i = 0; i<4;i++) {
                traj_msg.x[i] = min_path.at(i).x;
                traj_msg.y[i] = min_path.at(i).y;
            }

            traj_publisher_->publish(traj_msg);
        }   
    }

public:
    TrajectoryPlanner()
        : Node("trajectory_planner_node") {

            this->start_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
                "/start", 10, std::bind(&TrajectoryPlanner::startCallback, this, std::placeholders::_1));

            this->goal_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
                "/goal", 10, std::bind(&TrajectoryPlanner::goalCallback, this, std::placeholders::_1));

            this->traj_publisher_ = this->create_publisher<environment_interfaces::msg::BoatTrajectory>("/traj", 10);
    }

    std::vector<Point> planificationTrajectoire(Point &start, Point &initial_velocity, const Point &goal,
                                            std::vector<Obstacle *> &obstacles, std::vector<Obstacle *> &obstaclesBateau) {
        std::vector<NodeTraj*> path;
        std::vector<Point> path_point;
        bool traj_found = false;
        int i = 0;

        NodeTraj* initial_node = new NodeTraj(start, start.distanceTo(goal));
        initial_node->velocity = initial_velocity;
        path.push_back(initial_node);

        while (!traj_found && i < 2000) {

            NodeTraj* current = path.back();

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
            path.push_back(new NodeTraj(best_neighbor, best_neighbor.distanceTo(goal)));
            i++;
        }
        return path_point;
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

    bool isInPath(const Point &neighbor, std::vector<NodeTraj *> path) {
        for (auto node : path) {
            if (neighbor.x == node->position.x && neighbor.y == node->position.y) {
                return true;
            }
        }
        return false;
    }

    std::vector<Point> minimizePath(std::vector<Point> &path) {
        std::vector<Point> path_point;

        for (int i = 0; i < (int)path.size()-3; i++ ) {
            Point first_point = path.at(i);
            Point second_point = path.at(i+1);
            Point third_point = path.at(i+2);

            if ((second_point.x - first_point.x) != (third_point.x - second_point.x) || (second_point.y - first_point.y) != (third_point.y - second_point.y)) {
                path_point.push_back(second_point);
            }

        }

        return path_point;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}

