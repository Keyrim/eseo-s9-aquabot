#include <iostream>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "environment_interfaces/msg/boat_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/u_int32.hpp"

enum Phase {
    BUOY,
    PATROL,
    PURSUIT,
};

class TrajectoryPlanner : public rclcpp::Node {
private :
    const int DELTA = 2;

    double toRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    struct Point {
    double x, y;

    Point(double x, double y) : x(x), y(y) {}


    double distanceTo(const Point &other) {
        return std::hypot(double(x - other.x), double(y - other.y));
    }

    bool operator!=(const Point &other) const {
        return (x != other.x && y != other.y);
    }
};

    struct Obstacle {
        Point position;
        int diameter;

        Obstacle(const Point &pos, int diam) : position(pos), diameter(diam) {}
    };

    struct NodeTraj {
        Point position;
        double remainingScore;
        Point velocity;

        NodeTraj(Point& pos, double f) : position(pos), remainingScore(f), velocity(*new Point(0, 0)) {}

        bool operator>(const NodeTraj &other) const {
            return remainingScore > other.remainingScore;
        }
    };

    // Configuration
    Phase currentPhase = BUOY;
    bool isFirstTrajPat = true;

    // Infos Bateau
    double baseLat = 48.046300000000;
    double baseLon = -4.976320000000; 
    Point start = Point(-1000,-1000);
    Point initialVelocity = Point(0,0);

    // Infos Target
    Point target = Point(-1000,-1000);

    // Infos Obstacles
    std::vector<Obstacle *> fixedObstacles = {new Obstacle(*new Point(120, -50), 25),
                                        new Obstacle(*new Point(-152, -6), 50),
                                        new Obstacle(*new Point(110, 130), 50),
                                        new Obstacle(*new Point(12, -102), 25),
                                        new Obstacle(*new Point(92, 170), 25),
                                        new Obstacle(*new Point(-92, 176), 30),
                                        new Obstacle(*new Point(-40, 220), 30),
                                        new Obstacle(*new Point(-44, -95), 30),
                                        new Obstacle(*new Point(-30, -150), 30)};
    std::vector<Obstacle *> alliesBoat = {};

    //Topics
    rclcpp::Subscription<environment_interfaces::msg::BoatState>::SharedPtr startSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr buoySubscription;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr threatSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr helpThreatSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr alliesSubscription;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr currentPhaseSubscription;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr trajPublisher;

    void startCallback(environment_interfaces::msg::BoatState ourBoat) {
        this->start.x = ourBoat.x;
        this->start.y = ourBoat.y;
        this->initialVelocity.x = ourBoat.vx;
        this->initialVelocity.y = ourBoat.vy;
        calculateAndPublishTrajectory();
    }

    void buoyCallback(geometry_msgs::msg::Point buoy) {

        if (this->currentPhase == BUOY){
            this->target.x = buoy.x;
            this->target.y = buoy.y;
        } 
    }

    void threatCallback(geometry_msgs::msg::Point threat) {

        if (this->currentPhase != BUOY){
            this->target.x = threat.x;
            this->target.y = threat.y;
        }
    }

    void currentPhaseCallback(std_msgs::msg::UInt32 v_currentPhase) {
        if(v_currentPhase.data == static_cast<uint32_t>(1)){
            this->currentPhase = BUOY;
        }else if(v_currentPhase.data == static_cast<uint32_t>(2)){
            this->currentPhase = PATROL;
        }else if(v_currentPhase.data == static_cast<uint32_t>(3)){
            this->currentPhase = PURSUIT;
        } 
    }

    void alliesCallback(geometry_msgs::msg::PoseArray allies) {

        std::vector<Obstacle *> l_alliesBoat = {};
        
        for (int i = 0 ; i < (int)allies.poses.size() ; i++ ){

            geometry_msgs::msg::Pose pose = allies.poses[i];
            Point tmpPoint = convert_gps_to_xy(pose.position.x, pose.position.y);
            l_alliesBoat.push_back(new Obstacle(tmpPoint, 20));
        }
        this->alliesBoat = l_alliesBoat;
    }

    void calculateAndPublishTrajectory() {

        if(this->start != Point(-1000,-1000) && this->target != Point(-1000,-1000)){

            if(this->currentPhase == PATROL){
                if(isFirstTrajPat){
                    this->target = Point(200,-200);
                }else{
                    if(this->start.x > 150 && this->start.y < -150){
                        this->target = Point(-200,-200);
                    }else if(this->start.x < -150 && this->start.y < -150){
                        this->target = Point(-200,200);
                    }else if(this->start.x < -150 && this->start.y > 150){
                        this->target = Point(0,0);
                    }else if(this->start.x > -100 && this->start.x < 100 && this->start.y > -100 && this->start.y < 100){
                        this->target = Point(200,200);
                    }else if(this->start.x > 150 && this->start.y > 150){
                        this->target = Point(200,-200);
                    } 
                }
            }

            std::vector<Point> path = planificationTrajectoire(this->start, this->initialVelocity, this->target, this->fixedObstacles, this->alliesBoat);
            std::vector<Point> minPath = minimizePath(path);

            // Publish the trajectory
            if (minPath.size() > 0){

                geometry_msgs::msg::Point trajMsg;
                trajMsg.x = minPath.at(0).x;
                trajMsg.y = minPath.at(0).y;

                trajPublisher->publish(trajMsg);
            } 
        }   
    }

public:
    TrajectoryPlanner()
        : Node("trajectory_planner_node") {

            this->startSubscription = this->create_subscription<environment_interfaces::msg::BoatState>(
                "/boat/estimator/position", 10, std::bind(&TrajectoryPlanner::startCallback, this, std::placeholders::_1));

            this->buoySubscription = this->create_subscription<geometry_msgs::msg::Point>(
                "/buoy", 10, std::bind(&TrajectoryPlanner::buoyCallback, this, std::placeholders::_1));

            this->threatSubscription = this->create_subscription<geometry_msgs::msg::Point>(
                "/threat", 10, std::bind(&TrajectoryPlanner::threatCallback, this, std::placeholders::_1));

            this->helpThreatSubscription = this->create_subscription<geometry_msgs::msg::Point>(
                "/wamv/ais_sensor/ennemy_position ", 0.1, std::bind(&TrajectoryPlanner::threatCallback, this, std::placeholders::_1));

            this->currentPhaseSubscription = this->create_subscription<std_msgs::msg::UInt32>(
                "/vrx/patrolandfollow/current_phase", 10, std::bind(&TrajectoryPlanner::currentPhaseCallback, this, std::placeholders::_1));

            this->alliesSubscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/wamv/ais_sensor/allies_positions", 1, std::bind(&TrajectoryPlanner::alliesCallback, this, std::placeholders::_1));

            this->trajPublisher = this->create_publisher<geometry_msgs::msg::Point>("/boat/controller/traj", 10); 
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

        while (!traj_found && i < 1000) {

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
            if(best_neighbor != Point(-1000,1000)){
                path.push_back(new NodeTraj(best_neighbor, best_neighbor.distanceTo(goal)));
            } 
            i++;
        }
        while (!path.empty()) {
            path_point.push_back(path.back()->position);
            path.pop_back();
        }
        std::reverse(path_point.begin(), path_point.end());
        return path_point;
    }

    bool isValidPoint(Point &point, const std::vector<Obstacle *> &obstacles) {
        for (auto obs : obstacles) {
            if (point.x >= (obs->position.x - obs->diameter) 
                && point.x <= (obs->position.x + obs->diameter) 
                && point.y >= (obs->position.y - obs->diameter) 
                && point.y <= (obs->position.y + obs->diameter)) {
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

    Point convert_gps_to_xy(double latitude, double longitude) {
        // Haversine distance calculation
        double R = 6371000;  // Earth radius in meters

        double dLat = toRadians(latitude - baseLat);
        double dLon = toRadians(longitude - baseLon);
        double a = sin(dLat / 2) * sin(dLat / 2) + cos(toRadians(baseLat)) * cos(toRadians(latitude)) * sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double distance = R * c;

        // Calculate x and y based on the distance and bearing
        double y = sin(dLon) * cos(toRadians(latitude));
        double x = cos(toRadians(baseLat)) * sin(toRadians(latitude)) - sin(toRadians(baseLat)) * cos(toRadians(latitude)) * cos(dLon);
        double bearing = atan2(y, x);
        y = distance * cos(bearing);
        x = distance * sin(bearing);
        return *new Point(x, y);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}

