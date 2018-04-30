#include <iostream>
#include <string>
#include <csignal>
#include <sstream>
#include <cmath>
#include <climits>

# include <algorithm>
# include <vector>
# include <list>
# include <deque>

using namespace std;

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>


//Messages
#include <std_msgs/Int16.h>
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

#include "visualization_msgs/Marker.h"

#include <autonomous_nav/PotentialGrid.h>
#include <autonomous_nav/PotentialPlanner.h>
#include <autonomous_nav/CollisionChecker.h>


struct Pixel{
    int x, y;
    Pixel(int x_in, int y_in){
        x = x_in; y = y_in;
    }
};

class PotentialMapMaker{
public:
    PotentialMapMaker();
    //virtual ~PotentialMapMaker();

    void projectedMapCallback(const nav_msgs::OccupancyGrid& msg);
    void odometryCallback(const nav_msgs::Odometry& msg);
    bool planPathTo(autonomous_nav::PotentialPlanner::Request& request, 
                    autonomous_nav::PotentialPlanner::Response& response);
    bool checkIfCollisionFree(autonomous_nav::CollisionChecker::Request& request, 
                              autonomous_nav::CollisionChecker::Response& response);

private:
    boost::mutex mtx;

    ros::NodeHandle node_handle;

    //Robot odometry tracking
    ros::Subscriber robot_pos_sub;
    double robot_x, robot_y; 

    //Potential Maps
    ros::Subscriber projected_map_sub;
    ros::Publisher potential_map_pub;

    vector<vector<int>> projected_map;
    int robot_pixel_i, robot_pixel_j;
    autonomous_nav::PotentialGrid last_potential_map;

    //Planning
    ros::ServiceServer planning_srv;
    ros::Publisher online_traj_pub;

    //Collision checking
    ros::ServiceServer collision_srv;
    
    //Auxiliar functions
    bool isWalkable(struct Pixel start, struct Pixel end);
    void publishRvizPath(const autonomous_nav::PotentialPlanner::Response& response);
};

PotentialMapMaker::PotentialMapMaker(){
    //Initalize some variables
    robot_x = 0; robot_y = 0;
    last_potential_map = autonomous_nav::PotentialGrid();

    //Node I/O
    robot_pos_sub = node_handle.subscribe("/odom", 10, &PotentialMapMaker::odometryCallback, this);
    
    //Potential Map Making
    projected_map_sub = node_handle.subscribe("/projected_map", 1, &PotentialMapMaker::projectedMapCallback, this);
    potential_map_pub = node_handle.advertise<autonomous_nav::PotentialGrid>("potential_map", 1);

    //Planning
    planning_srv = node_handle.advertiseService("/autonomous_nav/PotentialPlanner", &PotentialMapMaker::planPathTo, this);
    online_traj_pub = node_handle.advertise<visualization_msgs::Marker> ("/autonomous_nav/solution_path", 3, true);

    //Collision checking
    collision_srv = node_handle.advertiseService("/autonomous_nav/CollisionChecker", &PotentialMapMaker::checkIfCollisionFree, this);
}

void PotentialMapMaker::odometryCallback(const nav_msgs::Odometry& msg){
    mtx.lock();
    
    robot_x = msg.pose.pose.position.x;
    robot_y = msg.pose.pose.position.y;

    mtx.unlock();
}

//#####################################################################
// POTENTIAL MAP CREATION

void PotentialMapMaker::projectedMapCallback(const nav_msgs::OccupancyGrid& msg){
    mtx.lock();

    //Resize the projected_map variable for new size
    projected_map.resize(msg.info.width);
    for(int i = 0; i < projected_map.size(); i++)
        projected_map[i].resize(msg.info.height, 0);
    

    //Copy map into 2D array and initialize inflation queue
    //IN: UNKNOWN -1, FREE 0, OCCUPIED 100
    //OUT: UNKNOWN 1, FREE 0, OCCUPIED 2
    deque<Pixel> inflation_queue;
    for(int i = 0; i < projected_map.size(); i++){ //msg.info.width -> columns
        for(int j = 0; j < projected_map[0].size(); j++){//msg.info.height -> rows

            //Obstacle
            if (msg.data[projected_map.size()*j + i] == 100){ 
                projected_map[i][j] = 2;
                //if obstacle add to queue for inflation
                inflation_queue.push_back({i, j});

            //Unknown    
            }else if(msg.data[i +  projected_map.size()*j] == -1){ 
                projected_map[i][j] = 1;

            //Free    
            }else{ 
                projected_map[i][j] = 0;
            }
        }
    }

    //1. INFLATE OBSTACLES
    int inflate = 3;
    deque<Pixel> inflation_queue2;

    for(int n = 0; n < inflate; n++){
        inflation_queue2 = deque<Pixel>();

        while(!inflation_queue.empty()){
            for(int i = std::max(0, inflation_queue.front().x-1); i < std::min(inflation_queue.front().x+2, (int)msg.info.width); i++){
                for(int j = std::max(0, inflation_queue.front().y-1); j < std::min(inflation_queue.front().y+2, (int)msg.info.height); j++){
                    if(projected_map[i][j] <= 1){
                        projected_map[i][j] = 2;
                        inflation_queue2.push_back(Pixel(i, j));
                    }
                }
            }

            inflation_queue.pop_front();
        }

        inflation_queue = inflation_queue2;
    }

    //2. COMPUTE POTENTIAL
    //Put 3 in robot position (have to convert to pixel)
    robot_pixel_i = round((robot_x - msg.info.origin.position.x)/msg.info.resolution);
    robot_pixel_j = round((robot_y - msg.info.origin.position.y)/msg.info.resolution);

    if(robot_pixel_i < 0 || robot_pixel_i > msg.info.width ||
        robot_pixel_j < 0 || robot_pixel_j > msg.info.height){
        ROS_WARN("Robot outside of projected_map");
        mtx.unlock();
        return;
    }

    projected_map[robot_pixel_i][robot_pixel_j] = 3;

    //Initialize queue with robot position
    deque<Pixel> queue;
    queue.push_back(Pixel(robot_pixel_i, robot_pixel_j));

    while(!queue.empty()){
        for(int i = std::max(0, queue.front().x-1); i < std::min(queue.front().x+2, (int)msg.info.width); i++){
            for(int j = std::max(0, queue.front().y-1); j < std::min(queue.front().y+2, (int)msg.info.height); j++){
                //Free
                if(projected_map[i][j] == 0){
                    projected_map[i][j] = projected_map[queue.front().x][queue.front().y] + 1;
                    queue.push_back(Pixel(i,j));
                }
            }
        }

        queue.pop_front();
    }


    //Create message for potential map based on original occupancy grid msg
    last_potential_map = autonomous_nav::PotentialGrid();
    last_potential_map.header = msg.header;
    last_potential_map.info = msg.info;

    for(int j = 0; j < projected_map[0].size(); j++)
        for(int i = 0; i < projected_map.size(); i++)
            last_potential_map.data.push_back( projected_map[i][j]);

    potential_map_pub.publish(last_potential_map);  
    
    mtx.unlock();
}

//#####################################################################
// PLANNING 

bool PotentialMapMaker::planPathTo(autonomous_nav::PotentialPlanner::Request& request, 
                                   autonomous_nav::PotentialPlanner::Response& response)
{
    mtx.lock();


    float goal_x = request.goal_state_x,
          goal_y = request.goal_state_y,
          map_origin_x = last_potential_map.info.origin.position.x,
          map_origin_y = last_potential_map.info.origin.position.y,
          map_resolution = last_potential_map.info.resolution;

    int goal_px_x = round((goal_x - map_origin_x)/map_resolution),
        goal_px_y = round((goal_y - map_origin_y)/map_resolution);

    if(projected_map[goal_px_x][goal_px_y] == 2){
        ROS_ERROR("Goal is marked as occupied.");
        mtx.unlock();
        return false;
    }

    struct Pixel robot_cell = Pixel(robot_pixel_i, robot_pixel_j);
    struct Pixel current_cell = Pixel(goal_px_x, goal_px_y);
    vector<Pixel> pixel_path = vector<Pixel>();

    int watchdog_count = 0;
    int watchdog_max = 1500;

    while(current_cell.x != robot_cell.x || current_cell.y != robot_cell.y){
        //Add current_cell to path
        pixel_path.push_back(current_cell);

        //Init neighbourhood search
        float min_potential = (float)INT_MAX;
        float compare_potential = (float)INT_MAX; //Used for weighting the diagonals
        struct Pixel min_pixel = Pixel(-1, -1);

        //Prepare for loop limits
        int i_start = std::max(0, current_cell.x-1);
        int i_end = std::min(current_cell.x+2, (int)last_potential_map.info.width);

        int j_start = std::max(0, current_cell.y-1);
        int j_end = std::min(current_cell.y+2, (int)last_potential_map.info.height);

        for(int i = i_start; i < i_end; i++){
            for(int j = j_start; j < j_end; j++){
                //Can be unknown or obstacle (1,2) or potential >= 3
                if(projected_map[i][j] < 3) continue;

                compare_potential = projected_map[i][j];
                //Diagonal check (increase potential to make them less attractive)
                if( (abs(i-current_cell.x) + abs(j-current_cell.y)) == 2 )
                    compare_potential += 0.5;
                
                //Comparison
                if(compare_potential <= min_potential){
                    min_pixel.x = i; 
                    min_pixel.y = j;
                    min_potential = compare_potential;
                }
            }
        }
        //Update cell to add to path
        current_cell = min_pixel;

        //Watchdog anti-blocker
        if(watchdog_count++ > watchdog_max){
            ROS_WARN("No path found, max iter reached");
            response.poses.clear(); mtx.unlock(); return false;
        }
    }


    //GREEDY smoothing
    vector<Pixel> pixel_path_smooth = vector<Pixel>();
    pixel_path_smooth.push_back(pixel_path.back());

    int segment_end_idx = pixel_path.size()-1;
    while(segment_end_idx != 0){
        for(int i = 0; i < segment_end_idx; i++){
            if(isWalkable(pixel_path.at(i), 
                          pixel_path.at(segment_end_idx)))
            {   
                //Add new point to smooth path
                pixel_path_smooth.push_back(pixel_path.at(i));

                //Reset search for segment until this one
                segment_end_idx = i;

                //Get out of loop
                break;
            }
        }
    }
    // Reverse path to get start to goal path
    std::reverse(pixel_path_smooth.begin(), pixel_path_smooth.end());

    //Convert from pixel path (i,j) to pose path (x,y) for driver
    geometry_msgs::Pose2D pose;
    for(int i = 0; i < pixel_path_smooth.size(); i ++){
        pose = geometry_msgs::Pose2D();
        pose.x = pixel_path_smooth.at(i).x * map_resolution + map_origin_x;
        pose.y = pixel_path_smooth.at(i).y * map_resolution + map_origin_y;
        response.poses.insert(response.poses.begin(), pose);
    }

    publishRvizPath(response);

    mtx.unlock();
}

//############################################
// Collision checking

bool PotentialMapMaker::checkIfCollisionFree(autonomous_nav::CollisionChecker::Request& request, 
                                             autonomous_nav::CollisionChecker::Response& response)
{
    mtx.lock();

    float map_origin_x = last_potential_map.info.origin.position.x,
          map_origin_y = last_potential_map.info.origin.position.y,
          map_resolution = last_potential_map.info.resolution;

    response.path_safe = true;

    struct Pixel  start_px(-1, -1), end_px(-1,-1);
    for(int i = 0; i < request.poses.size()-1; i++){
        //Check each point in the way
        start_px.x = round((request.poses.at(i).x - map_origin_x)/map_resolution);
        start_px.y = round((request.poses.at(i).y - map_origin_y)/map_resolution);

        end_px.x = round((request.poses.at(i+1).x - map_origin_x)/map_resolution);
        end_px.y = round((request.poses.at(i+1).y - map_origin_y)/map_resolution);

        if(!isWalkable(start_px, end_px)) {
            response.path_safe = false; 
            break;
        }
    }

    mtx.unlock();
    return true;
}

void PotentialMapMaker::publishRvizPath(const autonomous_nav::PotentialPlanner::Response& response){
    //Send path to rviz
    visualization_msgs::Marker m = visualization_msgs::Marker();

    m.header.frame_id = "/odom";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.scale.x = 0.02;

    m.color.g = m.color.a = 1.0;  
    m.ns = "potential_path";  

    geometry_msgs::Point p;
    for(int i = 1; i < response.poses.size(); i++){
        //Build pose for marker display
        p = geometry_msgs::Point();
        p.x = response.poses[i].x;
        p.y = response.poses[i].y;
        p.z = 0.2;
        m.points.push_back(p);

        p = geometry_msgs::Point();
        p.x = response.poses[i-1].x;
        p.y = response.poses[i-1].y;
        p.z = 0.2;
        m.points.push_back(p);
    }

    online_traj_pub.publish(m);
}


// #####################################

bool PotentialMapMaker::isWalkable(struct Pixel start, struct Pixel end){
    //Checks collision-free straight line from start to end

    //Get vector between start and end
    float path_x = end.x - start.x;
    float path_y = end.y - start.y;
    float path_length = std::sqrt(path_x*path_x + path_y*path_y);

    //Prepare step increase
    float step_length = 0.2;
    float delta_x = step_length*(path_x/path_length); 
    float delta_y = step_length*(path_y/path_length);

    //Prepare checking loop
    float check_point_x = start.x;
    float check_point_y = start.y;
    for(float i = 0; i < path_length; i += step_length){
        //Obstacle == 2 in projected_map; <= 2 for obstacle + unknown space
        if(projected_map.at(std::round(check_point_x)).at(std::round(check_point_y)) == 2) 
            return false;

        check_point_x += delta_x;
        check_point_y += delta_y;
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_map");
    
    PotentialMapMaker pmm;

    ros::spin();

    return (0);
}