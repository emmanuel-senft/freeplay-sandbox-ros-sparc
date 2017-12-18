#include <string>
#include <map>
#include <utility> // for std::pair
#include <iterator>
#include <algorithm>
#include <queue>

// opencv3
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>

#include <freeplay_sandbox_msgs/PlaygroundPlan.h>

using namespace std;
using namespace cv;

const double physicalMapWidth = 0.6; //m
const double physicalMapHeight = 0.335; //m

const double RESOLUTION=0.005; // m per cell

const double delayBeforeExecuting = 0.; // sec
const double t0 = 0.; //sec - extra time for the robot to get to the first point in traj
const double dt = 0.06; //sec - seconds between points in traj

typedef std::pair<int, int> Location;

bool endsWith(const std::string& fullString, const std::string& ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

class PlaygroundMapMaker {

public:

    PlaygroundMapMaker(std::shared_ptr<ros::NodeHandle> nh):
                _nh(nh),
                footprints_sub(_nh->subscribe("footprints", 10, &PlaygroundMapMaker::getFootprints, this)),
                path_pub(_nh->advertise<nav_msgs::Path>("playground_manipulation_path", 1)),
                occupancygrid_pub(_nh->advertise<nav_msgs::OccupancyGrid>("map", 1)),
                goal_sub(_nh->subscribe("goal", 10, &PlaygroundMapMaker::onGoal, this)),
                service(_nh->advertiseService("plan_motion", &PlaygroundMapMaker::planService, this))
    {

    }


    void getFootprints(const visualization_msgs::MarkerArray& markers)
    {

        for (const auto& marker : markers.markers) {
            vector<geometry_msgs::PointStamped> bb;

            for (const auto& point : marker.points) {
                geometry_msgs::PointStamped p;
                p.header.frame_id=marker.header.frame_id;
                p.point = point;

                bb.push_back(p);
            }
            footprints[marker.header.frame_id] = bb;

            cout << "Got footprint for " << marker.header.frame_id << ": ";
            for(const auto& p : bb) {
                cout << "(" << p.point.x << "," << p.point.y << "), ";
            }
            cout << endl;

        }

    }

    /** Plan a path from A to B using the A* algorithm. The resulting path is expressed in frame reference_frame.
     *
     * The starting point A is implicitely defined as the point (0,0,0) in the reference frame of the goal B:
     *  If you pass goal_pose = (1,2,3, /myframe), the implicit start point is (0,0,0, /myframe)
     *
     *  Note that currently, only the position of the goal_pose is used, not its orientation
     */
    nav_msgs::Path plan(const geometry_msgs::PoseStamped& goal_pose,
                        const std::string& reference_frame = "/sandtray")
    {

        // create implicit start point and transform it into reference_frame
        geometry_msgs::PointStamped start_point;
        start_point.header.frame_id = goal_pose.header.frame_id;
        geometry_msgs::PointStamped start_point_transformed;
        listener.transformPoint(reference_frame, start_point, start_point_transformed);

        // convert goal to a point and transform it into reference_frame
        geometry_msgs::PointStamped goal_point;
        goal_point.header = goal_pose.header;
        goal_point.point = goal_pose.pose.position;
        geometry_msgs::PointStamped goal_point_transformed;
        listener.transformPoint(reference_frame, goal_point, goal_point_transformed);

        ROS_INFO_STREAM("Planning from " << goal_pose.header.frame_id << " (" << 
                        start_point_transformed.point.x << ", " << 
                        start_point_transformed.point.y << ") to (" << 
                        goal_point_transformed.point.x << ", " << 
                        goal_point_transformed.point.y << ")");

        auto start = to_map(start_point_transformed.point.x, start_point_transformed.point.y);
        auto goal = to_map(goal_point_transformed.point.x, goal_point_transformed.point.y);


        nav_msgs::Path path;
        path.header.frame_id = reference_frame;
        path.header.stamp = ros::Time::now() + ros::Duration(delayBeforeExecuting);

        auto points = astar(start, goal);

        size_t i = 0;
        for (auto p : points) {
            double x, y;
            tie (x, y) = from_map(p);
            auto point = geometry_msgs::PoseStamped();
            point.pose.position.x = x;
            point.pose.position.y = y;
            point.header.frame_id = reference_frame;
            point.header.stamp = ros::Time(t0 + i * dt);

            path.poses.push_back(point);

            i++;

        }

        return path;
    }

    void updateOccupancyGrid(const std::string& inhibitedFrame = "", 
                             const std::string& reference_frame = "/sandtray")
    {

        auto _inhibitedFrame = inhibitedFrame;
        if (!inhibitedFrame.empty() && inhibitedFrame[0] != '/') _inhibitedFrame = '/' + inhibitedFrame;

        Mat occupancygrid((int)(physicalMapHeight / RESOLUTION),(int)(physicalMapWidth / RESOLUTION),  CV_8UC1, 255);

        map.header.frame_id = reference_frame;
        map.info.map_load_time = ros::Time::now();
        map.info.resolution = RESOLUTION;
        map.info.width = occupancygrid.size().width;
        map.info.height = occupancygrid.size().height;
        map.info.origin.position.y=-physicalMapHeight;


        for (auto const& kv : footprints) {

            if (kv.first == _inhibitedFrame) continue;

            vector<cv::Point> poly;

            for (const auto& p : kv.second) {
                geometry_msgs::PointStamped base_point;
                listener.transformPoint(reference_frame, p, base_point);
                poly.push_back(cv::Point(base_point.point.x / RESOLUTION, -base_point.point.y / RESOLUTION));
            }

            fillConvexPoly(occupancygrid,poly,0);
        }

        flip(occupancygrid, occupancygrid,0);
        blur(occupancygrid, occupancygrid,Size(3,3));

        vector<signed char> grid;
        for(size_t i = 0; i < occupancygrid.total(); ++i) {
            grid.push_back(100-occupancygrid.data[i]*100/255); // replace with a memcpy?
        }
        map.data = grid;
    }

    void onGoal(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {

        ROS_INFO("Received planning goal. Planning now.");

        // rebuild the map, without considering my start frame as an obstacle
        // (ie, inhibit goal->header.frame_id)
        updateOccupancyGrid(goal->header.frame_id);
        publishOccupancy();

        // plan and publish
        path_pub.publish(plan(*goal));

    }

    bool planService(freeplay_sandbox_msgs::PlaygroundPlan::Request& req,
                    freeplay_sandbox_msgs::PlaygroundPlan::Response& res)
    {
        ROS_INFO_STREAM("Got a planning request for " << req.goal.header.frame_id);
        ROS_INFO("Updating occupancy map...");
        updateOccupancyGrid(req.goal.header.frame_id);
        ROS_INFO("Planing...");
        auto p = plan(req.goal);
        res.path = p;
        path_pub.publish(p);
        ROS_INFO("Done!");
        return true;
    }

    void publishOccupancy()
    {
        occupancygrid_pub.publish(map);
    }

private:

    inline double heuristic(Location a, Location b) {
        int x1, y1, x2, y2;
        tie (x1, y1) = a;
        tie (x2, y2) = b;
        return abs(x1 - x2) + abs(y1 - y2);
    }

    inline int cost(Location a) {
        int x, y;
        tie (x, y) = a;

        const float COST_SCALE=1;
        int _cost = 50 * COST_SCALE;

        if (x>=0 && y>=0 && x<map.info.width && y<map.info.height) {
            auto raw = map.data[(map.info.height-y) * map.info.width + x];
            if (raw >= 0) { // raw = -1 -> occupancy unknown; let assign an average cost

                _cost = (raw + 1) * COST_SCALE; // raw + 1 -> 1 is the cost to move by one cell
            }
        }
        else {
            // --> if outside of the map, the cost grows linearly with the distance to the map.
            if (x < 0) _cost = cost({0, y}) + (-x * 10);
            if (y < 0) _cost = cost({x, 0}) + (-y * 10);
            if (x >= (int)map.info.width) _cost = cost({map.info.width - 1, y}) + ((x - map.info.width) * 10);
            if (y >= (int)map.info.height) _cost = cost({x, map.info.height - 1}) + ((y - map.info.height) * 10);
        }

        return _cost;
    }

    inline Location to_map(double x, double y) {

        return {(int)(x / RESOLUTION), (int)(-y / RESOLUTION)};
    }

    inline std::pair<double, double> from_map(Location a) {

        int x, y;
        tie (x, y) = a;

        return {x * RESOLUTION, -y * RESOLUTION};
    }

    // from http://www.redblobgames.com/pathfinding/a-star/implementation.html
    template<typename T, typename priority_t>
        struct PriorityQueue {
            typedef pair<priority_t, T> PQElement;
            priority_queue<PQElement, vector<PQElement>,
                std::greater<PQElement>> elements;

            inline bool empty() const { return elements.empty(); }

            inline void put(T item, priority_t priority) {
                elements.emplace(priority, item);
            }

            inline T get() {
                T best_item = elements.top().second;
                elements.pop();
                return best_item;
            }
        };

    std::vector<Location> neighbours(Location p){

        vector<Location> nghbs;

        nghbs.push_back({p.first-1, p.second-1});
        nghbs.push_back({p.first-1, p.second});
        nghbs.push_back({p.first-1, p.second+1});
        nghbs.push_back({p.first, p.second-1});
        nghbs.push_back({p.first, p.second+1});
        nghbs.push_back({p.first+1, p.second-1});
        nghbs.push_back({p.first+1, p.second});
        nghbs.push_back({p.first+1, p.second+1});

        return nghbs;

    }

    // based on http://www.redblobgames.com/pathfinding/a-star/implementation.html
    std::vector<Location> astar(Location start, Location goal) {

        vector<Location> path;

        std::map<Location,Location> came_from;
        std::map<Location,int> cost_so_far;

        PriorityQueue<Location, double> frontier;
        frontier.put(start, 0);

        came_from[start] = start;
        cost_so_far[start] = 0;


        //cout << "Cost map:" << endl;
        //for(int y=0;y<map.info.height;y++) {
        //    for(int x=0;x<map.info.width;x++) {
        //        cout << (int)(cost(map, {x,y}) / 100.);
        //    }
        //    cout << endl;
        //}
        cout << "Starting A*...";
        while (!frontier.empty()) {
            auto current = frontier.get();

            if (current == goal) {
                break;
            }

            for (auto next : neighbours(current)) {
                int new_cost = cost_so_far[current] + cost(next);
                if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    double priority = new_cost + heuristic(next, goal);
                    frontier.put(next, priority);
                    came_from[next] = current;
                }
            }
        }

        Location next = goal;
        while(true) {
            path.push_back(next);
            if (next == start) break;
            next = came_from[next];
        }
        std::reverse(std::begin(path), std::end(path));

        cout << "done." << endl;
        cout << "Publishing path (" << path.size() << " points)" << endl;
        return path;

    }

    nav_msgs::OccupancyGrid map;
    std::map<std::string, std::vector<geometry_msgs::PointStamped>> footprints;

    std::shared_ptr<ros::NodeHandle> _nh;

    ros::Subscriber footprints_sub;
    ros::Publisher occupancygrid_pub;
    ros::Publisher path_pub;
    ros::Subscriber goal_sub;

    ros::ServiceServer service;

    tf::TransformListener listener;
};

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "playground_map_and_plan");
    auto nh = make_shared<ros::NodeHandle>();
    ros::NodeHandle _private_node("~");

    PlaygroundMapMaker mapmaker(nh);

    ROS_INFO("playground_map_and_plan is ready. Waiting for footprints on /footprints + TF updates");

    ros::Rate loop_rate(5);
    while(nh->ok()){

        mapmaker.updateOccupancyGrid();
        mapmaker.publishOccupancy();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

