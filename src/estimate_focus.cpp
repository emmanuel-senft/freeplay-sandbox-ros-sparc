#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <freeplay_sparc/estimate_focusConfig.h> // generated header from cfg/estimate_focus.cfg

#include <freeplay_sandbox_msgs/AttentionTargetsStamped.h>
#include <freeplay_sandbox_msgs/AttentionTarget.h>

using namespace std;

static const string HUMAN_FRAME_PREFIX = "face_";

static const double FOV = 10. / 180 * M_PI; // radians
static const float RANGE = 3.; //m

// this value sets the level of attention (in [0, 1]) when a target is at the max distance
// of the gaze axis permitted by the field of view FOV (ie, at the limit of the field of view)
static const double ATTENTION_LEVEL_AT_FOV_LIMIT = 0.01;


inline double gaussian(double x, double mean, double variance) {
    return exp(-(x-mean)*(x-mean)/(2*variance));
}

// minimum time, in ms, that an object must remain in focus to be considered attended
std::chrono::milliseconds MIN_ATTENTIONAL_SPAN(1000);

template <class Rep, class Period>
constexpr std::chrono::milliseconds to_ms(const std::chrono::duration<Rep,Period>& d) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(d);
}

typedef std::vector<geometry_msgs::Point> Region;

std::map<std::string, std::vector<Region>> zones;

/**
 * Based on http://paulbourke.net/geometry/polygonmesh/
 */
bool isIn(tf::Vector3 p, Region region) {

    bool inside = false;
    double xinters;

    auto p1 = region[0];

    for (size_t i = 1; i <= region.size(); i++) {
        auto p2 = region[i % region.size()];

        if (p.y() > min(p1.y,p2.y)) {
            if (p.y() <= max(p1.y,p2.y)) {
                if (p.x() <= max(p1.x,p2.x)) {
                    if (p1.y != p2.y) {
                        xinters = (p.y()-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                        if (p1.x == p2.x || p.x() <= xinters) {
                            inside = !inside;
                        }
                    }
                }
            }
        }
        p1 = p2;
    }
    return inside;
}

void on_zones_received(const visualization_msgs::MarkerArrayConstPtr& markers) {

    ROS_INFO("Received playground zones");
    zones.clear();

    for (const auto& m : markers->markers) {
        zones[m.ns].push_back(m.points);
    }

}

map<std::string, std::chrono::system_clock::time_point> last_seen_frames;

void callback(freeplay_sandbox::estimate_focusConfig &config, uint32_t level) {
    cout << "Hit callback" << endl;
    ROS_INFO("Reconfiguring: attentional span is now %dms", config.attentional_span);
    MIN_ATTENTIONAL_SPAN = chrono::milliseconds(config.attentional_span);
}

visualization_msgs::Marker makeMarker(int id,
                                      const string& frame, 
                                      std_msgs::ColorRGBA color, 
                                      double dx=0, double dy=0, double dz=0,
                                      const string& ns = "focus_of_attention") {

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = ns;
    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = dx;
    marker.pose.position.y = dy;
    marker.pose.position.z = dz;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color = color;

    marker.lifetime = ros::Duration(0.5);

    return marker;
}




tf::Vector3 gazePlanIntersection(const tf::TransformListener& listener, 
                                 const std::string& reference_frame, 
                                 const std::string& gaze_frame) {

    tf::StampedTransform transform;

    // transform the candidate target to the observer's frame of reference
    try{
        listener.lookupTransform(reference_frame, gaze_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }


    // from now on, everything takes place in the reference frame (ie, the target plan)

    auto gaze_origin = transform.getOrigin();

    auto gaze_vector = transform * tf::Vector3(1,0,0) - gaze_origin;
    gaze_vector.normalize();

    tf::Vector3 plane_normal(0,0,1);
    plane_normal.normalize(); // trivial when the vector is 0,0,1
    double distance_plane_to_origin = 0.;

    auto t = - (gaze_origin.dot(plane_normal) + distance_plane_to_origin) / (gaze_vector.dot(plane_normal));

    auto gaze_projection = gaze_origin + gaze_vector * t;

    return gaze_projection;

}






double intensityOfVisualAttention(const tf::TransformListener& listener, 
                                  const string& target_frame, 
                                  const string& observer_frame) {

    if (target_frame == observer_frame) return 0;

    tf::StampedTransform transform;

    // transform the candidate target to the observer's frame of reference
    try{
        listener.lookupTransform(observer_frame, target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // object behind the observer?
    if (transform.getOrigin().x() < 0) return 0.;

    // the field of view's main axis is the observer's X axis. So, distance to main axis is
    // simply sqrt(y^2 + z^2)
    double distance_to_main_axis = sqrt(transform.getOrigin().y() * transform.getOrigin().y() + 
                                        transform.getOrigin().z() * transform.getOrigin().z());

    double fov_radius_at_x = tan(FOV/2) * transform.getOrigin().x();

    //ROS_INFO_STREAM(target_frame << ": distance_to_main_axis: " << distance_to_main_axis << ", fov_radius_at_x: " << fov_radius_at_x);
    
    // compute the variance of the gaussian distribution so that whenever 
    // distance_to_main_axis == fov_radius_at_x, the resulting attention level is
    // ATTENTION_LEVEL_AT_FOV_LIMIT
    double variance = - fov_radius_at_x / (2*log(ATTENTION_LEVEL_AT_FOV_LIMIT));

    return gaussian(distance_to_main_axis, 0, variance);

}

/** Returns a value between 0 (no attention) and 1 (maximal attention),
 * computed as a function of the duration a given target is being looked at.
 *
 * Currently, a linear function of time.
 */
double attentionIntensity(const chrono::milliseconds duration) {

    double intensity = (1. * (duration - MIN_ATTENTIONAL_SPAN).count()) / MIN_ATTENTIONAL_SPAN.count();

    return std::max(0.0, std::min(1.0, intensity));

}

int main( int argc, char** argv )
{

    ros::init(argc, argv, "estimate_focus");

    dynamic_reconfigure::Server<freeplay_sandbox::estimate_focusConfig> server;
    dynamic_reconfigure::Server<freeplay_sandbox::estimate_focusConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


    ros::NodeHandle n;
    ros::Rate r(30);

    auto zones_sub = n.subscribe("zones", 1, on_zones_received);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("estimate_focus", 1);
    ros::Publisher fov_pub = n.advertise<sensor_msgs::Range>("face_0_field_of_view", 1);
    ros::Publisher attentional_targets_pub = 
                n.advertise<freeplay_sandbox_msgs::AttentionTargetsStamped>("attention_targets", 1);

    tf::TransformListener listener;
    vector<string> frames;


    // Prepare a range sensor msg to represent the fields of view
    sensor_msgs::Range fov;

    fov.radiation_type = sensor_msgs::Range::INFRARED;
    fov.field_of_view = FOV;
    fov.min_range = 0;
    fov.max_range = 10;
    fov.range = RANGE;


    ROS_INFO("Waiting until a face becomes visible...");
    while (!listener.waitForTransform("sandtray", "face_0", ros::Time::now(), ros::Duration(.5))) {
        ROS_DEBUG("Still no face visible...");
        r.sleep();
    }

    ROS_INFO("Face detected! We can start estimating the focus of attention...");

    while (ros::ok())
    {

        while (   marker_pub.getNumSubscribers() < 1
               && fov_pub.getNumSubscribers() < 1
               && attentional_targets_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the markers or field of view or "
                          "attentional targets");
            ros::spinOnce();
            r.sleep();
        }


        frames.clear();
        listener.getFrameStrings(frames);

        int nb_faces = 0;

        for(auto frame : frames) {
            if(frame.find(HUMAN_FRAME_PREFIX) == 0) {

                nb_faces++;

                int face_idx = stoi(frame.substr(HUMAN_FRAME_PREFIX.length(), 1));

                if (face_idx == 0) {

                    freeplay_sandbox_msgs::AttentionTargetsStamped targets;
                    targets.header.frame_id = frame;
                    targets.header.stamp = ros::Time::now();

                    //////////////////////////////////////////
                    // gaze projection on sandtray
                    auto gaze_projection = gazePlanIntersection(listener, "sandtray", frame);

                    std_msgs::ColorRGBA col;
                    col.r = 1.; col.g = 0.; col.b = 0.; col.a = 1.;

                    marker_pub.publish(makeMarker(1000 + face_idx, 
                                                "sandtray", 
                                                col, 
                                                gaze_projection[0], gaze_projection[1],0,
                                                "gaze_projection"));

                    // Does the gaze falls on one existing playground zone?
                    // If yes, publish it as a new active attention target
                    ROS_INFO_STREAM("nb zones: " << zones.size());
                    for (const auto& kv : zones) {
                        bool isinzone = false;
                        for (const auto& region : kv.second) {

                            isinzone = isIn(gaze_projection, region);

                            if (isinzone) break;
                        }
                        if (isinzone) {

                            col.r = 0.; col.g = 0.; col.b = 1.; col.a = 1.;

                            marker_pub.publish(makeMarker(2000, 
                                                "sandtray", 
                                                col, 
                                                gaze_projection[0], gaze_projection[1],0,
                                                "active_zones"));

                            freeplay_sandbox_msgs::AttentionTarget target;
                            target.modality = freeplay_sandbox_msgs::AttentionTarget::VISUAL;
                            target.intensity = 1.0;
                            target.frame_id = kv.first;
                            targets.targets.push_back(target);
                        }
                    }
                    //////////////////////////////////////////

                    map<std::string, std::chrono::system_clock::time_point> seen_frames;

                    for(size_t i = 0 ; i < frames.size(); ++i) {
                        auto now = std::chrono::system_clock::now();
                        auto intensity_distance = intensityOfVisualAttention(listener, 
                                                                             frames[i],
                                                                             frame);
                        if(intensity_distance > 0.001) {

                            if (last_seen_frames.count(frames[i])) {
                                auto lasttime_seen = last_seen_frames[frames[i]];
                                seen_frames[frames[i]] = lasttime_seen;
                                auto duration = now - lasttime_seen;
                                if(duration > MIN_ATTENTIONAL_SPAN) {

                                    auto intensity_duration = attentionIntensity(to_ms(duration));
                                    auto intensity = intensity_distance * intensity_duration;

                                    ROS_DEBUG_STREAM(frames[i] << " is being attended to by " 
                                                             << frame << 
                                                             ": distance: " << intensity_distance 
                                                             << ", duration: " << intensity_duration
                                                             << ", final: " << intensity);

                                    col.r = 1.; col.g = 1.; col.b = 0.; col.a = intensity;
                                    marker_pub.publish(makeMarker(i, frames[i], col));

                                    freeplay_sandbox_msgs::AttentionTarget target;
                                    target.modality = freeplay_sandbox_msgs::AttentionTarget::VISUAL;
                                    target.intensity = intensity;
                                    target.frame_id = frames[i];
                                    targets.targets.push_back(target);
                                }
                            }
                            else {
                                seen_frames[frames[i]] = now;
                            }
                        }
                    }

                    last_seen_frames = seen_frames;

                    attentional_targets_pub.publish(targets);

                    if (fov_pub.getNumSubscribers() > 0) {
                        fov.range = RANGE;
                        fov.header.stamp = ros::Time::now();
                        fov.header.frame_id = frame;
                        fov_pub.publish(fov);
                    }
                }
                else {
                    ROS_WARN_THROTTLE(10, "Found a second human face. "
                                          "estimate_focus only processing face_0 for now");
                }
            }
        }
        if (nb_faces == 0) {

            // hide the field of view
            fov.range = 0;

            fov.header.stamp = ros::Time::now();
            fov.header.frame_id = "face_0";
            fov_pub.publish(fov); 

        }
        ros::spinOnce();
        r.sleep();
    }
}
