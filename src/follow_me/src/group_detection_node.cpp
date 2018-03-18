// Signal handling
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#define COUNTOF(x) (sizeof(x)/sizeof(*x))
#define group_person_threshold 3

using namespace std;

class group_detection {
private:
    
    ros::NodeHandle n;
    //Receive the set of detected persons
    ros::Subscriber sub_detect_person;
    //Receive robot moving
    ros::Subscriber sub_robot_moving;
    //Publish goal_to_reach
    ros::Publisher pub_group_detector;
    //Publish display
    ros::Publisher pub_group_detector_marker;
    bool new_data;
    bool new_goal;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];
    
    //to store the goal to reach that we will be published
    geometry_msgs::Point goal_to_reach;

    //to perform detection of moving legs and to store them
    int nb_group_detected;
    // to store the middle of each group
    geometry_msgs::Point group_detected[1000];
    //Person detected 
    geometry_msgs::Point person_detected[1000];
    //Nb person detected
    int nb_person_detected;
    //to check if the robot is moving or not
    bool new_robot;
    bool previous_robot_moving;
    bool current_robot_moving;
public:

    group_detection(){
        //Réception : Robot mouvant ou non
        sub_robot_moving = n.subscribe("robot_moving", 1, &group_detection::robot_movingCallback, this);
        //Réception du tableau de personne
        sub_detect_person = n.subscribe("moving_persons_detector_array", 1,&group_detection::perso_callback, this );
        pub_group_detector_marker = n.advertise<visualization_msgs::Marker>("group_detector", 1);
        pub_group_detector = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);

        current_robot_moving = true;
        new_robot = false;
        new_goal = false;
        ros::Rate r(10);
        while (ros::ok()) {
            ros::spinOnce();
            update();
            r.sleep();
        }
    }
    void perso_callback(const geometry_msgs::PoseArray::
                        ConstPtr& array){
        
        nb_person_detected = COUNTOF(array);
        
        for(int i=0; i<nb_person_detected; i++){
            person_detected[i].x = array->poses[i].position.x;
            person_detected[i].y = array->poses[i].position.y;
        }
        new_data = true;
        return;
        
    }
    void position_callback(const geometry_msgs::Point::ConstPtr& g);
    void update() {
        if ( new_data ) {
            new_data = false;
            nb_pts = 0;
            if ( !current_robot_moving ) {
                ROS_INFO("robot is not moving");
                //we search for moving persons in 4 steps
                detect_group();
                //to publish the goal_to_reach
                if(new_goal) {
                    new_goal = false;
                    pub_group_detector.publish(goal_to_reach);
                }
            }
            else
                ROS_INFO("robot is moving");
            ROS_INFO("\n");
        }
        else
            ROS_INFO("wait for data");
    }
    
    void detect_group() {
        ROS_INFO("detecting group");
        
        int nb_group = 0;//to count the number of group

        int group_start[1000];
        int group_end[1000];
    
        //initialization of the first cluster
        group_start[0] = 0;
        group_end[0] = 0;
        int loop;
        for(loop = 1; loop < nb_person_detected; loop++) {
            float d;
            d=distancePoints(person_detected[group_start[nb_group]],person_detected[loop]);
            if(d <= group_person_threshold){
                group_end[nb_group]++;
            }else{
                nb_group++;
                group_start[nb_group] = loop;
                group_end[nb_group] = loop;
            }
        }      
        nb_group_detected = 0;
        for(int i = 0; i< nb_group; i++) {
            if(group_start[i] < group_end[i]){
                float x1 = person_detected[group_start[i]].x;
                float y1 = person_detected[group_start[i]].y;
                float x2 = person_detected[group_end[i]].x;
                float y2 = person_detected[group_end[i]].y;
                geometry_msgs::Point m;
                group_detected[i].x = (float)(x2+x1)/2;
                group_detected[i].y = (float)(y2+y1)/2; 
            
                display[nb_pts].x = group_detected[nb_group_detected].x;
                display[nb_pts].y = group_detected[nb_group_detected].y;
                display[nb_pts].z = group_detected[nb_group_detected].z;

                nb_group_detected++;
            
                colors[nb_pts].r = 0;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;
            }
        }
    
        if ( nb_group_detected )
            ROS_INFO("%d group have been detected.\n", nb_group_detected);
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

        return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

    }

    // Draw the field of view and other references
    void populateMarkerReference() {

        visualization_msgs::Marker references;

        references.header.frame_id = "laser";
        references.header.stamp = ros::Time::now();
        references.ns = "example";
        references.id = 1;
        references.type = visualization_msgs::Marker::LINE_STRIP;
        references.action = visualization_msgs::Marker::ADD;
        references.pose.orientation.w = 1;

        references.scale.x = 0.02;

        references.color.r = 1.0f;
        references.color.g = 1.0f;
        references.color.b = 1.0f;
        references.color.a = 1.0;
        geometry_msgs::Point v;

        v.x =  0.02 * cos(-2.356194);
        v.y =  0.02 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        v.x =  5.6 * cos(-2.356194);
        v.y =  5.6 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        float beam_angle = -2.356194 + 0.006136;
        // first and last beam are already included
        for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
            v.x =  5.6 * cos(beam_angle);
            v.y =  5.6 * sin(beam_angle);
            v.z = 0.0;
            references.points.push_back(v);
        }

        v.x =  5.6 * cos(2.092350);
        v.y =  5.6 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        v.x =  0.02 * cos(2.092350);
        v.y =  0.02 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        pub_group_detector_marker.publish(references);

    }

    void populateMarkerTopic(){

        visualization_msgs::Marker marker;

        marker.header.frame_id = "laser";
        marker.header.stamp = ros::Time::now();
        marker.ns = "example";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;

        marker.color.a = 1.0;

        ROS_INFO("%i points to display", nb_pts);
        for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

        pub_group_detector_marker.publish(marker);
        populateMarkerReference();

    }

    void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

        new_robot = true;
        ROS_INFO("New data of robot_moving received");
        previous_robot_moving = current_robot_moving;
        current_robot_moving = state->data;

    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "moving_persons_detector");

    group_detection bsObject;

    ros::spin();


}
