#include "ros/ros.h"
#include "ros/time.h"
// We use the already built messages types
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"

using namespace std;

class display_laser {

private:
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Publisher pub_scan_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    float angle[1000];
    geometry_msgs::Point current_scan[1000];

    bool new_laser;//to check if new data of laser is available or not

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

public:

display_laser() {

    pub_scan_marker = n.advertise<visualization_msgs::Marker>("display_laser", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    sub_scan = n.subscribe("scan", 1, &display_laser::scanCallback, this);

    new_laser = false;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    if ( new_laser ) {
        new_laser = false;
        display_text();

        //change color of marker
        put_color();

        //graphical display of the results
        populateMarkerTopic();
    }

}// update

void display_text() {

    // display data of laser
    for ( int loop=0; loop<nb_beams; loop++ )
        ROS_INFO("hit[%i]: (r = %f, theta = %f) -> (x = %f, y = %f)", loop, range[loop], angle[loop], current_scan[loop].x, current_scan[loop].y);

}//display_text

void put_color() {

    nb_pts = 0;//number of points to draw

    for ( int loop=0; loop<nb_beams; loop++ ) {

        // we add the current hit to the hits to display
        display[nb_pts].x = current_scan[loop].x;
        display[nb_pts].y = current_scan[loop].y;
        display[nb_pts].z = current_scan[loop].z;

        // we switch between white, yellow, red, green and blue for the graphical display of the hits of the laser
        if ( loop%5 == 0 ) {
            //white color
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;
        }
        else
            if ( loop%5 == 1 ) {
                //yellow color
                colors[nb_pts].r = 1;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
            }
            else
                if ( loop%5 == 2 ) {
                    //red color
                    colors[nb_pts].r = 1;
                    colors[nb_pts].g = 0;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;
                }
                else
                    if ( loop%5 == 3 ) {
                        //green color
                        colors[nb_pts].r = 0;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 0;
                        colors[nb_pts].a = 1.0;
                    }
                    else
                        if ( loop%5 == 4 ) {
                            //blue color
                            colors[nb_pts].r = 0;
                            colors[nb_pts].g = 0;
                            colors[nb_pts].b = 1;
                            colors[nb_pts].a = 1.0;
                        }
        nb_pts++;
    }

}//put_color

//CALLBACK
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    new_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    ROS_INFO("New data of laser received");
    ROS_INFO("range_min, range_max: %f, %f", range_min, range_max);
    ROS_INFO("angle_min: %f", angle_min*180/M_PI);
    ROS_INFO("angle_max: %f", angle_max*180/M_PI);
    ROS_INFO("angle_increment: %f", angle_inc*180/M_PI);
    ROS_INFO("number_of_beams: %d", nb_beams);

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        angle[loop] = beam_angle;
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            range[loop] = scan->ranges[loop];
        else
            range[loop] = range_max;

        //transform the scan in cartesian framework
        current_scan[loop].x = range[loop] * cos(beam_angle);
        current_scan[loop].y = range[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

//GRAPHICAL_DISPLAY
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
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

    pub_scan_marker.publish(references);

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

    //ROS_INFO("%i points to display", nb_pts);
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

    pub_scan_marker.publish(marker);
    populateMarkerReference();

}

};


int main(int argc, char **argv){

    ros::init(argc, argv, "display_laser");

    display_laser bsObject;

    ros::spin();

    return 0;
}


