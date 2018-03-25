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

/**
 * Constante représentant la distance maximal entre deux personnes d'un même groupe
 */
#define group_person_threshold 1
/**
 *
 */
#define dist_person_tracker 0.1
/**
 *
 */
#define active_threshold 10

using namespace std;

/**
 * Classe représentant le noeud pour la détection des groupes 
 */
class group_detection {
private:
    /**
     *
     */
    ros::NodeHandle n;
    /**
     *Receive the set of detected persons
     */
    ros::Subscriber sub_detect_person;
    /**
     * Receive robot moving
     */
    ros::Subscriber sub_robot_moving;
    /**
     *Receive robot position
     */
    ros::Subscriber sub_robot_position;
    /**
     *Publish goal_to_reach
     */
    ros::Publisher pub_group_detector;
    /**
     *Publish display
     */
    ros::Publisher pub_group_detector_marker;
    /**
     * Booléen indiquant si de nouvelles données ont été reçu
     */
    bool new_data;
    /**
     * Booléen indiquant si un nouveau but a été trouvé
     */
    bool new_goal;

    // GRAPHICAL DISPLAY
    /**
     * Nombre de point à afficher
     */
    int nb_pts;
    /**
     * Les point à afficher
     */
    geometry_msgs::Point display[2000];
    /**
     * La couleur des différents points
     */
    std_msgs::ColorRGBA colors[2000];
    
    /**
     *to store the goal to reach that we will be published
     */
     geometry_msgs::Point goal_to_reach;

    /**
     *to perform detection of moving legs and to store them
     */
    int nb_group_detected;
    /**
     * to store the middle of each group
     */
    geometry_msgs::Point group_detected[1000];
    /**
     *Person detected
     */ 
    geometry_msgs::Point person_detected[1000];
    /**
     * Personne active
     */
    geometry_msgs::Point active_person[1000];
    /**
     * Score des différentes personnes
     */
    int score[1000];
    /**
     *Nb person detected
     */
    int nb_person_detected;
    /**
     * Nombre de personnes actives
     */
    int nb_person_active;
    
    geometry_msgs::Point robot_position;
    /**
     * Booléen indiquatn que le robot s'est arrêté
     */
    bool new_robot;
    /**
     * Permet de savoir si le robot à changer de position
     */
    bool previous_robot_moving;
    bool current_robot_moving;
public:

    /**
     * \fn group_detection()
     * Constructeur de la classe group_detection
     */
    group_detection(){
        //Réception : Robot mouvant ou non
        sub_robot_moving = n.subscribe("robot_moving", 1, &group_detection::robot_movingCallback, this);
        //Réception du tableau de personne
        sub_detect_person = n.subscribe("moving_persons_detector_array", 1,&group_detection::perso_callback, this );
        sub_robot_position = n.subscribe("goal_reached", 1,&group_detection::position_callback, this );
        pub_group_detector_marker = n.advertise<visualization_msgs::Marker>("group_detector", 1);
        pub_group_detector = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);

        current_robot_moving = true;
        new_robot = false;
        new_goal = false;
        ros::Rate r(10);
        robot_position.x = 0;
        robot_position.y = 0;
        while (ros::ok()) {
            ros::spinOnce();
            update();
            r.sleep();
        }
    }

    /**
     * \fn perso_callback(const geometry_msgs::PoseArray::ConstPtr& array)
     *
     * Callback pour la réception des points des personnes
     *
     * \param array les personnes détectés
     */
    void perso_callback(const geometry_msgs::PoseArray::
                        ConstPtr& array){
        int len = array->poses.size();
     	geometry_msgs::Point tmp[len];

        //On récupert les points envoyés par moving_person_node
        for(int i=0; i<len; i++){
            tmp[i].x = array->poses[i].position.x;
            tmp[i].y = array->poses[i].position.y;
        }
        //On abaisse le score de tous les points
        for(int j=0; j<nb_person_detected; j++){
            score[j]--;
        }
        
        for(int i =0; i<len; i++){
            bool b = true;
            for(int j=0; j<nb_person_detected; j++){
                if(distancePoints(tmp[i], person_detected[j])>dist_person_tracker){
                    person_detected[j].x=tmp[i].x;
                    person_detected[j].y=tmp[i].y;
                    score[j]++;
                    b=false;
                    break;
                }
            }
            if(b){
                person_detected[nb_person_detected].x = tmp[i].x;
                person_detected[nb_person_detected].y = tmp[i].y;
                score[nb_person_detected]=1;
                nb_person_detected++;
            }
        }
        
        nb_person_active = 0;
        for(int i=0; i<nb_person_detected; i++) {
            if(score[i]>0){
                active_person[nb_person_active].x = person_detected[i].x;
                active_person[nb_person_active].y = person_detected[i].y;
                nb_person_active++;
            }
        }
        
        new_data = true;
        return;
        
    }

    /**
     * \fn position_callback(const geometry_msgs::Point::ConstPtr& g)
     *
     * \brief Callback pour la réception de la position courante du robot
     *
     * \param g la position du robot
     */
    void position_callback(const geometry_msgs::Point::ConstPtr& g){
        robot_position.x = g->x;
        robot_position.y = g->y;
    }

    /**
     * \fn geometry_msgs::Point closest_group()
     *
     * \brief renvoie le groupe le plus proche du robot
     * \return le groupe le plus proche
     */  
    geometry_msgs::Point closest_group() {
        int i_min = 0;
        float dist_min = 0, d;
        for(int i =0; i<nb_group_detected; i++){
            d = distancePoints(robot_position, group_detected[i]);
            if (d <= dist_min) {
                i_min = i;
                dist_min = d;
            }
        }
        return group_detected[i_min];
    }

    /**
     *\fn update ()
     *\brief met à jours le noeud
     */
    void update() {
        if ( new_data ) {
            new_data = false;
            nb_pts = 0;
            if ( !current_robot_moving) {
                ROS_INFO("robot is not moving");

                //we search for moving persons in 4 steps
                detect_group();

                //graphical display of the results
                populateMarkerTopic();

                if(new_goal) {
                    new_goal = false;
                    goal_to_reach.x = closest_group().x;
                    goal_to_reach.y = closest_group().y;
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

    /**
     * \fn void detect_group ()
     * \brief Recherche les différents groupes
     */
    void detect_group() {
        ROS_INFO("detecting group");
        
        int nb_group = 1;//to count the number of group

        int group_start[1000];
        int group_end[1000];
    
        //initialization of the first cluster
        group_start[0] = 0;
        group_end[0] = 0;
        int loop;
        ROS_INFO("nb person = %d", nb_person_active);
        for(loop = 1; loop < nb_person_active; loop++) {
            float d;
            d=distancePoints(active_person[loop-1],active_person[loop]);
            ROS_INFO("distance = %f", d);
            if(d <= group_person_threshold){
                group_end[nb_group]++;
            }else{
                nb_group++;
                group_start[nb_group] = loop;
                group_end[nb_group] = loop;
            }
        }      
        nb_group_detected = 0;
        for(int i = 0; i< nb_group+1; i++) {
            if(group_start[i] <= group_end[i]){
                float x1 = active_person[group_start[i]].x;
                float y1 = active_person[group_start[i]].y;
                float x2 = active_person[group_end[i]].x;
                float y2 = active_person[group_end[i]].y;
                geometry_msgs::Point m;
                group_detected[i].x = (float)(x2+x1)/2;
                group_detected[i].y = (float)(y2+y1)/2; 
            
                display[nb_pts].x = group_detected[nb_group_detected].x;
                display[nb_pts].y = group_detected[nb_group_detected].y;
                display[nb_pts].z = group_detected[nb_group_detected].z;

                nb_group_detected++;
            
                colors[nb_pts].r = 1;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;
                new_goal=true;
            }
        }
    
        ROS_INFO("%d group have been detected.\n", nb_group_detected);
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

    }

    /**
     * \fn void populateMarkerReference()
     *
     * \brief Draw the field of view and other references
     */ 
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

    /**
     * \fn void populateMarkerTopic()
     */
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
    ros::init(argc, argv, "group_detection");

    group_detection bsObject;

    ros::spin();


}
