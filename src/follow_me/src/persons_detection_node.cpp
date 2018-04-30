/**
 * \file persons_detection_node.cpp
 * \brief Neud pour la détection de personnes
 * \Authors Groupe 7
 */
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"

//used for clustering
/**
 * Distance entre deux points pour qu'ils appartiennent au même cluster
 */
#define cluster_threshold 0.2 //threshold for clustering
/**
 * 
 */
#define detection_threshold 0.2 //threshold for motion detection
/**
 * Taux de points dynamique
 */
#define dynamic_threshold 0 //to decide if a cluster is static or dynamic

//Distance entre deux personnes d'un même groupe
#define group_person_threshold 0.8

//used for detection of moving legs
/**
 * Largeur minimale d'une jambe
 */
#define leg_size_min 0.1
/**
 * Largeur maximale d'une jambe
 */
#define leg_size_max 0.3

/**
 * Distance maximale entre deux jambes d'une personne
 */
#define legs_distance_max 1
/**
 * Distance maximale entre deux personnes d'un même groupe
 */
#define distance_person 1.5
/**
 * Nombre d'iteration pour la détection
 */
#define iter_person 10
/**
 * Nombre de vue minimum pour la détection
 */
#define min_view 7
using namespace std;

/**
 * \class moving_persons_detector
 * \brief Détecte les personnes et les groupes
 */
class moving_persons_detector {

private:
    /**
     * Handle du noeud
     */
    ros::NodeHandle n;
    /**
     * Abonnement pour le scan du laser
     */
    ros::Subscriber sub_scan;
    /**
     * Abonnement pour l'état du robot
     */
    ros::Subscriber sub_robot_moving;
    /**
     * Abonnement pour la fin du robot
     */
    ros::Subscriber sub_finish_move;
    /**
     * Topic pour donner les personnes détectées
     */
    ros::Publisher pub_moving_persons_detector;
    /**
     * Topic pour le marker
     */
    ros::Publisher pub_moving_persons_detector_marker;

    // to store, process and display laserdata
    /**
     * Nombre de points récupérés
     */
    int nb_beams;
    /**
     *
     */
    float range_min;
    /**
     *
     */
    float range_max;
    /**
     *
     */
    float angle_min;
    /**
     *
     */
    float angle_max;
    /**
     *
     */
    float angle_inc;
    /**
     *
     */
    float range[1000];
    /**
     * Les différents points reçus
     */
    geometry_msgs::Point current_scan[1000];
    /**
     * Booléen indiquant si un nouveau bu a &t& détecté
     */
    bool new_goal;
    
    //to perform detection of motion
    /**
     * Sauvegarde des points
     */
    float background[1000];//to store the background
    /**
     * booléen indiquant si le ième point est dynamique
     */
    bool dynamic[1000];//to store if the current is dynamic or not

    //to perform clustering
    /**
     * Nombre de cluster
     */
    int nb_cluster;// number of cluster
    /**
     * Tableau des cluster
     */
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    /**
     * Tableau des tailles de cluster
     */
    float cluster_size[1000];// to store the size of each cluster
    /**
     * Positions des clusters
     */
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    /**
     * Les clusters dynamiques
     */
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic
    /**
     * Premier indice du débbut et de la fin de chaque cluster
     */
    int cluster_start[1000], cluster_end[1000];

    //to perform detection of moving legs and to store them
    /**
     * Nombre de jambes détectés
     */
    int nb_moving_legs_detected;
    /**
     * Les positions des différentes jambes
     */
    geometry_msgs::Point moving_leg_detected[1000];// to store the middle of each moving leg

    //to perform detection of moving person and store them
    /**
     * Nombre de personnes détectées
     */
    int nb_moving_persons_detected;
    /**
     * Position des différentes personnes
     */
    geometry_msgs::Point moving_persons_detected[1000];// to store the middle of each moving person

    //to store the goal to reach that we will be published
    /**
     * Le point à atteindre pour le robot
     */
    geometry_msgs::Point goal_to_reach;

    // GRAPHICAL DISPLAY
    /**
     * Nombre point à afficher
     */
    int nb_pts;
    /**
     * Les points à afficher
     */
    geometry_msgs::Point display[2000];
    /**
     * Les couleurs
     */
    std_msgs::ColorRGBA colors[2000];

    //to check if the robot is moving or not
    /**
     *
     */
    bool previous_robot_moving;
    /**
     *
     */
    bool current_robot_moving;
    /**
     *Publish goal_to_reach
     */
    ros::Publisher pub_goal_to_reach;
    /**
     * to check if new data of laser is available or not
     */
    bool new_laser;
    /*
     * to check if new data of robot_moving is available or not
     */
    bool new_robot;
    /**
     *Publish token
     */
    ros::Publisher pub_token;
    /**
     * Nombre d'iteration
     */
    int nb_iter;
    /**
     * Score de chaque jambe
     */
    int score[1000];
    /**
     *to perform detection of moving legs and to store them
     */
    int nb_group_detected;
    /**
     * to store the middle of each group
     */
    geometry_msgs::Point group_detected[1000];
    /**
     * Vérifie que le robot a fini son mouvement
     */
    bool finish_move;
public:
    /**
     * \fn moving_persons_detector()
     * Constructeur de la classe moving_person_detector
     */
    moving_persons_detector() {
        nb_iter=0;
        finish_move = true;
	//Inscription au scan
        sub_scan = n.subscribe("scan", 1, &moving_persons_detector::scanCallback, this);
	//inscription à l'état du robot
	sub_robot_moving = n.subscribe("robot_moving", 1, &moving_persons_detector::robot_movingCallback, this);
	//Création du topic pour les tokens
	pub_token = n.advertise<std_msgs::Bool>("token",0);
	//Création du topic pour les markers
	pub_moving_persons_detector_marker = n.advertise<visualization_msgs::Marker>("moving_persons_detector", 1); 
	//Création du topic pour le but
	pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);// Preparing a topic to publish the goal to reach.
	//Incription au topic finish_move
	sub_finish_move = n.subscribe("finish_move", 1, &moving_persons_detector::finish_move_Callback, this);
        current_robot_moving = true;
        new_laser = false;
        new_robot = false;
        new_goal = false;
        //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10);// this node will run at 10hz
        while (ros::ok()) {
            ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
            update();//processing of data
            r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }

    }
    /**
     * \fn finish_move_Callback(const std_msgs::Bool::ConstPtr& a )
     * \brief réception des messages du topic finish_move
     * \param a : le message
     */
    void finish_move_Callback(const std_msgs::Bool::ConstPtr& a ){
        finish_move = a->data;
    }

    /**
     * \fn update()
     * \brief Met à jour le noeud
     */
    void update() {
        // we wait for new data of the laser and of the robot_moving_node to perform laser processing
        if ( new_laser && new_robot && finish_move) {
            new_laser = false;
            new_robot = false;
            nb_pts = 0;

            // if the robot is not moving then we can perform moving persons
	    // detection
            if ( !current_robot_moving ) {
                nb_iter++;
                // if the robot was moving previously and now it is
		// not moving now then we store the background
                if ( previous_robot_moving && !current_robot_moving )
                    store_background();

                //we search for moving persons in 4 steps
                detect_motion();//to classify each hit of the laser as dynamic or not
                perform_clustering();//to perform clustering
                detect_moving_legs();//to detect moving legs using cluster
                if(nb_iter % iter_person == 0){
		    //to detect moving_persons using moving legs detected
                    detect_moving_persons();
                    detect_group();
                    ROS_INFO("leg : %d, person : %d, group : %d",nb_moving_legs_detected, nb_moving_persons_detected, nb_group_detected );
                    send_goal_to_reach();
                    reset_score();
                }
                populateMarkerTopic();
            }
        }
    }// update

    /**
     * \fn send_goal_to_reach()
     * \brief envoie du but
     */
    void send_goal_to_reach() {
        finish_move = false;
        bool detected = (nb_group_detected > 0);
	//Si au moins un groupe a été détecté, on envoie le plus proche
        if(detected){
            ROS_INFO("Normal");
            goal_to_reach.x = closest_group().x;
            goal_to_reach.y = closest_group().y;
        }else{//Sinon on envoie un point aléatoire
            ROS_INFO("Random");
            goal_to_reach.x = ( (float) rand() /(RAND_MAX/4) )-2.0;
            goal_to_reach.y = (goal_to_reach.x >= 0 ? 2.5 - goal_to_reach.x :
			       2.5 - (0-goal_to_reach.x)) ;
        }
        send_token(detected);
        pub_goal_to_reach.publish(goal_to_reach);
    }

    /**
     * \fn store_background()
     * \brief Enregistre les points
     */
    void store_background() {
        // store all the hits of the laser in the background table
        for (int loop=0; loop<nb_beams; loop++)
            background[loop] = range[loop];

    }//init_background

    /**
     * \fn detect_motion()
     * \brief detecte la mouvement des points
     */
    void detect_motion() {
        for (int loop=0; loop<nb_beams; loop++ ){//loop over all the hits
            //Compute distance d
            float x = background[loop]<0?0-background[loop]:background[loop];
            float y = range[loop] < 0 ? 0-range[loop] : range[loop];
            float d = x-y;
            if (d > detection_threshold){
                dynamic[loop] = 1;//the current hit is dynamic
            }else
                dynamic[loop] = 0;//else its static
    
        }
    }//detect_motion

    /**
     * \fn perform_clustering ()
     * \brief Effectue le clustering des points
     */
    void perform_clustering() {
        nb_cluster = 0;//to count the number of cluster

        //initialization of the first cluster
        cluster_start[0] = 0;// the first hit is the start of the first cluster
        cluster[0] = 0;// the first hit belongs to the first cluster
        cluster_size[nb_cluster]=0;
        cluster_end[nb_cluster] = 0;
        int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic
        int nb_elem=1;
	/**
	 * Si deux points sont distancé d'au plus cluster_threshold, alors 
	 * il font parties du même cluster, sinon ils sont dans deux clusters 
	 * diffrents
	 *
	 * cluster_start[i] : contient le début du cluster i
	 * cluster_end[i] : contient la fin du cluster i  
	 */
        for( int loop=1; loop<nb_beams; loop++ ){
            //Compute distance d
            float d = distancePoints(current_scan[loop],current_scan[loop-1]);
            //ROS_INFO("distance = %f", d);
            if(d <= cluster_threshold){
                cluster_size[nb_cluster]+=d;//update size's cluster
                cluster_end[nb_cluster] = loop;//update end's cluster
                if ( dynamic[loop] )
                    nb_dynamic++;
                nb_elem++;
            }else{
            
                float p = nb_dynamic / (float)nb_elem;
                if(p >= dynamic_threshold){
                    cluster_dynamic[nb_cluster]=1;
                }else{
                    cluster_dynamic[nb_cluster]=0;
                }
                nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic
                nb_elem=1;
                nb_cluster++;
                cluster_start[nb_cluster] = loop;
                cluster_size[nb_cluster] = 0;
                cluster_end[nb_cluster] = loop;
                cluster[loop] = nb_cluster;
                if ( dynamic[loop] )
                    nb_dynamic++;
            
            }
        }
	/**
	 * On vérifie que les clusters sont dynamique
	 */
        if(nb_dynamic >= dynamic_threshold){
            cluster_dynamic[nb_cluster]=1;
        }else{
            cluster_dynamic[nb_cluster]=0;
        }

        //Compute middles
        for(int i=0; i<nb_cluster; i++){
            float x1 = current_scan[cluster_start[i]].x;
            float y1 = current_scan[cluster_start[i]].y;
            float x2 = current_scan[cluster_end[i]].x;
            float y2 = current_scan[cluster_end[i]].y;
            geometry_msgs::Point m;
            cluster_middle[i].x = (float)(x2+x1)/2;
            cluster_middle[i].y = (float)(y2+y1)/2;
        }

    }//perfor_clustering

    /**
     * \fn detect_moving_legs  ()
     * \brief Effectue la détection des jambes
     *
     * Chaque jambe a un score qui lui est attribué, le score est incrémenté
     * à chaque fois qu'une jambe est revue  
     */
    void detect_moving_legs() {
        int nb_leg = 0;
        geometry_msgs::Point legs[1000];
        for (int loop=0; loop<nb_cluster; loop++)
            if(cluster_size[loop] > leg_size_min && cluster_size[loop] <
               leg_size_max && cluster_dynamic[loop] )
                legs[nb_leg++] = cluster_middle[loop];

        for(int i = 0; i < nb_leg; i++) {
            bool b = true;
            int i_min = 0;
            float dist_min = 2;
	    //Détection des jambes
            for(int j =0; j<nb_moving_legs_detected; j++) {
                if(distancePoints(legs[i], moving_leg_detected[j]) < dist_min) {
                    i_min = j;
                    dist_min = distancePoints(legs[i], moving_leg_detected[j]);
                }
            }
	    /**
	     * Si deux jambes sont assez proches, le score augments
	     */ 
            if(dist_min > 0.5) {
                nb_moving_legs_detected++;
                moving_leg_detected[ nb_moving_legs_detected] = legs[i];
                score[nb_moving_legs_detected]=1;
            }else{//Sinon c'est une nouvelle jambes
                moving_leg_detected[i_min] = legs[i];
                score[i_min]++;
            }
        }
	/**
	 * Affichage des james
	 */
        for(int j =0; j<nb_moving_legs_detected; j++) {
            
            display[nb_pts].x = moving_leg_detected[j].x;
            display[nb_pts].y = moving_leg_detected[j].y;
            display[nb_pts].z = moving_leg_detected[j].z;

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;
        }

    }//detect_moving_legs

    /**
     * \fn detect_moving_persons()
     * \brief Détect les personnes
     * 
     * Une personnes possède deux jambes espacé d'au plus une certaines tailles,
     * chaque score doit avoir un score d'au moins 7
     */
    void detect_moving_persons() {
        nb_moving_persons_detected = 0;

        for (int loop_leg1=0; loop_leg1<nb_moving_legs_detected; loop_leg1++)//loop over all the legs
            for (int loop_leg2=loop_leg1+1; loop_leg2<nb_moving_legs_detected; loop_leg2++){//loop over all the legs
                if(score[loop_leg2]<=min_view || score[loop_leg1]<=min_view)
                    continue;
                //Compute distance d
                float d = distancePoints(moving_leg_detected[loop_leg1],
                                         moving_leg_detected[loop_leg2]);
                if (d<=legs_distance_max){

                    // we update the moving_persons_detected table to store the middle of the moving person
                
                    float x2 = moving_leg_detected[loop_leg1].x;
                    float y2 = moving_leg_detected[loop_leg1].y;
                    float x1 = moving_leg_detected[loop_leg2].x;
                    float y1 = moving_leg_detected[loop_leg2].y;
                    moving_persons_detected[nb_moving_persons_detected].x
			=(x1+x2)/2;
                    moving_persons_detected[nb_moving_persons_detected].y
			=(y1+y2)/2;
             
                    // the moving persons are green
                    display[nb_pts].x = moving_persons_detected
			[nb_moving_persons_detected].x;
                    display[nb_pts].y = moving_persons_detected
			[nb_moving_persons_detected].y;
                    display[nb_pts].z = moving_persons_detected
			[nb_moving_persons_detected].z;

                    colors[nb_pts].r = 0;
                    colors[nb_pts].g = 1;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;

                    nb_pts++;

                    //update of the goal
                    goal_to_reach.x = moving_persons_detected
			[nb_moving_persons_detected].x;
                    goal_to_reach.y = moving_persons_detected
			[nb_moving_persons_detected].y;

                    nb_moving_persons_detected++;
                    new_goal = true;
                }
            }

    }//detect_moving_persons

    /**
     * \fn geometry_msgs::Point closest_group()
     *
     * \brief renvoie le groupe le plus proche du robot
     * \return le groupe le plus proche
     */  
    geometry_msgs::Point closest_group() {
        int i_min = 0;
        float dist_min = 0, d;
        geometry_msgs::Point robot_position;
        robot_position.x=0;
        robot_position.y=0;
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
     * \fn scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
     * \brief callback du scan du laser
     * \param scan le scan
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

        new_laser = true;
        // store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;
        float beam_angle = angle_min;
        for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
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

    /**
     * \fn robot_movingCallback(const std_msgs::Bool::ConstPtr& state)
     * \brief callback de l'état du robot
     * \param state l'état
     */
    void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {
        new_robot = true;
        previous_robot_moving = current_robot_moving;
        current_robot_moving = state->data;
    }//robot_movingCallback

    /**
     * \fn distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
     * \brief Revoie la distance entre deux points
     * \param pa le premier point
     * \param pb le second point
     * \return la distance entre le point pa et le point pb
     */
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
    }

    // Draw the field of view and other references
    /**
     * \fn populateMarkerReference()
     * \brief affichage des points
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

        pub_moving_persons_detector_marker.publish(references);

    }

    /**
     * \fn populateMarkerTopic()
     * \brief affichage des points
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

        pub_moving_persons_detector_marker.publish(marker);
        populateMarkerReference();

    }

    /**
     * \fn send_token(bool b)
     * \brief Envoie du token
     * \param b la valeur du token
     */
    void send_token(bool b) {
        std_msgs::Bool msg_token;
        msg_token.data = b;
        pub_token.publish(msg_token);
    }

    /**
     * \fn print_sorce()
     * \brief affichage des scores
     */
    void print_sorce() {
        ROS_INFO("Score : "); 
        for(int i=0; i<nb_moving_legs_detected; i++)
            ROS_INFO("%d", score[i]);
    }

    /**
     * \fn reset_score()
     * \brief Suppression des scores
     */
    void reset_score(){
        nb_moving_legs_detected = 0;
        nb_moving_persons_detected = 0;
        nb_group_detected=0;
        for(int i=0; i<1000; i++)
            score[i]=0;
    }

    /**
     * \fn void detect_group ()
     * \brief Recherche les différents groupes
     */
    void detect_group() {
        if(nb_moving_persons_detected){
            nb_group_detected = 0;
            int nb_group = 0;//to count the number of group

            int group_start[1000];
            int group_end[1000];
    
            //initialization of the first cluster
            group_start[0] = 0;
            group_end[0] = 0;
            int loop;

	    //ROS_INFO("nb per0son = %d", nb_person_active);
            for(loop = 1; loop < nb_moving_persons_detected; loop++) {
                float d;
                d=distancePoints(moving_persons_detected[loop-1],moving_persons_detected[loop]);
                //ROS_INFO("distance = %f", d);
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
                if(group_start[i] < group_end[i]){
                    float x1 = moving_persons_detected[group_start[i]].x;
                    float y1 = moving_persons_detected[group_start[i]].y;
                    float x2 = moving_persons_detected[group_end[i]].x;
                    float y2 = moving_persons_detected[group_end[i]].y;
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
        }
        ROS_INFO("%d group have been detected.\n", nb_group_detected);
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "moving_persons_detector");

    moving_persons_detector bsObject;

    ros::spin();

    return 0;
}
