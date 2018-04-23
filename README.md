# Projet Robotique : robot distributeur de café

## Description du projet

## Compilation & Exécution
### Compilation
	catkin_make
### Connection au robot
	export ROS_MASTER_IP=http://<robair_ip>:13133
	export ROS_IP=<your_ip_address>
### Exécution d'un noeud
	rosrun node
## Clonage du projet

Avant de cloner le projet vérifiez que [Ros Kinetic et ses dépendances ont bien été installés](http://wiki.ros.org/kinetic/Installation/Ubuntu) que
que la package [sound_play](https://answers.ros.org/question/260933/how-to-install-sound_play/).

	git clone https://github.com/grandmaxIm2ag/catkin_ws.git
	catkin_make
	source devel/setup.bash
