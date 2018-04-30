# Robot distributeur de café (cofee_distributor)


- [Abdullah Hasim Mohd Thaqif](mailto:thaqifabdullah@gmail.com)
- [Grand Maxence](mailto:maxence.grand@etu.univ-grenoble-alpes.fr)
- [Schwing Georges](mailto:georges.schwing@free.fr)

## But du projet

Le but du projet est d'implanter un programme pour qu'un robot utilisant ros distribue du café à un groupe de personne. Vous trouverez plus d'information sur le but et sur son implantation dans le fichier specification.pdf.

## Code et documentation

Vous trouverez le code dans le répertoire src, et la documentation au format HTML dans le répertoire documentation/HTML et au format Latex dans le répertoire docuementation/Latex. Pour la documentation Latex, lire le READE.md présent dans le répertoire pour générer le fichier au format pdf.

## Les noeuds

* decision node : Noeud pour la décision
* oersons_detection_node : Noeud pour la détection des personnes
* obstacle_detection_node : noeud pour la détection des obstacle
* robot_moving_node : Noeud pour savoir si le robot est en mouvement
* rotation_action_node : Noeud pour gérer la rotation du robot
* translation_action_node : Noeud pour gérer les translations du robot 
