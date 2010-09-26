#!/bin/bash -x

# Run roslaunc wpdps1 start_all.launch in one terminal first
rosrun gazebo_tools gazebo_model -f `rospack find gazebo_worlds`/objects/desk5.model spawn table2 -x 5 -y 0 -z 0.65
sleep 10
rosrun gazebo_tools gazebo_model -f `rospack find gazebo_worlds`/objects/coffee_cup.model spawn coffee_cup -x 5 -y 0 -z .95 -Y 90
