# roboteq_debug

mtr_connect.cpp (source file where I configure ctrlr and send commands to ctrlr)
mtr_connect_node.cpp  (source file that makes connection to ctrlr)
test_sled.py  (test ROS publisher which publishes position for motors to move to)

To launch the mtr_connect node, type "roslaunch sled_motor sled_motor.launch", 
then "rosrun sled_motor test_sled.py" to command the motor to move a certain distance.  


Video Link:  https://www.youtube.com/watch?v=X6pa8o40-c8  
