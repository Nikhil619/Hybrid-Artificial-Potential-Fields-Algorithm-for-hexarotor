# Hybrid-Artificial-Potential-Fields-Algorithm
# The code outputs V_x, V_y and time duration which should be passed to the pixhawk flight controller
# We have used an optical flow sensor with sonar (px4flow) in the place of gps for position estimation
# We have used a rotation lidar to form a map of the surroundings in every time step
# THIS CODE CAN BE USED FOR COMPLETE 360 degree INDOOR OBSTACLE AVOIDANCE
# Please check the dronekit documentation to set the optical flow sensor up with the flight controller(pixhawk) using mission planner

# tool for rplidar - https://github.com/Slamtec/rplidar_sdk/releases
