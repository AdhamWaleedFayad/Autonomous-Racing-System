# Autonomous-Racing-System
This was a solo project initially completed during February 2024
The Vision Module utilizes OpenCV to detect the lane lines and calculates their slopes then uses numpy.linalg to find the intersection points by solving the straight line equations, the intersection point is then used to estimate the heading of the track 
The intersection of the lane lines with the bottom of the image is used to estimate the cross-track error

The Control Module is based on a Stanley controller, the heading error is calculated from the difference between Vehicle's Yaw and the track heading that is estimated from the vision module. The vehicles target velocity is set as a function of the heading error and a PID controller is used to achieve that target velocity
