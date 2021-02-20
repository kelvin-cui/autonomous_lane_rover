# autonomous_lane_rover
<img src="https://i.ibb.co/nCW67Xm/152375054-496836404648972-4190929028758680099-n.jpg" alt="152375054-496836404648972-4190929028758680099-n" border="0">

Autonomous lane-following rover using openCV to identify and predict lane positions, and KP feedback loop for movement control, and ROS as a robotics framework to combine different modules together. The image processing was done on a Raspberry Pi 4B, which sends the appropriate motor commands to an Arduino Nano, which then sends the commands to stepper motors controlling a differential drive.


## objective
The lane-following rover was designed to follow a lane marked by duct tape, as a proof of concept for future projects. I wanted to create a project utilizing ROS and feedback control schemes, which I learned in class. OpenCV was something that I wanted to learn on my own, applying it for lane detection for this project. Putting these together, I wanted to create a robot that would follow a track with challenging features, such a chicanes and decreasing radius turns. 

<img src="https://i.ibb.co/qpDWSh9/151649569-354136375634744-1835493396324256374-n.jpg" alt="rover" border="0">

## mechanical/electrical design:
The mechanical design of the rover went through multiple iterations of refinement. V1 was designed with DC motors in mind, but even with a gearbox, there was not fine enough control to accurately position the rover on the relatively small table surface.

<img src="https://i.ibb.co/JrBX14D/roverv1.png" alt="roverv1" border="0">

As a result, V2 used stepper motors, as well as a stand-alone Arduino Nano that communicated with the Raspberry Pi over serial to control the motors. However, this design used the default V1.3 Pi Camera, which had a minimum focus distance of 0.5m - too far for our use. Furthermore, the position of the drive motors at the rear increased the torque required to turn the robot, causing in slippage of the tires.

<img src="https://i.ibb.co/7Nvj1Sv/roverv2.png" alt="roverv2" border="0">

The final design came with moving the differential drive tires to the middle, as well as moving the camera module to the top of the robot to get as clear of an image as possible. The camera was swapped for the wide angle module, and the microprocessors were moved to the top as they required the most interaction. This redesign also allowed the entire rover to be much more compact, utilizing the space given as effectively as possible. 

<img src="https://i.ibb.co/DbRTMgt/roverv3.png" alt="roverv3" border="0">


The layered design was heavily inspired by racing quadcopters, where carbon fiber plates would be layered on top of one another using standoffs. All parts were designed in Solidworks, and 3D printed using PETG.


The Arduino connects to two ULN2003 stepper driver modules, which are then connected to 28BYJ-48 Stepper Motors. The Raspberry Pi 4 is powered by an external 10000mAh battery bank capable of delivering 3A at 5V. The Arduino Nano is powered over the USB connection with the Raspberry Pi, while the stepper drivers have a separate 5V connection to the battery bank.


## software design:

### lane detection:
The lane detection was done utilizing OpenCV, in the camera.py module. When a ROS image is received, the module first converts it to an OpenCV image. The image is then masked to only detect a certain distance away from the rover, ignoring objects in the distant background. It then finds the canny edges, which are then grouped into lanes using hierarchical clustering, based on each points relative euclidian distance. Then, the cluster of points is fitted to a quadratic equation, which predicts the lane position near the center of the robot. It then finds the middle of the lane in terms of pixels as well as phi_avg, the angle the lanes are relative to the robot. 

![Lane Detection:](https://github.com/ntnox/autonomous_lane_rover/blob/main/lane_detection.gif)

*Left: The white lines are the predicted lanes, green are the left and right lane x locations, and the red line represents the middle of both lanes, as well as the slope of the lane. Right : The Canny Edges detected after the mask is applied*

### movement:
x_avg and phi_avg are then used in a KP feedback loop. x_avg is centred at 0, ranging from -320pixels to +320pixels (the camera has a resolution of 640 pixels across). Similarly, phi_avg is centred at 0, ranging from -90degrees to 90degrees. Both are normalized to -1/+1, and then used in a proportional feedback loop to regulate the values to zero. The forward velocity is fixed at 0.07m/s, while the rotational velocity is controlled by this feedback loop, with tunable gains on the x_error and phi_error. This can all be found in move.py. Once the rotational velocity is determined, it's then sent to motor.py, which takes the linear and rotational velocities and calculates the rotational speed of each motor.

![Lane Following:](https://github.com/ntnox/autonomous_lane_rover/blob/main/lane_following.gif)

### ros and optimization:
The different modules communicate using the ROS framework. The original camera image is published to the image_raw topic, which is subscribed to by the camera.py node. The camera node publishes to the x_avg and phi_avg topics with Float64 values, which is subscribed to by the move.py node. The Move node then publishes Twist messages, including the linear and rotational velocities, which are subscribed to by the motor.py node. The motor.py node communicates the motor commands over serial to the Arduino. 

I made use of two external ros packages - the usb_cam test package in order to quickly publish images from the Pi Camera, and the web_video_server package to visualize the images for debugging.

The camera node requires the most computational time - as a result, it was optimized to have a maximum runtime of 0.06 seconds on the Pi4, allowing for a global rate of 10hz. 
