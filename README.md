# **Ball Tracking using ROS 2 and OpenCV**

## **Problem Statement**

The goal of this project is to create a ROS 2 node that detects a tennis ball in a live camera feed and tracks its position. The algorithm uses the OpenCV library for all computer vision tasks and publishes the processed video stream and the ball's coordinates to ROS 2 topics for other nodes to use.

## **Approach**

The core computer vision logic follows these steps:

1. **Convert to HSV:** The input camera image (in BGR format) is converted to the HSV (Hue, Saturation, Value) color space. The HSV space is generally better for color detection under varying lighting conditions compared to BGR or RGB.  
2. **Color Filtering:** A binary mask is created by keeping only the pixels that fall within a predefined HSV color range for a greenish-yellow tennis ball.  
3. **Contour Detection:** The outlines (contours) of all the white shapes in the binary mask are found. To reduce noise, the mask is first cleaned up using erosion and dilation operations.  
4. **Process Contour:** The algorithm identifies the largest contour, assuming it is the tennis ball. It then calculates the minimum enclosing circle and the precise center (centroid) of this shape.  
5. **Visualize:** A yellow circle is drawn around the detected ball and a red dot is placed at its center on the original camera image, which is then published.

### **Published Topics**

* **/ball\_tracking/image\_processed** (sensor\_msgs/msg/Image): The original camera feed with the detection visuals drawn on it.
* 
* **/ball\_tracking/ball\_coordinates** (geometry\_msgs/msg/PointStamped): The pixel coordinates of the ball's center (x, y) and its radius (z).

### **Disadvantage**

The primary weakness of this method is its reliance on static color filtering. It is **vulnerable to significant changes in lighting conditions.** If a shadow falls on the ball or the ambient light changes, the ball's HSV values can shift outside the predefined range, causing the detection to fail.

## **How to Run**

### **1\. Place the Package**

Copy the tennis\_ball\_tracker folder into your ROS 2 workspace's source directory (e.g., \~/ros2\_ws/src/).

### **2\. Build the Package**

Navigate to the root of your ROS 2 workspace and build the package using colcon:  

cd \~/ros2\_ws  

colcon build \--packages-select tennis\_ball\_tracker

### **3\. Run the Nodes**

You will need at least two terminals. Make sure to source your workspace in each new terminal: source \~/ros2\_ws/install/setup.bash  

Terminal 1: Start the Camera  

Run the usb\_cam node to start publishing your camera feed.  

ros2 run usb\_cam usb\_cam\_node\_exe

Terminal 2: Start the Tracker  

Use ros2 launch to run the ball tracker node. This command also correctly remaps the input image topic.  

ros2 launch tennis\_ball\_tracker ball\_tracker\_launch.py image\_topic:=/image\_raw

### **4\. Visualize the Output**

Terminal 3: View the Result  

Use rqt\_image\_view to see the final processed video stream.  

rqt\_image\_view /ball\_tracking/image\_processed

## **Requirements**

* **ROS 2 Jazzy**  
* **Ubuntu 24.04 LTS**  
* **OpenCV 4.x** (Installed via python3-opencv)  
* **ros-jazzy-cv-bridge**  
* **ros-jazzy-usb-cam**
