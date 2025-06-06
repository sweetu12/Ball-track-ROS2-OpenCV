
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped # To publish ball center coordinates
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        # Declare parameters for HSV color range and image topic
        self.declare_parameter('image_topic', '/image_raw')
        # These are HSV values for a tennis ball, but might need tuning.
        # Hue: 0-179, Saturation: 0-255, Value: 0-255 in OpenCV
        self.declare_parameter('hsv_lower_h', 29) # Lower Hue
        self.declare_parameter('hsv_lower_s', 86) # Lower Saturation
        self.declare_parameter('hsv_lower_v', 60) # Lower Value
        self.declare_parameter('hsv_upper_h', 64) # Upper Hue
        self.declare_parameter('hsv_upper_s', 255)# Upper Saturation
        self.declare_parameter('hsv_upper_v', 255)# Upper Value
        self.declare_parameter('min_radius', 10)  # Minimum radius of the ball to detect

        # Get parameter values
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.hsv_lower = np.array([
            self.get_parameter('hsv_lower_h').get_parameter_value().integer_value,
            self.get_parameter('hsv_lower_s').get_parameter_value().integer_value,
            self.get_parameter('hsv_lower_v').get_parameter_value().integer_value
        ])
        self.hsv_upper = np.array([
            self.get_parameter('hsv_upper_h').get_parameter_value().integer_value,
            self.get_parameter('hsv_upper_s').get_parameter_value().integer_value,
            self.get_parameter('hsv_upper_v').get_parameter_value().integer_value
        ])
        self.min_radius = self.get_parameter('min_radius').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Subscriber to the image topic
        self.image_subscriber = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10  #  profile depth
        )
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

        # Publisher for the processed image with tracking visuals
        self.image_publisher = self.create_publisher(
            Image,
            '/ball_tracking/image_processed',
            10
        )
        self.get_logger().info("Publishing processed image to /ball_tracking/image_processed")

        # Publisher for the ball's 2D coordinates (and radius in z)
        self.ball_coords_publisher = self.create_publisher(
            PointStamped, # Using PointStamped for timestamp and frame_id
            '/ball_tracking/ball_coordinates',
            10
        )
        self.get_logger().info("Publishing ball coordinates to /ball_tracking/ball_coordinates")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            cv_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # frame dimensions
        h, w, _ = cv_frame.shape
        if h == 0 or w == 0:
            self.get_logger().warn("Received an empty frame.")
            return

        # Blur the frame to reduce noise and convert to HSV color space
        blurred_frame = cv2.GaussianBlur(cv_frame, (11, 11), 0)
        hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the tennis ball color
        mask = cv2.inRange(hsv_frame, self.hsv_lower, self.hsv_upper)
        
        # remove small blobs
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        center = None
        radius = 0

        if len(contours) > 0:
            # Find the largest contour, assume it's the ball
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0: # Ensure non-zero area to avoid division by zero
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # Proceed only if the radius meets a minimum size
                if radius > self.min_radius:
                    # Draw the circle and centroid on the frame
                    cv2.circle(cv_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2) # Yellow circle
                    cv2.circle(cv_frame, center, 5, (0, 0, 255), -1) # Red centroid

                    # Publish ball coordinates
                    point_msg = PointStamped()
                    point_msg.header = msg.header # Use the same header as the input image
                    point_msg.point.x = float(center[0]) # Center x in pixels
                    point_msg.point.y = float(center[1]) # Center y in pixels
                    point_msg.point.z = float(radius)    # Radius in pixels (using z for radius)
                    self.ball_coords_publisher.publish(point_msg)
                else:
                    # Reset center if radius is too small
                    center = None 
            else:
                center = None # No valid moments
        else:
            # No contours found
            pass


        # Publish the processed frame
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_frame, "bgr8")
            processed_image_msg.header = msg.header # Preserve header
            self.image_publisher.publish(processed_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error (publishing): {e}")


def main(args=None):
    rclpy.init(args=args)
    ball_detector_node = BallDetectorNode()
    try:
        rclpy.spin(ball_detector_node)
    except KeyboardInterrupt:
        ball_detector_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:

        ball_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

