#!/usr/bin/env python

# import rospy
# import cv2
# from ultralytics import YOLO
# import numpy as np
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import os # For checking if the model file exists

# # --- Configuration ---
# # IMPORTANT: Replace 'path/to/your/custom_yolo_obb_model.pt' with the actual path
# # to your trained YOLO OBB model file (e.g., 'runs/obb/train/weights/best.pt')
# # Ensure this path is correct relative to where you run your ROS node,
# # or provide an absolute path.
# model_path = '/home/d/icarus/path/to/your/custom_yolo_obb_model.pt' # Example absolute path

# # Check if the model file exists
# if not os.path.exists(model_path):
#     rospy.logerr(f"YOLO OBB model file not found at: {model_path}")
#     rospy.logerr("Please update 'model_path' to the correct location of your trained '.pt' file.")
#     rospy.signal_shutdown("Model file not found.") # Shut down ROS node if model is missing
#     exit() # Exit the script

# # Load your custom YOLO OBB model
# rospy.loginfo(f"Loading YOLO OBB model from: {model_path}")
# model = YOLO(model_path)
# rospy.loginfo("YOLO OBB model loaded successfully.")

# # Initialize CvBridge for converting ROS Images to OpenCV format
# bridge = CvBridge()

# # Define your object class names as specified in your dataset.yaml for OBB
# # IMPORTANT: Update 'your_class_name' to your actual class name, e.g., 'basket'
# class_names = ['your_class_name']

# # Drawing parameters
# obb_color = (0, 255, 0) # Green for Oriented Bounding Boxes (B, G, R)
# text_color = (255, 255, 255)    # White for text
# font_scale = 0.5
# font_thickness = 1
# confidence_threshold = 0.5 # Minimum confidence to display a detection

# def image_callback(data):
#     """
#     Callback function executed when a new image message is received.
#     """
#     try:
#         # Convert ROS Image message to OpenCV format (bgr8 is typical for color images)
#         cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     except CvBridgeError as e:
#         rospy.logerr(f"CvBridge Error: {e}")
#         return # Skip this frame if conversion fails

#     # Perform YOLO OBB inference on the OpenCV frame
#     # verbose=False suppresses detailed output for each frame
#     results = model(cv_image, verbose=False)

#     # Create a copy of the frame to draw on
#     annotated_frame = cv_image.copy()

#     # Process each detection result
#     for r in results:
#         # Get oriented bounding boxes data
#         # xywhr: (x_center, y_center, width, height, angle_degrees)
#         # conf: confidence scores for the detected object
#         obb_data = r.obb.xywhr.cpu().numpy()
#         confidences_data = r.obb.conf.cpu().numpy()
#         # cls: class indices (will be 0 for 'your_class_name')
#         class_indices_data = r.obb.cls.cpu().numpy().astype(int)

#         # Iterate through each detected object
#         for i in range(len(obb_data)):
#             # Check object confidence
#             obj_confidence = confidences_data[i]
#             if obj_confidence < confidence_threshold:
#                 continue # Skip if object confidence is too low

#             # Extract OBB data
#             x_center, y_center, width, height, angle_degrees = obb_data[i]
#             obj_class_idx = class_indices_data[i]

#             # Get the class name
#             obj_class_name = class_names[obj_class_idx] if obj_class_idx < len(class_names) else f"Unknown_Class_{obj_class_idx}"

#             # Create an OpenCV RotatedRect object
#             # Note: OpenCV angles are usually in the range [-90, 0) degrees, positive counter-clockwise from horizontal.
#             # YOLO's angle convention might vary, so you may need to adjust the angle (e.g., angle - 90) if the orientation is off.
#             rect = ((x_center, y_center), (width, height), angle_degrees)

#             # Get the four corner points of the rotated rectangle
#             box_points = cv2.boxPoints(rect)
#             box_points = np.int0(box_points) # Convert to integer coordinates

#             # Draw the oriented bounding box (polygon)
#             cv2.polylines(annotated_frame, [box_points], isClosed=True, color=obb_color, thickness=2)

#             # Add class name, confidence, and orientation label
#             # Format angle to 2 decimal places
#             label = f"{obj_class_name}: {obj_confidence:.2f}, Angle: {angle_degrees:.2f}°"
#             # Position the text near the top-left corner of the OBB for clarity
#             # You might need to adjust text position based on rotation for complex cases
#             text_x = min(box_points[:, 0]) # Get min x from corners
#             text_y = min(box_points[:, 1]) # Get min y from corners
#             cv2.putText(annotated_frame, label, (text_x, text_y - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, font_scale, obb_color, font_thickness, cv2.LINE_AA)


#     # Display the annotated frame
#     cv2.imshow("YOLO OBB Inference - ROS Camera", annotated_frame)
#     cv2.waitKey(1) # Important: Needed to refresh the OpenCV window

# def main():
#     """
#     Initializes the ROS node and subscribes to the image topic.
#     """
#     # Initialize the ROS node
#     rospy.init_node('yolo_obb_inference_node', anonymous=True)
#     rospy.loginfo("ROS node 'yolo_obb_inference_node' started.")

#     # Create a subscriber to the camera image topic
#     # The topic name is '/vtolcam/usb_cam/image_raw'
#     # The message type is Image from sensor_msgs.msg
#     # The callback function is image_callback
#     image_topic = "/vtolcam/usb_cam/image_raw"
#     rospy.Subscriber(image_topic, Image, image_callback, queue_size=1) # Added queue_size for better performance
#     rospy.loginfo(f"Subscribed to topic: {image_topic}")

#     # Keep the node running until it's shut down
#     rospy.spin()
#     rospy.loginfo("ROS node shut down.")

#     # Clean up OpenCV windows when the ROS node shuts down
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("ROS interrupt received. Exiting.")
#     except Exception as e:
#         rospy.logerr(f"An unexpected error occurred: {e}")




import rospy
import cv2
import torch
import numpy as np
from ultralytics import YOLO

# ROS Messages
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# Custom Message
# Make sure the package is compiled and sourced (`catkin_make` and `source devel/setup.bash`)
# to find the custom message definitions.
from ctl.msg import ObbDetection, ObbDetections

# For converting ROS Image messages to OpenCV images
from cv_bridge import CvBridge, CvBridgeError

# To find the package path
import rospkg

class YoloObbDetector:
    """
    A class to perform YOLO OBB detection on ROS Image messages and publish the results.
    """
    def __init__(self):
        """
        Initializes the ROS node, publishers, subscribers, and the YOLO model.
        """
        rospy.init_node('yolo_obb_detector', anonymous=True)

        # --- Parameters ---
        # Get the path to the model weights file from the ROS parameter server
        # The path should be specified in a launch file.
        # Default path is relative to the package.
        rospack = rospkg.RosPack()
        # package_path = rospack.get_path('ctl')
        default_model_path = "/home/d/icarus/catkin_ws/src/ctl/src/best.pt"
        model_path = rospy.get_param('~model_path', default_model_path)
        
        # Get confidence threshold from params
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)

        # --- Initialization ---
        self.bridge = CvBridge()
        
        # Check for CUDA availability
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo(f"Using device: {self.device}")

        # Load the YOLOv11 OBB model
        try:
            self.model = YOLO(model_path)
            self.model.to(self.device)
            rospy.loginfo(f"YOLO OBB model loaded successfully from {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            return

        # --- ROS Communication ---
        # Publisher for detections
        self.detections_pub = rospy.Publisher('/yolo_obb/xywhr', ObbDetections, queue_size=10)
        
        # Subscriber to the image topic
        image_topic = "/sky_vision/down_cam/img_raw"
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        rospy.loginfo(f"Subscribed to {image_topic}")
        rospy.loginfo("YOLO OBB detector node is ready.")

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Performs inference and publishes detections.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        target_width = 1920
        target_height = 1080
        
        # Resize the image
        # Using cv2.INTER_LINEAR for interpolation, which is a good general-purpose choice.
        # For downsampling, INTER_AREA is often preferred. For upsampling, INTER_CUBIC or INTER_LANCZOS4.
        resized_image = cv2.resize(cv_image, (target_width, target_height), interpolation=cv2.INTER_LINEAR)

        # Perform inference
        # The results object contains the detections
        results = self.model(resized_image, verbose=False, conf=self.confidence_threshold)
        
        # The first result in the list corresponds to the first image
        result = results[0]
        
        # Create the custom message to publish
        detections_msg = ObbDetections()
        detections_msg.header = msg.header # Use the same header as the input image
        
        # Check if there are any detections
        if result.obb is not None and len(result.obb.xywhr):
            # Process each detected object
            for i in range(len(result.obb.xywhr)):
                box = result.obb.xywhr[i]
                class_id = int(result.obb.cls[i])
                confidence = float(result.obb.conf[i])
                
                # Create a single detection message
                detection = ObbDetection()
                detection.header = msg.header
                detection.class_id = class_id
                detection.class_name = self.model.names[class_id]
                detection.confidence = confidence
                
                # Extract OBB parameters
                # xywhr format: [x_center, y_center, width, height, angle_rad]
                detection.x_center = float(box[0])
                detection.y_center = float(box[1])
                detection.width = float(box[2])
                detection.height = float(box[3])
                detection.angle_rad = float(box[4])
                
                # Add the detection to our list of detections
                detections_msg.detections.append(detection)

        # Publish the detections message (even if it's empty)
        self.detections_pub.publish(detections_msg)

        # --- Visualization ---
        # Use the built-in plot function from ultralytics to draw the boxes
        annotated_frame = result.plot()
        # if annotated_frame.shape[1] > self.display_width or annotated_frame.shape[0] > self.display_height:
        display_frame = cv2.resize(annotated_frame, (1080, 720), interpolation=cv2.INTER_AREA)
        # else:
            # display_frame = annotated_frame
        # Display the resulting frame
        cv2.imshow("obj detection", display_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = YoloObbDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
