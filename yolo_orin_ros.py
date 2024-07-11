#!/usr/bin/python3

import cv2
import torch
import rospy
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Import a custom YOLO model
model = YOLO("/home/orin/air_dmg_assesment_ws/src/yolo_ros/orin_run/best.pt")

class frame_process():
    def __init__(self):

        rospy.loginfo("started")
        # Set the device to a gpu or cpu ("cuda", "cpu", or use a specific gpu like "cuda:1")
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Create a Subscriber to get the raw image, Create 2 Publishers to publish the annotated image and the pixel coordinates
        self.frame_sub = rospy.Subscriber("/flir_boson/image_rect", Image, callback = self.frame_callback, queue_size = 1)
        self.frame_pub = rospy.Publisher("/annotated_frame", Image , queue_size = 10)
        self.coord_pub = rospy.Publisher("/coords", String , queue_size = 10)

        # Start a count for any intermittent processes that may need to be run (at certain frame numbers)
        self.count = 0

    # Loop through the video frames
    def frame_callback(self, frame):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(frame, desired_encoding='rgb8')

        # Add 1 to the count to count frame number
        self.count = self.count + 1
        rospy.loginfo(self.count)

        if self.count%30 == 0:
            # Run YOLOv8 tracking on the frame while persisting tracks between frames, adjust conf, iou, and other params
            results = model.track(frame, conf = 0.3, iou = 0.1, device = self.device, retina_masks = False, persist = True, tracker = '/home/orin/air_dmg_assesment_ws/src/yolo_ros/orin_run/bytetrack.yaml') #, classes = [0]) # Can set the model to track certain classes
            
            # Create a frame with the segments and boxes, can adjust the bool params to hide boxes, conf, id, label
            annotated_frame = results[0].plot()
            
            # If the result has no detection pass
            if results[0].boxes is None or results[0].boxes.id is None:
                pass

            # Get the boxes, track ids, and classifications for each detection in the frame
            else:
                boxes = results[0].boxes.xywh.cuda()
                track_ids = results[0].boxes.id.int().cuda().tolist()
                cls = results[0].boxes.cls.cuda()

                # Save the boxes, ids, and time stamps to the a dict
                for box, track_id, c in zip(boxes, track_ids, cls):
                    
                    # Save the label for the box
                    label = results[0].names[int(c)]

                    # Get box locations (x y center point)
                    x, y, w, h = box
                    label = str(label)
                    id = str(track_id)
                    x = str(float(x))
                    y = str(float(y))
                    point_data = label + "," + id + "," + x + "," + y
    
                    # Publish the tracking dictionary only with detections
                    self.coord_pub.publish(point_data)

            if self.count == 30:
                print(annotated_frame)
            # Convert the annotated frame to a ros Image
            processed_img = bridge.cv2_to_imgmsg(annotated_frame, encoding="passthrough")
            
            # Publish the frame regardless of detections
            self.frame_pub.publish(processed_img)
            
if __name__ == '__main__': 
    rospy.init_node('frame_process') 
    f = frame_process() 
    rospy.spin()
