import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math


class MultiImageSubscriber(Node):
    def __init__(self):
        super().__init__('multi_image_subscriber')
        self.bridge = CvBridge()

        # List of topics to subscribe to
        
        self.camera_1_topic = '/platypus/camera_1/dec/manual_white_balance'
        self.camera_2_topic = '/platypus/camera_2/dec/manual_white_balance'
        self.camera_3_topic = '/platypus/camera_3/dec/manual_white_balance'
        
        self.camera_1_angle = -35
        self.camera_2_angle = 0
        self.camera_3_angle = 35 
        
        self.target_col_size = 1500
        self.target_camera_2_col_size= 370
        self.overlap =45
        
        self.image_topics = [self.camera_1_topic, self.camera_2_topic,self.camera_3_topic]

        # Dictionary to hold the latest image from each topic
        self.images = {}
        
        # Create a publisher for panorama image
        self.panoramic_image_publisher_ = self.create_publisher(Image, 'panoramic_image', 10)

        # Create a subscription for each topic
        self.my_subscriptions = []
        for topic in self.image_topics:
            subscription = self.create_subscription(Image, topic, lambda msg, topic_name=topic: self.image_callback(msg, topic_name), 10)
            self.my_subscriptions.append(subscription)

        self.get_logger().info(f'Subscribed to topics: {self.image_topics}')

    def image_callback(self, msg, topic_name):
        try:
            # Convert the ROS Image message to a NumPy array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')       
            # Store the image in the dictionary
            self.images[topic_name] = cv_image
            
            # If all images are received, merge and display them
            if len(self.images) == len(self.image_topics):
                merged_image = self.adjust_and_merge_images()
                cv2.imshow('Merged Image', merged_image)
                cv2.waitKey(1)  # Required to display the image
                 # Convert OpenCV image to ROS Image message
                merged_image_msg = self.bridge.cv2_to_imgmsg(merged_image, encoding='bgr8')
                self.panoramic_image_publisher_.publish(merged_image_msg)
                self.get_logger().info("Published the panoramic image.")

        except Exception as e:
            self.get_logger().error(f'Error processing image from {topic_name}: {e}')
            
    def rotate_3d_image(self, img, rot_x, rot_y, rot_z):
        #Source: # https://github.com/saleh1312/python/blob/master/opencv%20-%203d%20rotation
        # Calculate the diagonal length for the new canvas size
        diagonal = int(math.sqrt(img.shape[0]**2 + img.shape[1]**2))
        new_row = diagonal
        new_col = diagonal

        # 2D to 3D projection (adjust for center)
        proj2dto3d = np.array([
            [1, 0, -img.shape[1] / 2],  # Center to origin
            [0, 1, -img.shape[0] / 2],
            [0, 0, 0],
            [0, 0, 1]
        ], np.float32)

        # Initialize rotation matrices
        rx = np.eye(4, dtype=np.float32)
        ry = np.eye(4, dtype=np.float32)
        rz = np.eye(4, dtype=np.float32)

        # Translation matrix to move the image along the Z-axis
        trans = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 300],  # Adjust Z-axis translation
            [0, 0, 0, 1]
        ], np.float32)

        # 3D to 2D projection
        proj3dto2d = np.array([
            [200, 0, new_row / 2, 0],  # Adjust for new canvas center
            [0, 200, new_col / 2, 0],
            [0, 0, 1, 0]
        ], np.float32)

        # Rotation angles (convert to radians)
        ax = float(rot_x * (math.pi / 180.0))
        ay = float(rot_y * (math.pi / 180.0))
        az = float(rot_z * (math.pi / 180.0))

        # Populate rotation matrices
        rx[1, 1] = math.cos(ax)
        rx[1, 2] = -math.sin(ax)
        rx[2, 1] = math.sin(ax)
        rx[2, 2] = math.cos(ax)

        ry[0, 0] = math.cos(ay)
        ry[0, 2] = -math.sin(ay)
        ry[2, 0] = math.sin(ay)
        ry[2, 2] = math.cos(ay)

        rz[0, 0] = math.cos(az)
        rz[0, 1] = -math.sin(az)
        rz[1, 0] = math.sin(az)
        rz[1, 1] = math.cos(az)

        # Combine transformations
        r = rx.dot(ry).dot(rz)
        final = proj3dto2d.dot(trans.dot(r.dot(proj2dto3d)))

        # Apply perspective transformation with the expanded canvas size
        dst = cv2.warpPerspective(
            img,
            final,
            (new_row, new_col),
            None,
            cv2.INTER_LINEAR,
            cv2.BORDER_CONSTANT,
            (255, 255, 255)
        )

        return dst
    
    def crop_whitespace(self,image):
    # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Threshold the image to create a binary mask
        _, binary = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours of the non-white regions
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get the bounding box of the largest contour
            x, y, w, h = cv2.boundingRect(np.vstack(contours))
            
            # Crop the image to the bounding box
            cropped_image = image[y:y+h, x:x+w]
            return cropped_image
        else:
            # If no contours are found, return the original image
            return image

    def resize_image_to_col(self, image, target_col): #keeps aspect ratio but reduces size of image based on col
        # Get the original dimensions of the image
        original_col, original_row = image.shape[:2]
        
        # Calculate the aspect ratio
        aspect_ratio = original_row / original_col
        
        # Calculate the new row based on the target col while maintaining aspect ratio
        target_row = int(target_col * aspect_ratio)
        
        # Resize the image
        resized_image = cv2.resize(image, (target_row, target_col), interpolation=cv2.INTER_LINEAR)
        
        return resized_image
    
    def pad_to_target_col(self, image, target_col): #adds white space to top and bottom of reduced image to match col of sides
        # Get the current col and row of the image
        original_col, original_row = image.shape[:2]
        
        # Calculate the padding required to reach the target col
        padding_needed = target_col - original_col
        
        if padding_needed <= 0:
            # If the image is already taller or equal to the target col, return it as is
            return image
        
        # Calculate padding for the top and bottom (even distribution if possible)
        padding_top = padding_needed // 2
        padding_bottom = padding_needed - padding_top  # This ensures total padding is equal to the difference
        
        # Add white padding (or any other color) using cv2.copyMakeBorder
        padded_image = cv2.copyMakeBorder(image, padding_top, padding_bottom, 0, 0, 
                                        cv2.BORDER_CONSTANT, value=(255, 255, 255))  # White color padding
        
        return padded_image
    
    def hstack_with_overlap(self,*images, overlap):# Get the cols and rows of the images
        cols = [img.shape[0] for img in images]
        # rows = [img.shape[1] for img in images]
        
        # Ensure all images have the same col
        max_col = max(cols)
        
        # Resize images to the maximum col
        resized_images = []
        for img in images:
            col, row = img.shape[:2]
            if col != max_col:
                new_row = int(row * max_col / col)
                resized_img = cv2.resize(img, (new_row, max_col))
            else:
                resized_img = img
            resized_images.append(resized_img)
        
        # Apply the overlap by cropping the images
        cropped_images = []
        for i in range(len(resized_images)):
            img = resized_images[i]
            if i == 0:
                cropped_images.append(img[:, :-overlap])  # Crop right part of the first image
            elif i == len(resized_images) - 1:
                cropped_images.append(img[:, overlap:])  # Crop left part of the last image
            else:
                cropped_images.append(img[:, overlap:])  # Crop left part of the middle image
        
        # Stack the cropped images
        result = cropped_images[0]
        for img in cropped_images[1:]:
            result = np.hstack((result, img))  # Stack images horizontally

        return result                                                            
    
    def adjust_and_merge_images(self):       
      
        image_1 = self.images[self.camera_1_topic]
        image_2 = self.images[self.camera_2_topic]  
        image_3 = self.images[self.camera_3_topic]
          
        angled_image1 = self.rotate_3d_image(image_1, 0, self.camera_1_angle, 0)
        angled_image1=  self.crop_whitespace(angled_image1)
        resized_image1 = self.resize_image_to_col(angled_image1, self.target_col_size)
        
        angled_image2 = self.rotate_3d_image(image_2, 0, self.camera_2_angle, 0)
        angled_image2=  self.crop_whitespace(angled_image2)
        resized_image2 = self.resize_image_to_col(angled_image2, self.target_camera_2_col_size)
        padded_image2 = self.pad_to_target_col(resized_image2, self.target_col_size)
        
        angled_image3 = self.rotate_3d_image(image_3, 0, self.camera_3_angle, 0)
        angled_image3=  self.crop_whitespace(angled_image3)
        resized_image3 = self.resize_image_to_col(angled_image3, self.target_col_size)   
        
        merged_image = self.hstack_with_overlap(resized_image3, padded_image2, resized_image1, overlap=self.overlap)
        
        # cv2.imshow('Image 1', resized_image1)     
        # cv2.imshow('Image 2', padded_image2)
        # cv2.imshow('Image 3', resized_image3)
        cv2.waitKey(1)  # Required to display the image
 
        return merged_image

def main():
    rclpy.init()
    node = MultiImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
