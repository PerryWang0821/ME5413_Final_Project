#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import os
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from stereo_msgs.msg import DisparityImage
from tf.transformations import quaternion_matrix

class ImprovedTemplateMatcher:
    def __init__(self):
        self.node_name = "template_matcher"
        rospy.init_node(self.node_name, anonymous=True)
        self.cv_bridge = CvBridge()
        self.tf_listener = tf.TransformListener()

        # Subscribers
        self.image_sub = rospy.Subscriber("/front/left/image_raw", Image, self.image_callback)
        self.disparity_sub = rospy.Subscriber("/front/disparity", DisparityImage, self.disparity_callback)
        self.camera_info_sub = rospy.Subscriber("/front/left/camera_info", CameraInfo, self.camera_info_callback)
        
        # Publishers
        self.depth_image_pub = rospy.Publisher("/template/depth_image", Image, queue_size=10)
        self.match_pub = rospy.Publisher("/template/match_point", PoseStamped, queue_size=10)
        
        # Load template image from the scripts folder next to this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        template_path = os.path.join(script_dir, "4.png")  # Adjusted file path
        self.template_img = cv2.imread(template_path, 0)
        if self.template_img is None:
            rospy.logerr("Failed to load template image from {}".format(template_path))
            rospy.signal_shutdown("Failed to load template")

        self.scales = [0.3,0.4,0.5,0.6,0.75, 1.0, 1.25, 1.5]  # Scales for template matching
        
        self.camera_info = None
        self.T = 0.12  # Baseline distance, adjust according to your setup
        self.current_depth_image = None   
        self.has_published=False

    def camera_info_callback(self, msg):
        self.camera_info = msg


    def disparity_callback(self, data):
        
        try:
            disparity_image = self.cv_bridge.imgmsg_to_cv2(data.image, desired_encoding="32FC1")
            self.current_depth_image = self.camera_info.K[0] * self.T / (disparity_image + 1e-6)
            
            # Publish depth image
            depth_image_msg = self.cv_bridge.cv2_to_imgmsg(self.current_depth_image, encoding="32FC1")
            self.depth_image_pub.publish(depth_image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)


    def image_callback(self, data):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 初始化最佳匹配变量
        best_match_val = -1
        best_scale = None
        best_match_loc = None
        best_resized_template = None

        for scale in self.scales:
            # 调整模板大小，使用不同的插值方法根据缩放比例
            interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC
            resized_template = cv2.resize(self.template_img, None, fx=scale, fy=scale, interpolation=interpolation)
            
            # 执行模板匹配
            result = cv2.matchTemplate(gray_image, resized_template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            # 更新最佳匹配
            if max_val > best_match_val:
                best_match_val = max_val
                best_scale = scale
                best_match_loc = max_loc
                best_resized_template = resized_template
            if best_match_val > 0.7:  # 根据需要调整匹配阈值
                top_left = best_match_loc
                w, h = best_resized_template.shape[::-1]
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
                
                # 发布最佳匹配点
                match_point = PoseStamped()
                match_point.pose.position.x = top_left[0] + w / 2
                match_point.pose.position.y = top_left[1] + h / 2
                # self.match_pub.publish(match_point)
                
                if  self.current_depth_image is not None:
                    depth = self.current_depth_image[int(match_point.pose.position.y), int(match_point.pose.position.x)]
                    fx = self.camera_info.K[0]
                    fy = self.camera_info.K[4]
                    cx = self.camera_info.K[2]
                    cy = self.camera_info.K[5]

                    X = (match_point.pose.position.x - cx) * depth / fx
                    Y = (match_point.pose.position.y - cy) * depth / fy
                    Z = depth

                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = "front_camera_optical"
                    pose_stamped.pose.position.x = X
                    pose_stamped.pose.position.y = Y
                    pose_stamped.pose.position.z = Z
                    pose_stamped.pose.orientation.w = -3.14

                    # Transform the point from the camera frame to the map frame
                    pose_stamped_map = self.transform_pose_to_map_frame(pose_stamped, "front_camera_optical", "map")
                    if pose_stamped_map and not self.has_published:
                        self.match_pub.publish(pose_stamped_map)
                        # self.has_published = True  # 发布消息后设置标志位为True               
                    else:
                     rospy.loginfo("No match found.")


            cv2.imshow("Template Matching", cv_image)
            cv2.waitKey(1)

    def transform_pose_to_map_frame(self, pose_stamped, from_frame, to_frame):
        try:
            self.tf_listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            pose_transformed = self.tf_listener.transformPose(to_frame, pose_stamped)
            return pose_transformed
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to transform pose: %s", str(e))
            return None




if __name__ == '__main__':
    itm = ImprovedTemplateMatcher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()