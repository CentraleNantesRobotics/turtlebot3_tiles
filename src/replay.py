#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

only = ['full.png']


class ReplayNode(Node):

    def __init__(self):
        super().__init__('tiles_replay')

        self.images = self.load_images()

        if len(self.images) == 0:
            print('No images found')
            return

        self.publisher = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self.idx = 0

        dt = self.declare_parameter('period', 1.).value
        self.timer = self.create_timer(dt, self.publish)

    def load_images(self):

        import os
        import cv2

        # get images
        im_dir = os.path.join(get_package_share_directory('turtlebot3_tiles'), 'images')

        if not os.path.exists(im_dir):
            return []

        ext = ('jpg','JPG','png','PNG')
        files = only if only else [f for f in os.listdir(im_dir) if any(f.endswith(e) for e in ext)]

        bridge = CvBridge()
        msg = []

        for f in sorted(files):
            im = cv2.imread(os.path.join(im_dir, f))
            msg.append(bridge.cv2_to_compressed_imgmsg(im, 'jpeg'))

        return msg

    def publish(self):
        self.publisher.publish(self.images[self.idx])
        self.idx = (self.idx+1) % len(self.images)


rclpy.init()

node = ReplayNode()
rclpy.spin(node)
node.destroy_node()
