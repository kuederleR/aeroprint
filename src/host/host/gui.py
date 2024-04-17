from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QGridLayout, QMainWindow, QApplication, QLabel
from PyQt5.QtCore import pyqtSignal, QByteArray
from PyQt5.QtGui import QPixmap, QImage
import sys
import threading
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy

import threading

class MainWindow(QMainWindow):

  def __init__(self, parent=None):
    super().__init__()
    self.setWindowTitle("ROS Image Viewer")
    self.setGeometry(100, 100, 800, 600)
    self.image_label = QLabel(self)  # QLabel to display the image
    self.setCentralWidget(self.image_label)

  def update_image_display(self, image_data):
    # Update the QLabel with the received image data
    # You can process the image_data here (e.g., convert to QPixmap)
    # For simplicity, assume image_data is a QPixmap

    self.image_label.setPixmap(image_data)

  def start_ros_thread(self, args=None):
    rclpy.init(args=args)
    ros_thread = threading.Thread(target=self.update_from_ros)
    ros_thread.start()

  def message(self, message):
    print(message)
    
  
  def update_from_ros(self):
    gui_node = GUINode(self)
    rclpy.spin(gui_node)
    gui_node.destroy_node()
    rclpy.shutdown()

class GUINode(Node):
  def __init__(self, window: MainWindow) -> None:
    self.qimage = QImage()
    self.window = window
    self.window.message("Node created")
    super().__init__("gui_node")
    self.bridge = CvBridge()
    self.get_logger().info("Starling data test node alive!")

      # Configure QoS profile for publishing and subscribing
    self.qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    self.qvio_sub = self.create_subscription(
        Image, 
        "/qvio_overlay", 
        self.qvio_callback, 
        qos_profile=qos_profile_sensor_data
    )

  def qvio_callback(self, image_msg: Image):
    img_format_table = {'rgb8': QImage.Format_RGB888, 'mono8': QImage.Format_Grayscale8, 'yuv422': QImage.Format_BGR30}
    try:
      self.format = img_format_table[image_msg.encoding]
      self.qimage = QImage(image_msg.data, image_msg.width, image_msg.height, self.format)
      self.pixmap = QPixmap.fromImage(self.qimage)
      self.window.update_image_display(self.pixmap)

    except Exception as e:
      self.window.message(e)




def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.start_ros_thread()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()