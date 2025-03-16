import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from python_qt_binding.QtWidgets import QLabel, QApplication, QPushButton, QVBoxLayout, QWidget
from python_qt_binding.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QDialog
import sys
import numpy as np
import cv2
import subprocess
import logging
import threading
from threading import Thread
import os
import signal
import argparse

# Add rqt plugin specific imports
from rqt_gui_py.plugin import Plugin
from qt_gui.plugin import Plugin as QtPlugin
from python_qt_binding import loadUi

# Add a debug statement to verify plugin loading
logging.basicConfig(level=logging.DEBUG)
logging.debug("ThermalRadiometricVisualizer plugin is being loaded.")

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Thermal Radiometric Visualizer')
parser.add_argument('--args', type=str, help='Path to the bag file')
args, _ = parser.parse_known_args()

class TransparentOverlay(QWidget):
    pixel_clicked = pyqtSignal(int, int)  # Signal emitted when a pixel is clicked

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)
        self.image_width = 0
        self.image_height = 0
        self.image_rect = None

    def update_image_dimensions(self, width, height, rect):
        self.image_width = width
        self.image_height = height
        self.image_rect = rect
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.image_rect is not None:
            img_pos = self._map_to_image(event.pos().x(), event.pos().y())
            if img_pos:
                x, y = img_pos
                self.pixel_clicked.emit(x, y)

    def _map_to_image(self, widget_x, widget_y):
        if self.image_rect is None or self.image_width == 0 or self.image_height == 0:
            return None
        if (widget_x < self.image_rect.left() or widget_x >= self.image_rect.right() or
            widget_y < self.image_rect.top() or widget_y >= self.image_rect.bottom()):
            return None
        norm_x = (widget_x - self.image_rect.left()) / self.image_rect.width()
        norm_y = (widget_y - self.image_rect.top()) / self.image_rect.height()
        img_x = int(norm_x * self.image_width)
        img_y = int(norm_y * self.image_height)
        return (img_x, img_y)

class ThermalRadiometricVisualizer(Plugin):
    def __init__(self, context):
        logging.debug("ThermalRadiometricVisualizer rqt plugin constructor called.")
        super().__init__(context)
        
        # Give QObjects reasonable names
        self.setObjectName('ThermalRadiometricVisualizer')

        # Create QWidget
        self._widget = QWidget()
        self._widget.setObjectName('ThermalRadiometricVisualizerUI')
        self._widget.setWindowTitle('Thermal Radiometric Visualizer')
        
        # Initialize ROS node only if not already initialized
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node('thermal_radiometric_visualizer')

        # Create subscriptions
        self.mono8_subscription = self.node.create_subscription(
            Image,
            f'/{os.getenv("ROBOT_NAME")}/hand/sensor/thermal/image_raw/mono8',
            self.mono8_callback,
            10
        )
        self.calibrated_subscription = self.node.create_subscription(
            Image,
            f'/{os.getenv("ROBOT_NAME")}/hand/sensor/thermal/image_calibrated',
            self.calibrated_callback,
            10
        )

        self.bridge = CvBridge()
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.resize(800, 600)

        self.overlay = TransparentOverlay(self.image_label)
        self.overlay.setGeometry(0, 0, self.image_label.width(), self.image_label.height())
        self.overlay.pixel_clicked.connect(self.on_pixel_clicked)

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        self.is_paused = False
        self.radiometric_mode = False
        self.calibrated_image = None
        self.bag_process = None
        self.bag_start_time = 0.0

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.pause_button)
        self._widget.setLayout(layout)

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Start ROS spin in a separate thread
        self.spin_thread = Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # Use the bag file path passed as an argument
        self.bag_file = args.args
        if not self.bag_file or not os.path.exists(self.bag_file):
            raise FileNotFoundError(f"Bag file not found: {self.bag_file}")
        self.bag_process = subprocess.Popen(['ros2', 'bag', 'play', self.bag_file])
        logging.debug(f"Started ros2 bag play process with PID: {self.bag_process.pid}")

    def spin_ros(self):
        rclpy.spin(self.node)
    
    def toggle_pause(self):
        if self.bag_process and self.bag_process.poll() is None:  # Check if the process is running
            self.is_paused = not self.is_paused
            self.pause_button.setText("Resume" if self.is_paused else "Pause")

            if self.is_paused:
                # Pause the bag process using SIGSTOP
                os.kill(self.bag_process.pid, signal.SIGSTOP)
            else:
                # Resume the bag process using SIGCONT
                os.kill(self.bag_process.pid, signal.SIGCONT)

    def get_current_bag_time(self):
        # Placeholder for logic to retrieve the current playback time from the bag file
        # This would require integration with the rosbag2 API or a similar mechanism
        return 0.0

    def mono8_callback(self, msg):
        if self.is_paused:
            return
        try:
            self.node.get_logger().info("mono8_callback triggered")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            height, width = cv_image.shape
            self.node.get_logger().info(f"Received image with dimensions: {width}x{height}")
            q_image = QImage(cv_image.data, width, height, QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(q_image)
            self.image_label.setPixmap(pixmap)
            self.overlay.update_image_dimensions(width, height, self.image_label.rect())
        except Exception as e:
            self.node.get_logger().error(f"Failed to process mono8 image: {e}")

    def calibrated_callback(self, msg):
        try:
            self.calibrated_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.node.get_logger().error(f"Failed to process calibrated image: {e}")

    def on_pixel_clicked(self, x, y):
        if not self.radiometric_mode or self.calibrated_image is None:
            self.node.get_logger().info("Radiometric mode is not enabled or calibrated image is unavailable.")
            return
        if 0 <= y < self.calibrated_image.shape[0] and 0 <= x < self.calibrated_image.shape[1]:
            temperature = self.calibrated_image[y, x]
            self.node.get_logger().info(f"Temperature at ({x}, {y}): {temperature}°C")
    
    def shutdown_plugin(self):
        if self.bag_process and self.bag_process.poll() is None:
            self.bag_process.terminate()
        logging.debug("ThermalRadiometricVisualizer is shutting down.")
        self.spin_thread.join(timeout=1.0)
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    from PyQt5.QtWidgets import QApplication  # Import here to avoid conflicts
    app = QApplication(sys.argv)
    visualizer_node = Node('thermal_radiometric_visualizer')
    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        visualizer_node.destroy_node()
        rclpy.shutdown()