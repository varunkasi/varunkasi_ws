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

        # Get robot name from environment
        robot_name = os.getenv("ROBOT_NAME", "default_robot")
        self.node.get_logger().info(f"Using robot name: {robot_name}")

        # Create subscriptions
        self.mono8_topic = f'/{robot_name}/hand/sensor/thermal/image_raw/mono8'
        self.calibrated_topic = f'/{robot_name}/hand/sensor/thermal/image_calibrated'
        self.raw_topic = f'/{robot_name}/hand/sensor/thermal/image_raw'
        
        # Log subscription information
        self.node.get_logger().info(f"Subscribing to mono8 topic: {self.mono8_topic}")
        self.node.get_logger().info(f"Subscribing to calibrated topic: {self.calibrated_topic}")
        self.node.get_logger().info(f"Subscribing to raw topic: {self.raw_topic}")
        
        # Add a message cache to store synchronized messages
        self.message_cache = {}  # Map of timestamp -> (mono8_msg, calibrated_msg, raw_msg)
        self.current_timestamp = None
        self.frame_counter = 0
        
        # Add sequence tracking
        self.last_mono8_seq = -1
        self.last_calibrated_seq = -1
        self.last_raw_seq = -1
        
        # Create subscriptions
        self.mono8_subscription = self.node.create_subscription(
            Image,
            self.mono8_topic,
            self.mono8_callback,
            10
        )
        self.calibrated_subscription = self.node.create_subscription(
            Image,
            self.calibrated_topic,
            self.calibrated_callback,
            10
        )
        
        # Add direct subscription to raw topic for compatibility
        self.node.get_logger().info(f"Adding direct subscription to exact bag topic: {self.raw_topic}")
        self.raw_subscription = self.node.create_subscription(
            Image,
            self.raw_topic,
            self.raw_image_callback,
            10
        )

        self.bridge = CvBridge()
        self.image_label = QLabel("Waiting for image data...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.resize(800, 600)

        self.overlay = TransparentOverlay(self.image_label)
        self.overlay.setGeometry(0, 0, self.image_label.width(), self.image_label.height())
        self.overlay.pixel_clicked.connect(self.on_pixel_clicked)

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        
        # Add status label for frame information
        self.status_label = QLabel("Frame: 0 | Timestamp: 0.000")
        self.status_label.setAlignment(Qt.AlignCenter)
        
        # Add info about radiometric mode
        self.mode_label = QLabel("Mode: Not initialized")
        self.mode_label.setAlignment(Qt.AlignCenter)

        self.is_paused = False
        self.radiometric_mode = False
        self.calibrated_image = None
        self.bag_process = None
        self.bag_start_time = 0.0
        self.received_image_count = 0

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.status_label)
        layout.addWidget(self.mode_label)
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
        if self.bag_file and os.path.exists(self.bag_file):
            self.node.get_logger().info(f"Found bag file: {self.bag_file}")
            # Don't start the bag process here - it's already started in the tmux script
        else:
            self.node.get_logger().warning(f"Bag file not found: {self.bag_file}")
            self.image_label.setText(f"Bag file not found: {self.bag_file}\nListening for topics...")

    def raw_image_callback(self, msg):
        """Handle raw thermal image directly from the bag file"""
        if self.is_paused:
            return
            
        # Track sequence numbers
        if hasattr(msg.header, 'seq'):
            seq = msg.header.seq
            if seq != self.last_raw_seq + 1 and self.last_raw_seq != -1:
                self.node.get_logger().warn(f"Raw sequence gap: {self.last_raw_seq} -> {seq}")
            self.last_raw_seq = seq
        
        # Track received image count
        if not hasattr(self, 'received_image_count'):
            self.received_image_count = 0
        self.received_image_count += 1
        
        # Store in cache
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if timestamp not in self.message_cache:
            self.message_cache[timestamp] = [None, None, None]
        self.message_cache[timestamp][2] = msg
        
        # Only log occasionally to avoid flooding
        if self.received_image_count == 1 or self.received_image_count % 30 == 0:
            self.node.get_logger().info(f"Received raw image #{self.received_image_count} from topic: {msg.header.frame_id}")
            self.node.get_logger().info(f"Raw image encoding: {msg.encoding}")
        
        # If the converted topics aren't working, fallback to processing raw
        # Check if we haven't received any mono8 messages yet
        if self.frame_counter == 0 and self.received_image_count > 30:
            try:
                # For 16-bit thermal images, convert to 8-bit for display
                if msg.encoding == '16UC1' or msg.encoding == 'mono16':
                    # Convert to CV image
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
                    
                    # Store a copy of the original for radiometric use
                    if self.calibrated_image is None:
                        self.calibrated_image = cv_image.copy()
                        self.radiometric_mode = True
                    
                    # Normalize to 8-bit for display
                    min_val = np.percentile(cv_image, 1)  # Use 1st percentile instead of min to ignore outliers
                    max_val = np.percentile(cv_image, 99)  # Use 99th percentile instead of max
                    
                    if max_val > min_val:
                        # Create a new array for the normalized image
                        mono8_image = np.clip((cv_image - min_val) * 255.0 / (max_val - min_val), 0, 255).astype(np.uint8)
                    else:
                        mono8_image = np.zeros_like(cv_image, dtype=np.uint8)
                else:
                    # For mono8 or other formats
                    mono8_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                
                # Display the mono8 image
                height, width = mono8_image.shape
                
                # Ensure contiguous memory for QImage
                if not mono8_image.flags['C_CONTIGUOUS']:
                    mono8_image = np.ascontiguousarray(mono8_image)
                    
                # Create QImage with correct stride
                bytes_per_line = width  # For mono8 format
                q_image = QImage(mono8_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
                
                if q_image.isNull():
                    self.node.get_logger().error("Created QImage is null!")
                    return
                    
                # Convert to pixmap and display
                pixmap = QPixmap.fromImage(q_image)
                self.image_label.setPixmap(pixmap)
                
                # Update overlay dimensions
                self.overlay.update_image_dimensions(width, height, self.image_label.rect())
                
                # Update frame counter and labels
                self.frame_counter += 1
                self.current_timestamp = timestamp
                self.status_label.setText(f"Frame: {self.frame_counter} | Timestamp: {timestamp:.3f} (Raw fallback)")
                self.mode_label.setText(f"Mode: {'Radiometric' if self.radiometric_mode else 'Visual only'} (Raw fallback)")
                
                if self.frame_counter == 1:
                    self.node.get_logger().info(f"Using raw fallback for display. First frame: {width}x{height}")
                    
            except Exception as e:
                self.node.get_logger().error(f"Failed to process raw image: {e}")
                import traceback
                self.node.get_logger().error(traceback.format_exc())

    def spin_ros(self):
        rclpy.spin(self.node)
    
    def toggle_pause(self):
        if self.bag_process and self.bag_process.poll() is None:  # Check if the process is running
            self.is_paused = not self.is_paused
            self.pause_button.setText("Resume" if self.is_paused else "Pause")

            if self.is_paused:
                # Pause the bag process using SIGSTOP
                os.kill(self.bag_process.pid, signal.SIGSTOP)
                self.node.get_logger().info(f"Paused bag playback at frame {self.frame_counter}")
                
                # Update status label with additional information
                if self.current_timestamp:
                    self.status_label.setText(f"Paused at Frame: {self.frame_counter} | Timestamp: {self.current_timestamp:.3f}")
            else:
                # Resume the bag process using SIGCONT
                os.kill(self.bag_process.pid, signal.SIGCONT)
                self.node.get_logger().info(f"Resumed bag playback from frame {self.frame_counter}")

    def get_current_bag_time(self):
        # Placeholder for logic to retrieve the current playback time from the bag file
        # This would require integration with the rosbag2 API or a similar mechanism
        return 0.0

    def find_image_topics(self):
        """Find all available image topics."""
        try:
            # Use a shorter timeout to prevent hanging
            result = subprocess.run(
                ['ros2', 'topic', 'list', '-t'],  # List topics with types
                capture_output=True,
                text=True,
                check=True,
                timeout=2  # Short timeout
            )
            
            topics = result.stdout.splitlines()
            # Filter for image topics
            image_topics = [t.split(' ')[0] for t in topics if 'sensor_msgs/msg/Image' in t]
            
            self.node.get_logger().info(f"Found image topics: {image_topics}")
            
            if image_topics:
                for topic in image_topics:
                    if topic not in [self.mono8_topic] + self.fallback_topics:
                        self.node.get_logger().info(f"Found new image topic: {topic}")
                        self.fallback_topics.append(topic)
                        sub = self.node.create_subscription(
                            Image,
                            topic,
                            self.mono8_callback,
                            10
                        )
                        self.fallback_subscriptions.append(sub)
        except subprocess.TimeoutExpired:
            self.node.get_logger().warning("Topic listing timed out - this is expected")
        except Exception as e:
            self.node.get_logger().error(f"Error finding topics: {e}")

    def mono8_callback(self, msg):
        if self.is_paused:
            return
            
        # Track sequence numbers
        if hasattr(msg.header, 'seq'):
            seq = msg.header.seq
            if seq != self.last_mono8_seq + 1 and self.last_mono8_seq != -1:
                self.node.get_logger().warn(f"Mono8 sequence gap: {self.last_mono8_seq} -> {seq}")
            self.last_mono8_seq = seq
        
        # Store in cache with timestamp as key
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if timestamp not in self.message_cache:
            self.message_cache[timestamp] = [None, None, None]
        self.message_cache[timestamp][0] = msg
        
        # Process message
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            # If we have a calibrated message for this timestamp, use it
            if self.message_cache[timestamp][1] is not None:
                self.current_timestamp = timestamp
                self.frame_counter += 1
                
                # Display the image
                height, width = cv_image.shape
                
                # Ensure contiguous memory for QImage
                if not cv_image.flags['C_CONTIGUOUS']:
                    cv_image = np.ascontiguousarray(cv_image)
                    
                bytes_per_line = width
                q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
                
                if q_image.isNull():
                    self.node.get_logger().error("Created QImage is null!")
                    return
                    
                pixmap = QPixmap.fromImage(q_image)
                self.image_label.setPixmap(pixmap)
                
                # Update overlay dimensions
                self.overlay.update_image_dimensions(width, height, self.image_label.rect())
                
                # Update status label
                self.status_label.setText(f"Frame: {self.frame_counter} | Timestamp: {timestamp:.3f}")
                self.mode_label.setText(f"Mode: {'Radiometric' if self.radiometric_mode else 'Visual only'}")
                
                # Log occasionally
                if self.frame_counter == 1 or self.frame_counter % 30 == 0:
                    self.node.get_logger().info(f"Processed synchronized frame {self.frame_counter} (timestamp: {timestamp:.3f})")
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to process mono8 image: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
        
        # Limit cache size to avoid memory issues
        if len(self.message_cache) > 100:
            # Remove oldest entries
            oldest_timestamps = sorted(list(self.message_cache.keys()))[:max(0, len(self.message_cache)-100)]
            for ts in oldest_timestamps:
                del self.message_cache[ts]

    def calibrated_callback(self, msg):
        if self.is_paused:
            return
            
        # Track sequence numbers
        if hasattr(msg.header, 'seq'):
            seq = msg.header.seq
            if seq != self.last_calibrated_seq + 1 and self.last_calibrated_seq != -1:
                self.node.get_logger().warn(f"Calibrated sequence gap: {self.last_calibrated_seq} -> {seq}")
            self.last_calibrated_seq = seq
        
        try:
            # Store calibrated image for temperature readings
            self.calibrated_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.radiometric_mode = True
            
            # Store in cache with timestamp as key
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            if timestamp not in self.message_cache:
                self.message_cache[timestamp] = [None, None, None]
            self.message_cache[timestamp][1] = msg
            
            if self.message_cache[timestamp][0] is not None:  # If we already have the mono8 for this frame
                self.current_timestamp = timestamp
                # No need to update UI here, it will be done in mono8_callback
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to process calibrated image: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())

    def on_pixel_clicked(self, x, y):
        if not self.radiometric_mode or self.calibrated_image is None:
            self.node.get_logger().info("Radiometric mode is not enabled or calibrated image is unavailable.")
            return
            
        if 0 <= y < self.calibrated_image.shape[0] and 0 <= x < self.calibrated_image.shape[1]:
            # Get the raw value from the calibrated image
            raw_value = self.calibrated_image[y, x]
            
            # Check if it's already in temperature scale (32FC1) or needs conversion
            if self.calibrated_image.dtype == np.float32:
                # Already in temperature scale
                temperature = raw_value
            else:
                # Get calibration parameters (should match thermal_calibrator.py)
                robot_name = os.getenv("ROBOT_NAME", "spot1")
                
                # Default values from your calibration file
                if robot_name == "spot1":
                    a = -5.74228169e-8
                    b = 6.43838380e-3
                    c = -90.2560653
                elif robot_name == "spot2":
                    a = 8.152e-6  # Fixed: 0000008152 -> 8.152e-6 (scientific notation)
                    b = -0.0294304271
                    c = 279.0969265845
                else:
                    a = -5.74228169e-8
                    b = 6.43838380e-3
                    c = -90.2560653
                
                # Apply quadratic calibration
                temperature = a * (raw_value**2) + b * raw_value + c
            
            self.node.get_logger().info(f"Temperature at ({x}, {y}): {temperature:.2f}°C (Frame: {self.frame_counter})")
            
            # Show temperature in a tooltip or status bar
            self.status_label.setText(f"Frame: {self.frame_counter} | Timestamp: {self.current_timestamp:.3f} | Temp at ({x},{y}): {temperature:.2f}°C")
    
    def shutdown_plugin(self):
        if self.bag_process and self.bag_process.poll() is None:
            self.bag_process.terminate()
        logging.debug("ThermalRadiometricVisualizer is shutting down.")
        self.spin_thread.join(timeout=1.0)
        self.node.destroy_node()
        rclpy.shutdown()

    def check_available_topics(self):
        """Periodically check for available topics."""
        # This is a workaround since direct topic listing might hang
        self.node.get_logger().info("Checking for received images...")
        
        # Try to discover topics (with timeout protection)
        if hasattr(self, 'topic_discovery_count'):
            self.topic_discovery_count += 1
        else:
            self.topic_discovery_count = 0
            
        if self.topic_discovery_count % 5 == 0:  # Every 10 seconds (2 second timer × 5)
            self.find_image_topics()
        
        if self.last_image_time is not None:
            time_since_last = (self.node.get_clock().now() - self.last_image_time).nanoseconds / 1e9
            self.node.get_logger().info(f"Time since last image: {time_since_last:.2f} seconds")
            if time_since_last > 10.0:  # If no image for 10 seconds, update status
                self.image_label.setText(f"No images received for {time_since_last:.1f} seconds.\n"
                                        f"Total images received: {self.received_image_count}")

    def examine_bag_topics(self):
        """Examine topics in the bag file using ros2 bag info."""
        if not self.bag_file or not os.path.exists(self.bag_file):
            self.node.get_logger().error(f"Bag file not found: {self.bag_file}")
            return
        
        try:
            # Call ros2 bag info and capture output
            result = subprocess.run(
                ['ros2', 'bag', 'info', self.bag_file],
                capture_output=True,
                text=True,
                check=True,
                timeout=5  # Add timeout to prevent hanging
            )
            
            # Log the full bag info
            self.node.get_logger().info(f"Bag file info: {result.stdout}")
            
            # Parse the output to find topics
            topics = []
            in_topics_section = False
            for line in result.stdout.splitlines():
                if "Topics:" in line:
                    in_topics_section = True
                    continue
                if in_topics_section and line.strip():
                    if line.startswith(' '):  # Topic entries are indented
                        # Extract topic name (everything before the type)
                        parts = line.strip().split(' ', 1)
                        if parts:
                            topic_name = parts[0].strip()
                            topics.append(topic_name)
                    else:
                        in_topics_section = False
            
            self.node.get_logger().info(f"Found topics in bag: {topics}")
            
            # Look for thermal image topics
            image_topics = [t for t in topics if 'image' in t.lower() or 'thermal' in t.lower()]
            self.node.get_logger().info(f"Potential thermal image topics: {image_topics}")
            
            # Update fallback topics with the topics found in the bag
            if image_topics:
                self.fallback_topics.extend(image_topics)
                # Remove duplicates while preserving order
                self.fallback_topics = list(dict.fromkeys(self.fallback_topics))
                self.node.get_logger().info(f"Updated fallback topics: {self.fallback_topics}")
                
                # Subscribe to any new topics found
                for topic in image_topics:
                    if topic not in [self.mono8_topic] + [s for s in self.fallback_topics]:
                        self.node.get_logger().info(f"Adding new subscription to: {topic}")
                        sub = self.node.create_subscription(
                            Image,
                            topic,
                            self.mono8_callback,
                            10
                        )
                        self.fallback_subscriptions.append(sub)
            
            # Update the UI
            self.image_label.setText(f"Examining bag file: {self.bag_file}\n"
                                    f"Found {len(topics)} topics\n"
                                    f"Potential image topics: {', '.join(image_topics)}")
                                    
        except subprocess.TimeoutExpired:
            self.node.get_logger().error("Timed out trying to get bag info")
        except Exception as e:
            self.node.get_logger().error(f"Error examining bag: {e}")

    def refresh_topics(self):
        """Manually refresh topic subscriptions."""
        # Recreate the mono8 subscription
        self.node.destroy_subscription(self.mono8_subscription)
        self.mono8_subscription = self.node.create_subscription(
            Image,
            self.mono8_topic,
            self.mono8_callback,
            10
        )
        
        # Recreate fallback subscriptions
        for sub in self.fallback_subscriptions:
            self.node.destroy_subscription(sub)
        
        self.fallback_subscriptions = []
        for topic in self.fallback_topics:
            sub = self.node.create_subscription(
                Image,
                topic,
                self.mono8_callback,
                10
            )
            self.fallback_subscriptions.append(sub)
        
        self.node.get_logger().info("Topic subscriptions refreshed")
        self.image_label.setText("Topic subscriptions refreshed.\n"
                                f"Listening on main topic: {self.mono8_topic}\n"
                                f"And fallback topics: {', '.join(self.fallback_topics)}")

    def show_topic_info(self):
        """Display information about the topics we're listening to."""
        info_text = (f"Domain ID: {os.environ.get('ROS_DOMAIN_ID', 'not set')}\n"
                    f"Robot Name: {os.getenv('ROBOT_NAME', 'not set')}\n"
                    f"Main Topic: {self.mono8_topic}\n"
                    f"Fallback Topics: {', '.join(self.fallback_topics)}\n"
                    f"Received Images: {self.received_image_count}\n")
        
        if self.last_image_time:
            time_since = (self.node.get_clock().now() - self.last_image_time).nanoseconds / 1e9
            info_text += f"Time Since Last Image: {time_since:.2f} seconds"
        else:
            info_text += "No images received yet"
        
        self.image_label.setText(info_text)
        self.node.get_logger().info(info_text.replace('\n', ' | '))

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