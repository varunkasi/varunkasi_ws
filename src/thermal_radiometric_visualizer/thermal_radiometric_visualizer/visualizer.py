import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from python_qt_binding.QtWidgets import QLabel, QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout
from python_qt_binding.QtGui import QPixmap, QImage, QFont, QPainter, QColor, QPen
from PyQt5.QtCore import Qt, pyqtSignal, QRect, QPoint
from PyQt5.QtWidgets import QDialog, QSlider, QSizePolicy, QProgressBar, QCheckBox, QMessageBox
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
from collections import deque
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Add rqt plugin specific imports
from rqt_gui_py.plugin import Plugin
from qt_gui.plugin import Plugin as QtPlugin
from python_qt_binding import loadUi

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('thermal_visualizer')

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Thermal Radiometric Visualizer')
parser.add_argument('--args', type=str, help='Path to the processed bag directory')
args, _ = parser.parse_known_args()

class TransparentOverlay(QWidget):
    pixel_clicked = pyqtSignal(int, int)  # Signal emitted when a pixel is clicked
    pixel_hovered = pyqtSignal(int, int)  # Signal emitted when mouse hovers over a pixel

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)
        self.image_width = 0
        self.image_height = 0
        self.image_rect = None
        self.last_clicked_pos = None
        self.hover_pos = None

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
                self.last_clicked_pos = (x, y)
                self.pixel_clicked.emit(x, y)
                self.update()

    def mouseMoveEvent(self, event):
        if self.image_rect is not None:
            img_pos = self._map_to_image(event.pos().x(), event.pos().y())
            if img_pos:
                x, y = img_pos
                self.hover_pos = (x, y)
                self.pixel_hovered.emit(x, y)
                self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if self.last_clicked_pos and self.image_rect is not None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            
            # Draw a cross at clicked position
            x, y = self.last_clicked_pos
            widget_x = self.image_rect.left() + (x / self.image_width) * self.image_rect.width()
            widget_y = self.image_rect.top() + (y / self.image_height) * self.image_rect.height()
            
            # Set pen properties for clicked point
            pen = QPen(QColor(255, 0, 0))
            pen.setWidth(2)
            painter.setPen(pen)
            
            # Draw cross at clicked point
            cross_size = 10
            painter.drawLine(widget_x - cross_size, widget_y, widget_x + cross_size, widget_y)
            painter.drawLine(widget_x, widget_y - cross_size, widget_x, widget_y + cross_size)
            
            # Draw circle around the cross
            painter.drawEllipse(QPoint(int(widget_x), int(widget_y)), cross_size, cross_size)

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

class SynchronizedMessage:
    """Class to hold synchronized message data"""
    def __init__(self, mono8_msg=None, calibrated_msg=None, raw_msg=None):
        self.mono8_msg = mono8_msg
        self.calibrated_msg = calibrated_msg
        self.raw_msg = raw_msg
        self.mono8_cv = None
        self.calibrated_cv = None
        self.raw_cv = None
        self.timestamp = None
        
    def is_complete(self):
        """Check if all required messages are present"""
        return (self.mono8_msg is not None or self.raw_msg is not None)
    
    def load_cv_images(self, bridge):
        """Convert ROS messages to OpenCV images"""
        try:
            if self.mono8_msg is not None and self.mono8_cv is None:
                self.mono8_cv = bridge.imgmsg_to_cv2(self.mono8_msg, desired_encoding='mono8')
                
            if self.calibrated_msg is not None and self.calibrated_cv is None:
                self.calibrated_cv = bridge.imgmsg_to_cv2(self.calibrated_msg, desired_encoding='passthrough')
                
            if self.raw_msg is not None and self.raw_cv is None and self.mono8_cv is None:
                # Only use raw as fallback if mono8 is not available
                raw_cv = bridge.imgmsg_to_cv2(self.raw_msg, desired_encoding='passthrough')
                
                # Convert to mono8 if needed
                if self.raw_msg.encoding == 'mono16' or self.raw_msg.encoding == '16UC1':
                    # Normalize to mono8 range
                    min_val = np.percentile(raw_cv, 1)
                    max_val = np.percentile(raw_cv, 99)
                    
                    if max_val > min_val:
                        self.mono8_cv = np.clip((raw_cv - min_val) * 255.0 / (max_val - min_val), 0, 255).astype(np.uint8)
                    else:
                        self.mono8_cv = np.zeros_like(raw_cv, dtype=np.uint8)
                else:
                    self.mono8_cv = raw_cv
                    
            return True
        except Exception as e:
            logger.error(f"Error loading CV images: {e}")
            return False
    
    def get_display_image(self):
        """Get the image to display (mono8)"""
        return self.mono8_cv
    
    def get_calibrated_image(self):
        """Get the calibrated temperature image"""
        return self.calibrated_cv
        
    def set_timestamp(self, timestamp):
        """Set timestamp from message header"""
        self.timestamp = timestamp

class ThermalRadiometricVisualizer(Plugin):
    def __init__(self, context):
        logger.info("Initializing ThermalRadiometricVisualizer plugin")
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
        self.robot_name = os.getenv("ROBOT_NAME", "default_robot")
        self.node.get_logger().info(f"Using robot name: {self.robot_name}")

        # Create subscriptions
        self.mono8_topic = f'/{self.robot_name}/hand/sensor/thermal/image_raw/mono8'
        self.calibrated_topic = f'/{self.robot_name}/hand/sensor/thermal/image_calibrated'
        self.raw_topic = f'/{self.robot_name}/hand/sensor/thermal/image_raw'
        
        # Log subscription information
        self.node.get_logger().info(f"Subscribing to mono8 topic: {self.mono8_topic}")
        self.node.get_logger().info(f"Subscribing to calibrated topic: {self.calibrated_topic}")
        self.node.get_logger().info(f"Subscribing to raw topic: {self.raw_topic}")
        
        # Configure QoS for better message delivery
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=30
        )
        
        # Add a message cache to store synchronized messages
        self.message_cache = {}  # Map of timestamp -> SynchronizedMessage
        self.message_queue = deque(maxlen=1000)  # Queue of processed messages
        self.current_message_index = -1
        self.current_timestamp = None
        self.frame_counter = 0
        self._is_stepping_frame = False
        self.synchronization_window = 0.01  # 10ms window for synchronization
        
        # Create subscriptions
        self.mono8_subscription = self.node.create_subscription(
            Image,
            self.mono8_topic,
            self.mono8_callback,
            qos
        )
        self.calibrated_subscription = self.node.create_subscription(
            Image,
            self.calibrated_topic,
            self.calibrated_callback,
            qos
        )
        
        # Add direct subscription to raw topic for compatibility
        self.node.get_logger().info(f"Adding direct subscription to exact bag topic: {self.raw_topic}")
        self.raw_subscription = self.node.create_subscription(
            Image,
            self.raw_topic,
            self.raw_image_callback,
            qos
        )

        self.bridge = CvBridge()
        
        # Create the UI
        self.create_ui()
        
        # State variables
        self.is_paused = False
        self.radiometric_mode = False
        self.calibrated_image = None
        self.current_temperature = None
        self.hover_temperature = None
        self.received_image_count = 0
        self.received_raw_count = 0
        self.received_calibrated_count = 0
        self.received_mono8_count = 0
        self.playback_speed = 1.0
        self.synced_messages_count = 0
        
        # Check for missing messages timer
        self.check_timer = self.node.create_timer(1.0, self.check_message_status)
        
        # Process image timer - handles displaying messages at a steady rate
        self.process_timer = self.node.create_timer(0.05, self.process_next_message)
        
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Start ROS spin in a separate thread
        self.spin_thread = Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # Use the bag file path passed as an argument
        self.bag_file = args.args
        if self.bag_file and os.path.exists(self.bag_file):
            self.node.get_logger().info(f"Found bag directory: {self.bag_file}")
            self.status_label.setText(f"Using bag directory: {self.bag_file}\nWaiting for messages...")
        else:
            self.node.get_logger().warning(f"Bag directory not found: {self.bag_file}")
            self.image_label.setText(f"Bag directory not found: {self.bag_file}\nListening for topics...")

    def create_ui(self):
        # Main layout
        main_layout = QVBoxLayout()
        
        # Image display area
        self.image_label = QLabel("Waiting for image data...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Create transparent overlay for pixel selection
        self.overlay = TransparentOverlay(self.image_label)
        self.overlay.setGeometry(0, 0, self.image_label.width(), self.image_label.height())
        self.overlay.pixel_clicked.connect(self.on_pixel_clicked)
        self.overlay.pixel_hovered.connect(self.on_pixel_hovered)
        
        # Info display area
        info_layout = QGridLayout()
        
        # Status label for frame information
        self.status_label = QLabel("Frame: 0 | Timestamp: 0.000")
        self.status_label.setAlignment(Qt.AlignLeft)
        
        # Temperature display
        self.temp_label = QLabel("Temperature: N/A")
        self.temp_label.setAlignment(Qt.AlignLeft)
        font = QFont()
        font.setBold(True)
        font.setPointSize(10)
        self.temp_label.setFont(font)
        
        # Hover temperature display
        self.hover_temp_label = QLabel("Hover: N/A")
        self.hover_temp_label.setAlignment(Qt.AlignLeft)
        
        # Mode label (Radiometric or Visual)
        self.mode_label = QLabel("Mode: Not initialized")
        self.mode_label.setAlignment(Qt.AlignLeft)
        
        # Add progress bar for synchronization
        self.sync_progress = QProgressBar()
        self.sync_progress.setRange(0, 100)
        self.sync_progress.setValue(0)
        self.sync_progress.setFormat("Synchronization: %v%")
        
        # Add message counts
        self.message_counts_label = QLabel("Messages: 0 raw, 0 mono8, 0 calibrated")
        self.message_counts_label.setAlignment(Qt.AlignLeft)
        
        # Add labels to info layout
        info_layout.addWidget(self.status_label, 0, 0)
        info_layout.addWidget(self.temp_label, 0, 1)
        info_layout.addWidget(self.hover_temp_label, 1, 1)
        info_layout.addWidget(self.mode_label, 1, 0)
        info_layout.addWidget(self.sync_progress, 2, 0, 1, 2)
        info_layout.addWidget(self.message_counts_label, 3, 0, 1, 2)
        
        # Controls layout
        controls_layout = QHBoxLayout()
        
        # Playback controls
        self.prev_button = QPushButton("<<")
        self.prev_button.clicked.connect(self.goto_prev_frame)
        self.prev_button.setFixedWidth(40)
        
        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        
        self.next_button = QPushButton(">>")
        self.next_button.clicked.connect(self.goto_next_frame)
        self.next_button.setFixedWidth(40)
        
        # Add radiometric mode checkbox
        self.radiometric_checkbox = QCheckBox("Radiometric Mode")
        self.radiometric_checkbox.setChecked(True)
        self.radiometric_checkbox.stateChanged.connect(self.toggle_radiometric_mode)
        
        # Speed control slider
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(10)   # 0.1x speed
        self.speed_slider.setMaximum(200)  # 2.0x speed
        self.speed_slider.setValue(100)    # 1.0x speed
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.valueChanged.connect(self.update_playback_speed)
        self.speed_slider.setFixedWidth(150)
        
        self.speed_label = QLabel("Speed: 1.0x")
        self.speed_label.setFixedWidth(80)
        
        # Add controls to layout
        controls_layout.addWidget(self.prev_button)
        controls_layout.addWidget(self.pause_button)
        controls_layout.addWidget(self.next_button)
        controls_layout.addWidget(self.radiometric_checkbox)
        controls_layout.addWidget(self.speed_slider)
        controls_layout.addWidget(self.speed_label)
        
        # Add all components to main layout
        main_layout.addWidget(self.image_label)
        main_layout.addLayout(info_layout)
        main_layout.addLayout(controls_layout)
        
        # Set the main layout
        self._widget.setLayout(main_layout)

    def toggle_radiometric_mode(self, state):
        """Toggle radiometric mode for temperature readings"""
        self.radiometric_mode = (state == Qt.Checked)
        self.mode_label.setText(f"Mode: {'Radiometric' if self.radiometric_mode else 'Visual only'}")
        
        # If turning off radiometric mode, clear temperature displays
        if not self.radiometric_mode:
            self.temp_label.setText("Temperature: N/A (Radiometric mode off)")
            self.hover_temp_label.setText("Hover: N/A")
        else:
            # If turning on, update with current selection if available
            if hasattr(self.overlay, 'last_clicked_pos') and self.overlay.last_clicked_pos and self.calibrated_image is not None:
                x, y = self.overlay.last_clicked_pos
                self.on_pixel_clicked(x, y)

    def update_playback_speed(self, value):
        """Update playback speed based on slider value"""
        self.playback_speed = value / 100.0
        self.speed_label.setText(f"Speed: {self.playback_speed:.1f}x")
        # Adjust timer interval to match desired playback speed
        self.process_timer.timer_period_ns = int(0.05 / self.playback_speed * 1e9)
        self.node.get_logger().info(f"Playback speed set to {self.playback_speed:.1f}x")

    def goto_prev_frame(self):
        """Go to previous frame in the message queue"""
        if len(self.message_queue) > 0:
            self.is_paused = True
            self.pause_button.setText("Resume")
            
            # Move to previous frame if possible
            if self.current_message_index > 0:
                self.current_message_index -= 1
                synced_msg = self.message_queue[self.current_message_index]
                self.display_message(synced_msg)
                self.node.get_logger().info(f"Displaying previous frame: {self.current_message_index}/{len(self.message_queue)}")
            else:
                self.show_notification("Already at first frame")

    def goto_next_frame(self):
        """Go to next frame in the message queue"""
        if len(self.message_queue) > 0:
            self.is_paused = True
            self.pause_button.setText("Resume")
            
            # Move to next frame if possible
            if self.current_message_index < len(self.message_queue) - 1:
                self.current_message_index += 1
                synced_msg = self.message_queue[self.current_message_index]
                self.display_message(synced_msg)
                self.node.get_logger().info(f"Displaying next frame: {self.current_message_index}/{len(self.message_queue)}")
            else:
                self.show_notification("Already at last frame")

    def show_notification(self, message):
        """Show a brief notification in the UI"""
        # Use status label for simple notifications
        current_text = self.status_label.text()
        self.status_label.setText(f"{current_text}\n{message}")
        
        # Schedule reset of the status message after 2 seconds
        self.node.create_timer(2.0, lambda: self.status_label.setText(current_text), oneshot=True)

    def raw_image_callback(self, msg):
        """Handle raw thermal image directly from the bag file"""
        if self.is_paused and not self._is_stepping_frame:
            return
            
        # Track received raw count
        self.received_raw_count += 1
        self.received_image_count += 1
        
        # Store in message cache
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        
        if timestamp not in self.message_cache:
            self.message_cache[timestamp] = SynchronizedMessage()
            
        self.message_cache[timestamp].raw_msg = msg
        self.message_cache[timestamp].set_timestamp(timestamp)
        
        # Only log occasionally to avoid flooding
        if self.received_raw_count % 50 == 0:
            self.node.get_logger().info(f"Received raw images: {self.received_raw_count}")
            
        # Check for matching messages with close timestamps
        self.find_and_process_sync_messages(timestamp)
        
        # Update message counts in UI
        self.update_message_counts()

    def update_message_counts(self):
        """Update the message counts display in the UI"""
        self.message_counts_label.setText(
            f"Messages: {self.received_raw_count} raw, {self.received_mono8_count} mono8, "
            f"{self.received_calibrated_count} calibrated, {self.synced_messages_count} synced"
        )
    
    def spin_ros(self):
        rclpy.spin(self.node)
    
    def toggle_pause(self):
        self.is_paused = not self.is_paused
        self.pause_button.setText("Resume" if self.is_paused else "Pause")

        # Update status label with additional information
        if self.current_timestamp:
            status = "Paused" if self.is_paused else "Playing"
            self.status_label.setText(f"{status} at Frame: {self.frame_counter} | Timestamp: {self.current_timestamp:.3f}")
            self.node.get_logger().info(f"{status} at frame {self.frame_counter}")

    def mono8_callback(self, msg):
        """Process the mono8 image message"""
        if self.is_paused and not self._is_stepping_frame:
            return
        
        # Track received mono8 count
        self.received_mono8_count += 1
        
        # Store in cache with timestamp as key
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        
        if timestamp not in self.message_cache:
            self.message_cache[timestamp] = SynchronizedMessage()
            
        self.message_cache[timestamp].mono8_msg = msg
        self.message_cache[timestamp].set_timestamp(timestamp)
        
        # Only log occasionally
        if self.received_mono8_count % 50 == 0:
            self.node.get_logger().info(f"Received mono8 images: {self.received_mono8_count}")
            
        # Find matching messages with close timestamps
        self.find_and_process_sync_messages(timestamp)
        
        # Update message counts in UI
        self.update_message_counts()

    def calibrated_callback(self, msg):
        """Process the calibrated image message"""
        if self.is_paused and not self._is_stepping_frame:
            return
        
        # Track received calibrated count
        self.received_calibrated_count += 1
        
        # Store calibrated image for temperature readings
        try:
            # Store in cache with timestamp as key
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            
            if timestamp not in self.message_cache:
                self.message_cache[timestamp] = SynchronizedMessage()
                
            self.message_cache[timestamp].calibrated_msg = msg
            self.message_cache[timestamp].set_timestamp(timestamp)
            
            # Only log occasionally
            if self.received_calibrated_count % 50 == 0:
                self.node.get_logger().info(f"Received calibrated images: {self.received_calibrated_count}")
                
            # Enable radiometric mode since we have calibrated data
            self.radiometric_mode = True
            self.radiometric_checkbox.setChecked(True)
            
            # Find matching messages with close timestamps
            self.find_and_process_sync_messages(timestamp)
            
            # Update message counts in UI
            self.update_message_counts()
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to process calibrated image: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())

    def find_and_process_sync_messages(self, timestamp):
        """Find messages with timestamps close to the given one and process them together"""
        # Look for messages with timestamps within the synchronization window
        for ts in list(self.message_cache.keys()):
            if abs(ts - timestamp) <= self.synchronization_window:
                # Found a potential match
                synced_msg = self.message_cache[ts]
                
                # Check if it has the other required messages
                raw_found = synced_msg.raw_msg is not None
                mono8_found = synced_msg.mono8_msg is not None
                calibrated_found = synced_msg.calibrated_msg is not None
                
                # For best results, we want all three message types
                # But at minimum, we need either mono8 or raw to display something
                if (raw_found or mono8_found):
                    # If mono8 is missing but we have raw, we could create mono8 here
                    if not mono8_found and raw_found:
                        # The message itself will handle the conversion in load_cv_images
                        pass
                    
                    # Process the synchronized message
                    if synced_msg.load_cv_images(self.bridge):
                        # Add to queue if not already added
                        # Use a flag to check if this timestamp is already in the queue
                        timestamp_exists = False
                        for msg in self.message_queue:
                            if abs(msg.timestamp - ts) < 0.001:  # Within 1ms
                                timestamp_exists = True
                                break
                        
                        if not timestamp_exists:
                            self.message_queue.append(synced_msg)
                            self.current_message_index = len(self.message_queue) - 1
                            self.synced_messages_count += 1
                            
                            # Update synchronization progress
                            sync_percent = min(100, int(self.synced_messages_count / max(1, self.received_image_count) * 100))
                            self.sync_progress.setValue(sync_percent)
                            
                            # If we have all three message types, highlight this in the UI
                            if raw_found and mono8_found and calibrated_found:
                                # Every 50th fully synchronized message, log it
                                if self.synced_messages_count % 50 == 0:
                                    self.node.get_logger().info(f"Added fully synchronized message {self.synced_messages_count} (t={ts:.3f})")
                    
                    # Clean up message cache to save memory
                    if len(self.message_cache) > 1000:  # Large cache size for better synchronization
                        self.cleanup_message_cache()
                
    def process_next_message(self):
        """Process and display the next message in the queue"""
        if not self.is_paused and self.current_message_index < len(self.message_queue) - 1:
            self.current_message_index += 1
            synced_msg = self.message_queue[self.current_message_index]
            self.display_message(synced_msg)
            
    def display_message(self, synced_msg):
        """Display the synchronized message data"""
        if synced_msg is None:
            return
            
        try:
            # Get the mono8 image for display
            cv_image = synced_msg.get_display_image()
            
            if cv_image is None:
                self.node.get_logger().warn("No display image available")
                return
                
            # Update the current timestamp and frame counter
            self.current_timestamp = synced_msg.timestamp
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
            
            # Store calibrated image reference
            self.calibrated_image = synced_msg.get_calibrated_image()
            
            # Update radiometric mode flag based on whether we have calibrated data
            if self.calibrated_image is not None:
                self.radiometric_mode = self.radiometric_checkbox.isChecked()
            else:
                self.radiometric_mode = False
                self.radiometric_checkbox.setChecked(False)
                
            # Update status label
            status = "Paused" if self.is_paused else "Playing"
            self.status_label.setText(f"{status} at Frame: {self.frame_counter} | Timestamp: {synced_msg.timestamp:.3f}")
            self.mode_label.setText(f"Mode: {'Radiometric' if self.radiometric_mode else 'Visual only'}")
            
            # Update temperature display if we have a clicked position
            if hasattr(self.overlay, 'last_clicked_pos') and self.overlay.last_clicked_pos and self.radiometric_mode:
                x, y = self.overlay.last_clicked_pos
                self.on_pixel_clicked(x, y)
                
            # Log occasionally
            if self.frame_counter % 100 == 0:
                self.node.get_logger().info(f"Displayed frame {self.frame_counter} (timestamp: {synced_msg.timestamp:.3f})")
                
        except Exception as e:
            self.node.get_logger().error(f"Error displaying message: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())

    def on_pixel_clicked(self, x, y):
        """Handle pixel click to display temperature"""
        if not self.radiometric_mode or self.calibrated_image is None:
            self.temp_label.setText("Temperature: Radiometric data not available")
            return
            
        temperature = self.get_temperature_at_pixel(x, y)
        
        if temperature is not None:
            self.current_temperature = temperature
            self.temp_label.setText(f"Temperature: {temperature:.2f}°C at ({x}, {y})")
            self.node.get_logger().info(f"Temperature at ({x}, {y}): {temperature:.2f}°C (Frame: {self.frame_counter})")
        else:
            self.temp_label.setText(f"Temperature: Invalid pixel ({x}, {y})")

    def on_pixel_hovered(self, x, y):
        """Display temperature on hover"""
        if not self.radiometric_mode or self.calibrated_image is None:
            return
            
        temperature = self.get_temperature_at_pixel(x, y)
        
        if temperature is not None:
            self.hover_temperature = temperature
            self.hover_temp_label.setText(f"Hover: {temperature:.2f}°C at ({x}, {y})")
        else:
            self.hover_temp_label.setText(f"Hover: Invalid pixel ({x}, {y})")

    def get_temperature_at_pixel(self, x, y):
        """Retrieve calibrated temperature at a given pixel"""
        if self.calibrated_image is None:
            return None
            
        if 0 <= y < self.calibrated_image.shape[0] and 0 <= x < self.calibrated_image.shape[1]:
            # Get the raw value from the calibrated image
            raw_value = self.calibrated_image[y, x]
            
            # Check if it's already in temperature scale (32FC1) or needs conversion
            if self.calibrated_image.dtype == np.float32:
                # Already in temperature scale
                temperature = raw_value
            else:
                # Get calibration parameters (should match thermal_calibrator.py)
                if self.robot_name == "spot1":
                    a = -5.74228169e-8
                    b = 6.43838380e-3
                    c = -90.2560653
                elif self.robot_name == "spot2":
                    a = 8.152e-6  # Fixed: 0000008152 -> 8.152e-6 (scientific notation)
                    b = -0.0294304271
                    c = 279.0969265845
                else:
                    a = -5.74228169e-8
                    b = 6.43838380e-3
                    c = -90.2560653
                
                # Apply quadratic calibration
                temperature = a * (raw_value**2) + b * raw_value + c
            
            return temperature
        
        return None

    def check_message_status(self):
        """Check synchronization status and display stats"""
        if not self.message_queue:
            return
            
        # Calculate statistics
        message_count = len(self.message_queue)
        mono8_count = sum(1 for m in self.message_queue if m.mono8_msg is not None)
        calibrated_count = sum(1 for m in self.message_queue if m.calibrated_msg is not None)
        raw_count = sum(1 for m in self.message_queue if m.raw_msg is not None)
        
        # Log statistics occasionally
        if self.frame_counter % 200 == 0:
            self.node.get_logger().info(f"Message queue: {message_count} messages "
                                   f"({mono8_count} mono8, {calibrated_count} calibrated, {raw_count} raw)")
        
        # Check for missing calibrated messages if radiometric mode is expected
        if calibrated_count < mono8_count * 0.5 and message_count > 10:
            self.node.get_logger().warn(f"Many missing calibrated messages. Check if thermal_calibrator is running.")
            
    def cleanup_message_cache(self):
        """Remove old messages from the cache to save memory"""
        # Keep a reasonable amount of messages in the cache
        max_cache_size = 1000  # Large cache size
        
        if len(self.message_cache) > max_cache_size:
            # Sort timestamps and remove oldest
            timestamps = sorted(self.message_cache.keys())
            for ts in timestamps[:len(timestamps) - max_cache_size]:
                del self.message_cache[ts]
                
            self.node.get_logger().debug(f"Cleaned up message cache. Now contains {len(self.message_cache)} timestamps")
    
    def shutdown_plugin(self):
        """Clean up resources when the plugin is shut down"""
        self.node.get_logger().info("ThermalRadiometricVisualizer shutting down")

def main(args=None):
    """Main function to run the visualizer as a standalone application"""
    rclpy.init(args=args)
    
    # Create Qt application
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    
    # Create a simple QWidget to host our plugin
    widget = QWidget()
    widget.setWindowTitle("Thermal Radiometric Visualizer")
    layout = QVBoxLayout()
    widget.setLayout(layout)
    
    # Create the visualizer (using a dummy context)
    class DummyContext:
        def add_widget(self, widget):
            layout.addWidget(widget)
    
    visualizer = ThermalRadiometricVisualizer(DummyContext())
    
    # Show the widget
    widget.show()
    widget.resize(800, 600)
    
    # Start Qt event loop
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()