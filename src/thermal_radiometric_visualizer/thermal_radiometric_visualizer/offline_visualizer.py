#!/usr/bin/env python3
"""
Offline Thermal Radiometric Visualizer

This module provides a standalone GUI for visualizing pre-processed thermal data.
It loads data processed by process_thermal_bag.py and provides the same
visualization and interaction capabilities as the online visualizer.
"""

import rclpy
import os
import sys
import numpy as np
import cv2
import json
import argparse
from pathlib import Path
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton,
                            QVBoxLayout, QHBoxLayout, QGridLayout, QWidget,
                            QSlider, QCheckBox, QMessageBox, QProgressBar,
                            QSizePolicy, QFileDialog, QSplashScreen)
from PyQt5.QtGui import QPixmap, QImage, QFont, QPainter, QColor, QPen
from PyQt5.QtCore import Qt, pyqtSignal, QRect, QPoint, QTimer
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('thermal_offline_visualizer')

class TransparentOverlay(QWidget):
    """Transparent overlay for pixel selection and temperature reading"""
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

class ThermalOfflineVisualizer(QMainWindow):
    """Offline visualizer for pre-processed thermal data"""
    
    def __init__(self, data_dir=None):
        super().__init__()
        
        # Store the data directory
        self.data_dir = data_dir
        
        # State variables
        self.is_paused = True
        self.radiometric_mode = True
        self.current_frame_index = 0
        self.frame_count = 0
        self.frames = {
            'raw': None,
            'calibrated': None,
            'mono8': None,
            'timestamps': None,
            'metadata': None
        }
        self.playback_speed = 1.0
        self.play_timer = QTimer()
        self.play_timer.timeout.connect(self.next_frame)
        self.play_timer.setInterval(100)  # 10 FPS default
        
        # Setup UI
        self.setup_ui()
        
        # Load data if provided
        if data_dir:
            self.load_data(data_dir)
        
    def setup_ui(self):
        """Set up the user interface"""
        self.setWindowTitle("Thermal Radiometric Visualizer - Offline Mode")
        self.setMinimumSize(800, 600)
        
        # Main widget and layout
        main_widget = QWidget()
        main_layout = QVBoxLayout(main_widget)
        
        # Image display area
        self.image_label = QLabel("Waiting for data to load...")
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
        self.status_label = QLabel("No data loaded")
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
        
        # Add data info
        self.data_info_label = QLabel("No data loaded")
        self.data_info_label.setAlignment(Qt.AlignLeft)
        
        # Add frame slider
        self.frame_slider = QSlider(Qt.Horizontal)
        self.frame_slider.setMinimum(0)
        self.frame_slider.setMaximum(100)  # Will be updated when data is loaded
        self.frame_slider.setValue(0)
        self.frame_slider.setTickPosition(QSlider.TicksBelow)
        self.frame_slider.setTickInterval(10)
        self.frame_slider.valueChanged.connect(self.slider_value_changed)
        
        # Add labels to info layout
        info_layout.addWidget(self.status_label, 0, 0)
        info_layout.addWidget(self.temp_label, 0, 1)
        info_layout.addWidget(self.hover_temp_label, 1, 1)
        info_layout.addWidget(self.mode_label, 1, 0)
        info_layout.addWidget(self.data_info_label, 2, 0, 1, 2)
        info_layout.addWidget(self.frame_slider, 3, 0, 1, 2)
        
        # Controls layout
        controls_layout = QHBoxLayout()
        
        # Load data button
        self.load_button = QPushButton("Load Data...")
        self.load_button.clicked.connect(self.open_data_directory)
        
        # Playback controls
        self.prev_button = QPushButton("<<")
        self.prev_button.clicked.connect(self.prev_frame)
        self.prev_button.setFixedWidth(40)
        
        self.play_pause_button = QPushButton("Play")
        self.play_pause_button.clicked.connect(self.toggle_playback)
        
        self.next_button = QPushButton(">>")
        self.next_button.clicked.connect(self.next_frame)
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
        controls_layout.addWidget(self.load_button)
        controls_layout.addWidget(self.prev_button)
        controls_layout.addWidget(self.play_pause_button)
        controls_layout.addWidget(self.next_button)
        controls_layout.addWidget(self.radiometric_checkbox)
        controls_layout.addWidget(self.speed_slider)
        controls_layout.addWidget(self.speed_label)
        
        # Add all components to main layout
        main_layout.addWidget(self.image_label)
        main_layout.addLayout(info_layout)
        main_layout.addLayout(controls_layout)
        
        # Set the central widget
        self.setCentralWidget(main_widget)
    
    def open_data_directory(self):
        """Open a dialog to select the data directory"""
        dir_path = QFileDialog.getExistingDirectory(
            self, "Select Processed Data Directory", "", QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.load_data(dir_path)
    
    def load_data(self, dir_path):
        """Load pre-processed thermal data from the given directory"""
        try:
            # Show loading message
            self.status_label.setText(f"Loading data from {dir_path}...")
            QApplication.processEvents()
            
            # Check if directory exists
            if not os.path.isdir(dir_path):
                QMessageBox.critical(self, "Error", f"Directory not found: {dir_path}")
                return
            
            # Check for required files
            required_files = [
                'raw_frames.npy',
                'calibrated_frames.npy',
                'mono8_frames.npy',
                'timestamps.npy',
                'metadata.json'
            ]
            
            for file in required_files:
                if not os.path.exists(os.path.join(dir_path, file)):
                    QMessageBox.critical(self, "Error", f"Missing required file: {file}")
                    return
            
            # Load metadata first to get information about the data
            with open(os.path.join(dir_path, 'metadata.json'), 'r') as f:
                self.frames['metadata'] = json.load(f)
            
            # Load frame data
            self.frames['raw'] = np.load(os.path.join(dir_path, 'raw_frames.npy'))
            self.frames['calibrated'] = np.load(os.path.join(dir_path, 'calibrated_frames.npy'))
            self.frames['mono8'] = np.load(os.path.join(dir_path, 'mono8_frames.npy'))
            self.frames['timestamps'] = np.load(os.path.join(dir_path, 'timestamps.npy'))
            
            # Update UI with data info
            self.frame_count = len(self.frames['timestamps'])
            
            self.data_info_label.setText(
                f"Data: {self.frames['metadata']['source_bag']} | "
                f"Robot: {self.frames['metadata']['robot_name']} | "
                f"Frames: {self.frame_count}"
            )
            
            # Update slider
            self.frame_slider.setMaximum(self.frame_count - 1)
            self.frame_slider.setValue(0)
            
            # Enable radiometric mode
            self.radiometric_mode = True
            self.radiometric_checkbox.setChecked(True)
            
            # Reset to first frame
            self.current_frame_index = 0
            self.update_display()
            
            # Show success message
            self.status_label.setText(f"Loaded {self.frame_count} frames from {dir_path}")
            
            # Store current data directory
            self.data_dir = dir_path
            
            logger.info(f"Successfully loaded data from {dir_path} with {self.frame_count} frames")
            
        except Exception as e:
            logger.error(f"Error loading data: {e}")
            QMessageBox.critical(self, "Error", f"Failed to load data: {str(e)}")
    
    def update_display(self):
        """Update the display with the current frame data"""
        if self.frames['mono8'] is None or self.current_frame_index >= self.frame_count:
            return
        
        try:
            # Get the current frame
            mono8_frame = self.frames['mono8'][self.current_frame_index]
            timestamp = self.frames['timestamps'][self.current_frame_index]
            
            # Convert to QImage and display
            height, width = mono8_frame.shape
            bytes_per_line = width
            
            # Ensure contiguous memory for QImage
            if not mono8_frame.flags['C_CONTIGUOUS']:
                mono8_frame = np.ascontiguousarray(mono8_frame)
                
            q_image = QImage(mono8_frame.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
            
            if q_image.isNull():
                logger.error("Created QImage is null!")
                return
                
            pixmap = QPixmap.fromImage(q_image)
            self.image_label.setPixmap(pixmap)
            
            # Update overlay dimensions
            self.overlay.update_image_dimensions(width, height, self.image_label.rect())
            
            # Update status
            status = "Paused" if self.is_paused else "Playing"
            self.status_label.setText(
                f"{status} | Frame: {self.current_frame_index + 1}/{self.frame_count} | "
                f"Timestamp: {timestamp:.3f}s"
            )
            
            # Update mode display
            self.mode_label.setText(f"Mode: {'Radiometric' if self.radiometric_mode else 'Visual only'}")
            
            # Update temperature display if there's a selected pixel
            if hasattr(self.overlay, 'last_clicked_pos') and self.overlay.last_clicked_pos:
                x, y = self.overlay.last_clicked_pos
                self.on_pixel_clicked(x, y)
                
        except Exception as e:
            logger.error(f"Error updating display: {e}")
    
    def toggle_playback(self):
        """Toggle between play and pause states"""
        self.is_paused = not self.is_paused
        
        if self.is_paused:
            self.play_timer.stop()
            self.play_pause_button.setText("Play")
        else:
            # Restart the timer with current speed
            interval = int(100 / self.playback_speed)  # Base 10 FPS
            self.play_timer.setInterval(interval)
            self.play_timer.start()
            self.play_pause_button.setText("Pause")
            
        # Update status label
        self.update_display()
    
    def next_frame(self):
        """Go to the next frame"""
        if self.frames['mono8'] is None:
            return
            
        if self.current_frame_index < self.frame_count - 1:
            self.current_frame_index += 1
            self.frame_slider.setValue(self.current_frame_index)
            self.update_display()
        elif not self.is_paused:
            # Loop back to the beginning if playing
            self.current_frame_index = 0
            self.frame_slider.setValue(self.current_frame_index)
            self.update_display()
    
    def prev_frame(self):
        """Go to the previous frame"""
        if self.frames['mono8'] is None:
            return
            
        if self.current_frame_index > 0:
            self.current_frame_index -= 1
            self.frame_slider.setValue(self.current_frame_index)
            self.update_display()
    
    def slider_value_changed(self, value):
        """Handle changes to the frame slider"""
        if self.frames['mono8'] is None or value >= self.frame_count:
            return
            
        self.current_frame_index = value
        self.update_display()
    
    def update_playback_speed(self, value):
        """Update playback speed based on slider value"""
        self.playback_speed = value / 100.0
        self.speed_label.setText(f"Speed: {self.playback_speed:.1f}x")
        
        # Update timer interval if playing
        if not self.is_paused:
            interval = int(100 / self.playback_speed)
            self.play_timer.setInterval(interval)
    
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
            if hasattr(self.overlay, 'last_clicked_pos') and self.overlay.last_clicked_pos:
                x, y = self.overlay.last_clicked_pos
                self.on_pixel_clicked(x, y)
    
    def on_pixel_clicked(self, x, y):
        """Handle pixel click to display temperature"""
        if not self.radiometric_mode or self.frames['calibrated'] is None:
            self.temp_label.setText("Temperature: Radiometric data not available")
            return
            
        if self.current_frame_index >= self.frame_count:
            return
            
        # Get the current calibrated frame
        calibrated_frame = self.frames['calibrated'][self.current_frame_index]
        
        if 0 <= y < calibrated_frame.shape[0] and 0 <= x < calibrated_frame.shape[1]:
            # Get the temperature value directly from the calibrated frame
            temperature = calibrated_frame[y, x]
            self.temp_label.setText(f"Temperature: {temperature:.2f}°C at ({x}, {y})")
            logger.info(f"Temperature at ({x}, {y}): {temperature:.2f}°C (Frame: {self.current_frame_index})")
        else:
            self.temp_label.setText(f"Temperature: Invalid pixel ({x}, {y})")
    
    def on_pixel_hovered(self, x, y):
        """Display temperature on hover"""
        if not self.radiometric_mode or self.frames['calibrated'] is None:
            return
            
        if self.current_frame_index >= self.frame_count:
            return
            
        # Get the current calibrated frame
        calibrated_frame = self.frames['calibrated'][self.current_frame_index]
        
        if 0 <= y < calibrated_frame.shape[0] and 0 <= x < calibrated_frame.shape[1]:
            # Get the temperature value directly from the calibrated frame
            temperature = calibrated_frame[y, x]
            self.hover_temp_label.setText(f"Hover: {temperature:.2f}°C at ({x}, {y})")
        else:
            self.hover_temp_label.setText(f"Hover: Invalid pixel ({x}, {y})")
    
    def closeEvent(self, event):
        """Clean up resources when closing the window"""
        # Stop the timer
        self.play_timer.stop()
        event.accept()

def main():
    """Main function to run the visualizer as a standalone application"""
    parser = argparse.ArgumentParser(description='Visualize pre-processed thermal data')
    parser.add_argument('--data_dir', help='Directory containing processed thermal data')
    
    args = parser.parse_args()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create splash screen with loading message
    if args.data_dir:
        splash = QSplashScreen()
        splash.setMinimumSize(400, 200)
        splash.showMessage("Loading thermal data...",
                         Qt.AlignHCenter | Qt.AlignVCenter)
        splash.show()
        app.processEvents()
    
    # Create visualizer
    visualizer = ThermalOfflineVisualizer(args.data_dir)
    visualizer.show()
    
    # Close splash if used
    if args.data_dir:
        splash.finish(visualizer)
    
    # Start the event loop
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()