#!/usr/bin/env python3
"""
HSV Color Testing Tool for Klotski Sense Node

This tool provides interactive HSV sliders to test color masking values
and outputs the results to CSV for analysis and parameter tuning.

Usage:
    ros2 run pkg_sense test_hsv_sense
"""

import rclpy
import cv2
import numpy as np
import csv
import os
from datetime import datetime
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from klotski_interfaces.srv import CaptureBoard

# Global variables for trackbars
hsv_values = {
    'red1_h_low': 0, 'red1_s_low': 100, 'red1_v_low': 80,
    'red1_h_high': 10, 'red1_s_high': 255, 'red1_v_high': 255,
    'red2_h_low': 170, 'red2_s_low': 100, 'red2_v_low': 80,
    'red2_h_high': 180, 'red2_s_high': 255, 'red2_v_high': 255,
    'yellow_h_low': 20, 'yellow_s_low': 80, 'yellow_v_low': 100,
    'yellow_h_high': 35, 'yellow_s_high': 255, 'yellow_v_high': 255,
    'green_h_low': 40, 'green_s_low': 50, 'green_v_low': 60,
    'green_h_high': 85, 'green_s_high': 255, 'green_v_high': 255,
    'blue_h_low': 95, 'blue_s_low': 60, 'blue_v_low': 60,
    'blue_h_high': 130, 'blue_s_high': 255, 'blue_v_high': 255,
    'grey_h_low': 0, 'grey_s_low': 0, 'grey_v_low': 50,
    'grey_h_high': 180, 'grey_s_high': 70, 'grey_v_high': 230,
}

current_image = None
test_results = []

class HSVTestSense(Node):
    def __init__(self):
        super().__init__('hsv_test_sense')
        
        # CV Bridge
        self.cv_bridge = CvBridge()
        self.cv_image = None
        
        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        
        # Publisher for UI events
        self.ui_pub = self.create_publisher(String, '/ui/events', 10)
        
        # Client for capture service
        self.capture_client = self.create_client(CaptureBoard, '/sense/capture_board')
        
        # Create output directory for results
        self.output_dir = "/tmp/hsv_test_results"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # CSV file for logging results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"{self.output_dir}/hsv_test_results_{timestamp}.csv"
        self.init_csv()
        
        self.get_logger().info(f"HSV Test Sense Node started. Results will be saved to: {self.csv_filename}")
        self.get_logger().info("Press 's' to save current HSV values to CSV")
        self.get_logger().info("Press 'c' to capture board with current settings")
        self.get_logger().info("Press 'q' to quit")
        
        # Setup GUI
        self.setup_gui()
        
    def init_csv(self):
        """Initialize CSV file with headers"""
        headers = [
            'timestamp', 'test_id', 'lighting_condition', 'notes',
            # Red 1
            'red1_h_low', 'red1_s_low', 'red1_v_low',
            'red1_h_high', 'red1_s_high', 'red1_v_high',
            # Red 2
            'red2_h_low', 'red2_s_low', 'red2_v_low',
            'red2_h_high', 'red2_s_high', 'red2_v_high',
            # Yellow
            'yellow_h_low', 'yellow_s_low', 'yellow_v_low',
            'yellow_h_high', 'yellow_s_high', 'yellow_v_high',
            # Green
            'green_h_low', 'green_s_low', 'green_v_low',
            'green_h_high', 'green_s_high', 'green_v_high',
            # Blue
            'blue_h_low', 'blue_s_low', 'blue_v_low',
            'blue_h_high', 'blue_s_high', 'blue_v_high',
            # Grey
            'grey_h_low', 'grey_s_low', 'grey_v_low',
            'grey_h_high', 'grey_s_high', 'grey_v_high',
            # Results
            'red_mask_area', 'yellow_mask_area', 'green_mask_area',
            'blue_mask_area', 'grey_mask_area', 'total_mask_area',
            'red_percentage', 'yellow_percentage', 'green_percentage',
            'blue_percentage', 'grey_percentage',
            'board_capture_success', 'piece_count_valid'
        ]
        
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(headers)
            
    def image_callback(self, msg):
        """Callback for camera image"""
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            global current_image
            current_image = self.cv_image.copy()
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")
    
    def setup_gui(self):
        """Setup OpenCV GUI with trackbars"""
        # Create windows
        cv2.namedWindow('HSV Controls', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Original Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Color Masks', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Combined Mask', cv2.WINDOW_NORMAL)
        
        # Create trackbars for each color
        colors = ['red1', 'red2', 'yellow', 'green', 'blue', 'grey']
        
        for color in colors:
            # Hue trackbars
            cv2.createTrackbar(f'{color}_h_low', 'HSV Controls', 
                             hsv_values[f'{color}_h_low'], 179, 
                             lambda val, c=color: self.update_hsv(f'{c}_h_low', val))
            cv2.createTrackbar(f'{color}_h_high', 'HSV Controls', 
                             hsv_values[f'{color}_h_high'], 179, 
                             lambda val, c=color: self.update_hsv(f'{c}_h_high', val))
            
            # Saturation trackbars
            cv2.createTrackbar(f'{color}_s_low', 'HSV Controls', 
                             hsv_values[f'{color}_s_low'], 255, 
                             lambda val, c=color: self.update_hsv(f'{c}_s_low', val))
            cv2.createTrackbar(f'{color}_s_high', 'HSV Controls', 
                             hsv_values[f'{color}_s_high'], 255, 
                             lambda val, c=color: self.update_hsv(f'{c}_s_high', val))
            
            # Value trackbars
            cv2.createTrackbar(f'{color}_v_low', 'HSV Controls', 
                             hsv_values[f'{color}_v_low'], 255, 
                             lambda val, c=color: self.update_hsv(f'{c}_v_low', val))
            cv2.createTrackbar(f'{color}_v_high', 'HSV Controls', 
                             hsv_values[f'{color}_v_high'], 255, 
                             lambda val, c=color: self.update_hsv(f'{c}_v_high', val))
    
    def update_hsv(self, param_name, value):
        """Update HSV value from trackbar"""
        global hsv_values
        hsv_values[param_name] = value
    
    def create_color_masks(self, image_bgr):
        """Create color masks based on current HSV values"""
        if image_bgr is None:
            return None
            
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        
        def create_mask(color_prefix):
            h_low = hsv_values[f'{color_prefix}_h_low']
            s_low = hsv_values[f'{color_prefix}_s_low']
            v_low = hsv_values[f'{color_prefix}_v_low']
            h_high = hsv_values[f'{color_prefix}_h_high']
            s_high = hsv_values[f'{color_prefix}_s_high']
            v_high = hsv_values[f'{color_prefix}_v_high']
            
            lower = np.array([h_low, s_low, v_low])
            upper = np.array([h_high, s_high, v_high])
            
            return cv2.inRange(hsv, lower, upper)
        
        # Create masks for each color
        masks = {}
        
        # Red mask (combine red1 and red2)
        red1_mask = create_mask('red1')
        red2_mask = create_mask('red2')
        masks['red'] = cv2.bitwise_or(red1_mask, red2_mask)
        
        # Other color masks
        masks['yellow'] = create_mask('yellow')
        masks['green'] = create_mask('green')
        masks['blue'] = create_mask('blue')
        masks['grey'] = create_mask('grey')
        
        # Apply morphological operations to clean up masks
        kernel = np.ones((5, 5), np.uint8)
        for color in masks:
            masks[color] = cv2.morphologyEx(masks[color], cv2.MORPH_OPEN, kernel)
            masks[color] = cv2.morphologyEx(masks[color], cv2.MORPH_CLOSE, kernel)
        
        return masks
    
    def calculate_mask_statistics(self, masks, image_shape):
        """Calculate statistics for each mask"""
        if masks is None:
            return {}
            
        total_pixels = image_shape[0] * image_shape[1]
        stats = {}
        
        total_mask_area = 0
        for color, mask in masks.items():
            area = np.sum(mask > 0)
            percentage = (area / total_pixels) * 100
            stats[color] = {
                'area': area,
                'percentage': percentage
            }
            total_mask_area += area
        
        stats['total'] = {
            'area': total_mask_area,
            'percentage': (total_mask_area / total_pixels) * 100
        }
        
        return stats
    
    def save_to_csv(self, masks=None, board_success=False, piece_count_valid=False, 
                    lighting_condition="", notes=""):
        """Save current HSV values and results to CSV"""
        timestamp = datetime.now().isoformat()
        test_id = len(test_results) + 1
        
        # Calculate mask statistics if masks provided
        mask_stats = {}
        if masks is not None and current_image is not None:
            mask_stats = self.calculate_mask_statistics(masks, current_image.shape)
        
        row_data = [
            timestamp, test_id, lighting_condition, notes,
            # HSV values
            *[hsv_values[key] for key in sorted(hsv_values.keys())],
            # Mask statistics
            mask_stats.get('red', {}).get('area', 0),
            mask_stats.get('yellow', {}).get('area', 0),
            mask_stats.get('green', {}).get('area', 0),
            mask_stats.get('blue', {}).get('area', 0),
            mask_stats.get('grey', {}).get('area', 0),
            mask_stats.get('total', {}).get('area', 0),
            mask_stats.get('red', {}).get('percentage', 0),
            mask_stats.get('yellow', {}).get('percentage', 0),
            mask_stats.get('green', {}).get('percentage', 0),
            mask_stats.get('blue', {}).get('percentage', 0),
            mask_stats.get('grey', {}).get('percentage', 0),
            board_success,
            piece_count_valid
        ]
        
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(row_data)
        
        test_results.append(row_data)
        self.get_logger().info(f"Saved test result #{test_id} to CSV")
        
        return test_id
    
    def capture_board_with_current_hsv(self):
        """Capture board using current HSV settings"""
        if not self.capture_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Capture board service not available")
            return False, False
        
        request = CaptureBoard.Request()
        
        try:
            future = self.capture_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f"Board capture result: {response.note}")
                return response.ok, True
            else:
                self.get_logger().error("Failed to get capture board response")
                return False, False
                
        except Exception as e:
            self.get_logger().error(f"Error calling capture board service: {e}")
            return False, False
    
    def run_gui_loop(self):
        """Main GUI loop"""
        self.get_logger().info("Starting GUI loop. Press ESC or 'q' to quit.")
        
        while rclpy.ok():
            # Process ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if current_image is not None:
                # Create and display masks
                masks = self.create_color_masks(current_image)
                
                if masks is not None:
                    # Display original image
                    cv2.imshow('Original Image', current_image)
                    
                    # Create color-coded mask display
                    mask_display = np.zeros_like(current_image)
                    colors_bgr = {
                        'red': (0, 0, 255),
                        'yellow': (0, 255, 255),
                        'green': (0, 255, 0),
                        'blue': (255, 0, 0),
                        'grey': (128, 128, 128)
                    }
                    
                    for color, mask in masks.items():
                        mask_display[mask > 0] = colors_bgr[color]
                    
                    cv2.imshow('Color Masks', mask_display)
                    
                    # Combine all masks
                    combined_mask = np.zeros_like(masks['red'])
                    for mask in masks.values():
                        combined_mask = cv2.bitwise_or(combined_mask, mask)
                    
                    cv2.imshow('Combined Mask', combined_mask)
                    
                    # Calculate and display statistics
                    stats = self.calculate_mask_statistics(masks, current_image.shape)
                    
                    # Create info display
                    info_img = np.zeros((400, 600, 3), dtype=np.uint8)
                    y_offset = 30
                    
                    for color, stat in stats.items():
                        if color != 'total':
                            text = f"{color}: {stat['area']} px ({stat['percentage']:.1f}%)"
                            cv2.putText(info_img, text, (10, y_offset), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                            y_offset += 30
                    
                    # Instructions
                    instructions = [
                        "Controls:",
                        "s - Save current values to CSV",
                        "c - Capture board with current HSV",
                        "q/ESC - Quit"
                    ]
                    
                    y_offset += 20
                    for instruction in instructions:
                        cv2.putText(info_img, instruction, (10, y_offset), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        y_offset += 20
                    
                    cv2.imshow('HSV Controls', info_img)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            elif key == ord('s'):  # Save current values
                if current_image is not None:
                    masks = self.create_color_masks(current_image)
                    lighting = input("Enter lighting condition (optional): ").strip()
                    notes = input("Enter notes (optional): ").strip()
                    test_id = self.save_to_csv(masks, lighting_condition=lighting, notes=notes)
                    
                    # Save mask images
                    if masks is not None:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        cv2.imwrite(f"{self.output_dir}/original_{test_id}_{timestamp}.png", current_image)
                        
                        for color, mask in masks.items():
                            cv2.imwrite(f"{self.output_dir}/mask_{color}_{test_id}_{timestamp}.png", mask)
                        
                        self.get_logger().info(f"Saved images for test #{test_id}")
                else:
                    self.get_logger().warn("No image available to save")
                    
            elif key == ord('c'):  # Capture board
                if current_image is not None:
                    masks = self.create_color_masks(current_image)
                    board_success, service_success = self.capture_board_with_current_hsv()
                    
                    if service_success:
                        lighting = input("Enter lighting condition for board capture (optional): ").strip()
                        notes = input("Enter notes for board capture (optional): ").strip()
                        self.save_to_csv(masks, board_success=board_success, 
                                       piece_count_valid=board_success,
                                       lighting_condition=lighting, notes=notes)
                else:
                    self.get_logger().warn("No image available for board capture")
        
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    
    try:
        node = HSVTestSense()
        node.run_gui_loop()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
