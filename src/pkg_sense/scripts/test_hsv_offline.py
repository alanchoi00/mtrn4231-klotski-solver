#!/usr/bin/env python3
"""
HSV Test Tool - Offline Demo

This script demonstrates the HSV testing tool using sample images
instead of live camera feed. Useful for development and testing.
"""

import cv2
import numpy as np
import os
import sys

# Add the pkg_sense directory to the path so we can import the test module
sys.path.append('/home/mtrn/mtrn4231-klotski-solver/src/pkg_sense')

def create_sample_klotski_image():
    """Create a synthetic Klotski board image for testing"""
    # Create a 640x480 image
    img = np.ones((480, 640, 3), dtype=np.uint8) * 128  # Gray background
    
    # Define colors in BGR
    colors = {
        'red': (0, 0, 255),
        'yellow': (0, 255, 255),
        'green': (0, 255, 0),
        'blue': (255, 0, 0),
        'grey': (128, 128, 128)
    }
    
    # Create a simple board layout
    cell_w, cell_h = 120, 80
    start_x, start_y = 100, 60
    
    # Sample board layout (4x5 grid)
    board_layout = [
        ['red', 'red', 'yellow', 'empty'],      # top row
        ['red', 'red', 'blue', 'empty'],
        ['green', 'green', 'blue', 'yellow'],
        ['grey', 'yellow', 'blue', 'yellow'],
        ['grey', 'empty', 'blue', 'empty']      # bottom row
    ]
    
    # Draw the cells
    for row in range(5):
        for col in range(4):
            cell_type = board_layout[row][col]
            if cell_type != 'empty':
                x1 = start_x + col * cell_w
                y1 = start_y + row * cell_h
                x2 = x1 + cell_w - 10  # Small gap between cells
                y2 = y1 + cell_h - 10
                
                color = colors.get(cell_type, (128, 128, 128))
                cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 2)
    
    # Add some ArUco-like markers at corners
    marker_size = 40
    marker_color = (0, 0, 0)
    
    # Top-left (0)
    cv2.rectangle(img, (50, 30), (50 + marker_size, 30 + marker_size), marker_color, -1)
    cv2.putText(img, '0', (55, 55), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Top-right (1)
    cv2.rectangle(img, (580, 30), (580 + marker_size, 30 + marker_size), marker_color, -1)
    cv2.putText(img, '1', (585, 55), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Bottom-left (2)
    cv2.rectangle(img, (50, 430), (50 + marker_size, 430 + marker_size), marker_color, -1)
    cv2.putText(img, '2', (55, 455), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Bottom-right (3)
    cv2.rectangle(img, (580, 430), (580 + marker_size, 430 + marker_size), marker_color, -1)
    cv2.putText(img, '3', (585, 455), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return img

def test_hsv_tool_offline():
    """Test the HSV tool with a sample image"""
    print("HSV Test Tool - Offline Demo")
    print("This demo shows how the HSV sliders work with a sample Klotski board.")
    print("Use the sliders to adjust color detection parameters.")
    print("Press 's' to save (will create CSV), 'q' to quit.")
    
    # Create sample image
    sample_image = create_sample_klotski_image()
    
    # Save sample image for reference
    os.makedirs('/tmp/hsv_test_results', exist_ok=True)
    cv2.imwrite('/tmp/hsv_test_results/sample_klotski_board.png', sample_image)
    print("Sample board saved to: /tmp/hsv_test_results/sample_klotski_board.png")
    
    # HSV values (same as in the main tool)
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
    
    def update_hsv(param_name, value):
        hsv_values[param_name] = value
        update_display()
    
    def create_color_masks(image_bgr):
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
        
        # Create masks
        masks = {}
        red1_mask = create_mask('red1')
        red2_mask = create_mask('red2')
        masks['red'] = cv2.bitwise_or(red1_mask, red2_mask)
        masks['yellow'] = create_mask('yellow')
        masks['green'] = create_mask('green')
        masks['blue'] = create_mask('blue')
        masks['grey'] = create_mask('grey')
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        for color in masks:
            masks[color] = cv2.morphologyEx(masks[color], cv2.MORPH_OPEN, kernel)
            masks[color] = cv2.morphologyEx(masks[color], cv2.MORPH_CLOSE, kernel)
        
        return masks
    
    def update_display():
        masks = create_color_masks(sample_image)
        
        # Create color-coded mask display
        mask_display = np.zeros_like(sample_image)
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
        
        # Calculate statistics
        total_pixels = sample_image.shape[0] * sample_image.shape[1]
        
        info_img = np.zeros((400, 600, 3), dtype=np.uint8)
        y_offset = 30
        
        for color, mask in masks.items():
            area = np.sum(mask > 0)
            percentage = (area / total_pixels) * 100
            text = f"{color}: {area} px ({percentage:.1f}%)"
            cv2.putText(info_img, text, (10, y_offset), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y_offset += 30
        
        # Instructions
        instructions = [
            "",
            "OFFLINE DEMO MODE",
            "",
            "Controls:",
            "s - Save current values (demo)",
            "q/ESC - Quit",
            "",
            "Adjust sliders to see changes",
            "in color detection"
        ]
        
        y_offset += 20
        for instruction in instructions:
            cv2.putText(info_img, instruction, (10, y_offset), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            y_offset += 20
        
        cv2.imshow('Info', info_img)
    
    # Create windows and trackbars
    cv2.namedWindow('Original Image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Color Masks', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Combined Mask', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Info', cv2.WINDOW_NORMAL)
    
    # Create trackbars
    colors = ['red1', 'red2', 'yellow', 'green', 'blue', 'grey']
    
    for color in colors:
        # Hue trackbars
        cv2.createTrackbar(f'{color}_h_low', 'Info', 
                         hsv_values[f'{color}_h_low'], 179, 
                         lambda val, c=color: update_hsv(f'{c}_h_low', val))
        cv2.createTrackbar(f'{color}_h_high', 'Info', 
                         hsv_values[f'{color}_h_high'], 179, 
                         lambda val, c=color: update_hsv(f'{c}_h_high', val))
        
        # Saturation trackbars
        cv2.createTrackbar(f'{color}_s_low', 'Info', 
                         hsv_values[f'{color}_s_low'], 255, 
                         lambda val, c=color: update_hsv(f'{c}_s_low', val))
        cv2.createTrackbar(f'{color}_s_high', 'Info', 
                         hsv_values[f'{color}_s_high'], 255, 
                         lambda val, c=color: update_hsv(f'{c}_s_high', val))
        
        # Value trackbars
        cv2.createTrackbar(f'{color}_v_low', 'Info', 
                         hsv_values[f'{color}_v_low'], 255, 
                         lambda val, c=color: update_hsv(f'{c}_v_low', val))
        cv2.createTrackbar(f'{color}_v_high', 'Info', 
                         hsv_values[f'{color}_v_high'], 255, 
                         lambda val, c=color: update_hsv(f'{c}_v_high', val))
    
    # Initial display
    cv2.imshow('Original Image', sample_image)
    update_display()
    
    print("\nGUI windows opened. Use trackbars to adjust HSV values.")
    
    # Main loop
    while True:
        key = cv2.waitKey(30) & 0xFF
        
        if key == ord('q') or key == 27:  # 'q' or ESC
            break
        elif key == ord('s'):  # Save demo
            print("\nDemo save - Current HSV values:")
            for param, value in sorted(hsv_values.items()):
                print(f"  {param}: {value}")
            print("In the real tool, these would be saved to CSV")
    
    cv2.destroyAllWindows()
    print("Demo completed!")

if __name__ == '__main__':
    test_hsv_tool_offline()
