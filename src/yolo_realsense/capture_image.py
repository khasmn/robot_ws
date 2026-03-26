import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

def main():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    config = rs.config()
    
    # Configure the color stream
    # Note: 640x480 at 30 fps is a standard RealSense resolution, adjust if needed
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Create a folder to save images
    save_folder = "captured_images"
    os.makedirs(save_folder, exist_ok=True)

    print("-----------------------------------------")
    print(f"Saving images to: {os.path.abspath(save_folder)}")
    print("Press 'c' to capture an image.")
    print("Press 'q' or 'ESC' to quit.")
    print("-----------------------------------------")

    try:
        while True:
            # Wait for a coherent pair of frames (only color in this case)
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Show image
            cv2.imshow('RealSense Stream (Press c to capture, q to quit)', color_image)
            
            # Wait for key press (1ms delay)
            key = cv2.waitKey(1)
            
            if key & 0xFF == ord('c'):
                # Save the image when 'c' is pressed
                timestamp = int(time.time())
                filename = os.path.join(save_folder, f"image_{timestamp}.jpg")
                cv2.imwrite(filename, color_image)
                print(f"Captured: {filename}")
                
                # Flash the screen a bit to indicate capture
                flash_image = np.ones_like(color_image) * 255
                cv2.imshow('RealSense Stream (Press c to capture, q to quit)', flash_image)
                cv2.waitKey(50)
                
            elif key & 0xFF == ord('q') or key == 27:
                # Quit on 'q' or ESC
                print("Exiting...")
                break
                
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
