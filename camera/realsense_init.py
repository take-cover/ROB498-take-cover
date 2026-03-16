import pyrealsense2 as rs
import numpy as np
import cv2

# 1. Initialize the pipeline
pipeline = rs.pipeline()
config = rs.config()

# 2. Enable Fisheye streams (index 1 is left, index 2 is right)
config.enable_stream(rs.stream.fisheye, 1) 
config.enable_stream(rs.stream.fisheye, 2)

# 3. Start streaming
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        
        # Get the left and right fisheye frames
        f1 = frames.get_fisheye_frame(1)
        f2 = frames.get_fisheye_frame(2)
        
        if not f1 or not f2:
            continue

        # Convert to numpy arrays (Monochrome/Grayscale)
        left_image = np.asanyarray(f1.get_data())
        right_image = np.asanyarray(f2.get_data())

        # Stack them side-by-side to view both
        combined = np.hstack((left_image, right_image))

        cv2.imshow('T265 Fisheye Streams', combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()