import os
import subprocess
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import cv2
import numpy as np
from cv_bridge import CvBridge

def check_bag_and_convert_to_mp4(bag_name="aruco_debug_image_bag", output_video="output.mp4", fps=30):
    """
    Check for a ROS2 bag file and convert it to MP4 video.
    Only processes frames when they are readable/available.
    """
    bag_path = Path(bag_name)
    
    # Check if bag exists
    if not bag_path.exists():
        print(f"Error: Bag file '{bag_name}' not found")
        return False
    
    try:
        # Initialize bag reader
        storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
        converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Get bag info
        topic_types = reader.get_all_topics_and_types()
        
        # Find image topic
        image_topic = None
        for topic_type in topic_types:
            if "image" in topic_type.name.lower():
                image_topic = topic_type.name
                break
        
        if not image_topic:
            print("No image topic found in bag")
            return False
        
        frames = []
        frame_size = None
        
        # Extract frames from bag
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            if topic == image_topic:
                try:
                    # Convert ROS image message to OpenCV format
                    bridge = CvBridge()
                    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                    
                    if cv_image is not None and cv_image.size > 0:
                        frames.append(cv_image)
                        if frame_size is None:
                            frame_size = (cv_image.shape[1], cv_image.shape[0])
                        print(f"Frame captured: {len(frames)}")
                    
                except Exception as e:
                    print(f"Skipping unreadable frame: {e}")
                    continue
        
        if not frames or frame_size is None:
            print("No readable frames found")
            return False
        
        # Write frames to MP4
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_video, fourcc, fps, frame_size)
        
        for frame in frames:
            if frame.shape[1::-1] != frame_size:
                frame = cv2.resize(frame, frame_size)
            out.write(frame)
        
        out.release()
        print(f"Video saved to {output_video} ({len(frames)} frames)")
        return True
        
    except Exception as e:
        print(f"Error processing bag: {e}")
        return False

if __name__ == "__main__":
    check_bag_and_convert_to_mp4(bag_name="a", output_video="output.mp4")