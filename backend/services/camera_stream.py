import cv2
import numpy as np
import pyrealsense2 as rs
from threading import Lock
import time
from typing import Optional, Tuple

class RealSenseCamera:
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.is_streaming = False
        self.lock = Lock()
        self.color_frame = None
        self.depth_frame = None
        self.last_frame_time = 0
        # Auto-initialize camera on instantiation
        self.initialize()
        
    def initialize(self) -> bool:
        """Initialize the RealSense camera pipeline"""
        try:
            # Clean up existing pipeline if any
            if self.pipeline and self.is_streaming:
                self.pipeline.stop()
            
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # Start streaming
            self.pipeline.start(self.config)
            self.is_streaming = True
            print("RealSense camera initialized successfully")
            return True
            
        except Exception as e:
            print(f"Failed to initialize RealSense camera: {e}")
            self.is_streaming = False
            return False
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get the latest color and depth frames"""
        # Try to reconnect if not streaming
        if not self.is_streaming or not self.pipeline:
            print("Camera not streaming, attempting to reconnect...")
            if self.initialize():
                print("Camera reconnected successfully")
            else:
                return None, None
            
        try:
            with self.lock:
                # Wait for frames with timeout
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                
                # Get color and depth frames
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    return None, None
                
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                self.color_frame = color_image
                self.depth_frame = depth_image
                self.last_frame_time = time.time()
                
                return color_image, depth_image
                
        except Exception as e:
            print(f"Error getting frames: {e}")
            # Mark as not streaming and try to reconnect next time
            self.is_streaming = False
            return None, None
    
    def get_color_frame(self) -> Optional[np.ndarray]:
        """Get the latest color frame"""
        color_frame, _ = self.get_frames()
        return color_frame
    
    def get_depth_frame(self) -> Optional[np.ndarray]:
        """Get the latest depth frame"""
        _, depth_frame = self.get_frames()
        return depth_frame
    
    def get_depth_colormap(self) -> Optional[np.ndarray]:
        """Get depth frame as a colormap for visualization"""
        depth_frame = self.get_depth_frame()
        if depth_frame is None:
            return None
        
        # Apply colormap to depth frame
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
        return depth_colormap
    
# Global camera instance
camera = RealSenseCamera()
