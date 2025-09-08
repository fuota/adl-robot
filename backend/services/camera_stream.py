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
        
    def initialize(self) -> bool:
        """Initialize the RealSense camera pipeline"""
        try:
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
            return False
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get the latest color and depth frames"""
        if not self.is_streaming or not self.pipeline:
            return None, None
            
        try:
            with self.lock:
                # Wait for frames
                frames = self.pipeline.wait_for_frames()
                
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
    
    def stop(self):
        """Stop the camera streaming"""
        if self.pipeline and self.is_streaming:
            self.pipeline.stop()
            self.is_streaming = False
            print("RealSense camera stopped")
    
    def is_connected(self) -> bool:
        """Check if camera is connected and streaming"""
        return self.is_streaming and self.pipeline is not None
    
    def get_camera_info(self) -> dict:
        """Get camera information"""
        if not self.is_connected():
            return {"connected": False}
        
        try:
            # Get device info
            device = self.pipeline.get_active_profile().get_device()
            return {
                "connected": True,
                "device_name": device.get_info(rs.camera_info.name),
                "serial_number": device.get_info(rs.camera_info.serial_number),
                "firmware_version": device.get_info(rs.camera_info.firmware_version),
                "last_frame_time": self.last_frame_time
            }
        except Exception as e:
            return {"connected": False, "error": str(e)}

# Global camera instance
camera = RealSenseCamera()
