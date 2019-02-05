## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys

rgb_dir = './rgb/'
depth_dir = './depth/'

if not os.path.isdir(rgb_dir):
    os.mkdir(rgb_dir)

if not os.path.isdir(depth_dir):
    os.mkdir(depth_dir)


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(stream_type=rs.stream.depth, width=1280, height=720, format=rs.format.z16, framerate=30)
config.enable_stream(stream_type=rs.stream.color, width=1280, height=720, format=rs.format.bgr8, framerate=30)

# Start streaming
pipe_profile = pipeline.start(config)


frame_count = 0

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        frame_count+=1

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        # images = np.hstack((color_image, depth_colormap))


        # Save images
        if frame_count > 50 and frame_count%1 == 0:
            print(frame_count)

            depth_sensor = pipe_profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            with open ('./depth_scale.txt', 'a') as f:
                f.write(str(frame_count)+': '+ str(depth_scale)+ '\n')
            cv2.imwrite(rgb_dir+str(frame_count).rjust(5,'0')+'.png', color_image)
            cv2.imwrite(depth_dir+str(frame_count).rjust(5,'0')+'.png', depth_image)


        if frame_count == 200:
            # Stop streaming
            # pipeline.stop()
            sys.exit()
except KeyboardInterrupt:
    print('\nStopping...')

finally:
    # Stop streaming
    pipeline.stop()