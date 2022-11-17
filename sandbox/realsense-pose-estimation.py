'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import math
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time
import pyrealsense2 as rs


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)

        # If markers are detected
    if len(corners) > 0:
        
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            # 7.0 is marker size in CM
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 70.0, matrix_coefficients,
                                                                       distortion_coefficients)

            rotation_mat, _ = cv2.Rodrigues(rvec)

            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]]) 
    
            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

            thiscorner = corners[i]
            thiscorner = thiscorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = thiscorner
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # print(thiscorner)
            cv2.putText(frame, str(ids[i]),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            cv2.putText(frame, "{} {} {} {}".format(topLeft[0], topLeft[1], bottomRight[0], bottomRight[1]),(topLeft[0], topLeft[1] - 40), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            # cv2.putText(frame, "{:.2f} {:.2f} {:.2f}".format(math.degrees(rvec[0][0][0]), math.degrees(rvec[0][0][1]), math.degrees(rvec[0][0][2])),(topLeft[0], topLeft[1] - 80), cv2.FONT_HERSHEY_SIMPLEX,
            #     0.5, (0, 255, 0), 2)
            cv2.putText(frame, str(rotation_mat), (topLeft[0], topLeft[1] - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # cv2.putText(frame, "{} {} {}".format(math.degrees(rvec[0][0][0]), math.degrees(rvec[0][0][1]), math.degrees(rvec[0][0][2])),(topLeft[0], topLeft[1] - 80), cv2.FONT_HERSHEY_SIMPLEX,
            #     0.5, (0, 255, 0), 2)
            cv2.putText(frame, "{}".format(tvec),(topLeft[0], topLeft[1] - 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)


    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)
    print(k, d)

    # === Setup Realsense ===
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    print(intr.ppx, intr.ppy, intr.fx, intr.fy, intr.coeffs)

    k[0][0] = intr.fx
    k[1][1] = intr.fy
    k[0][2] = intr.ppx
    k[1][2] = intr.ppy
    d = np.asanyarray(intr.coeffs)
    print(k, d)
    
    # exit()
    print("Depth Scale is: " , depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 0.41 #1 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop

    while True:

        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        color_frame = frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        color_frame = np.asanyarray(color_frame.get_data())
        output = pose_estimation(color_frame, aruco_dict_type, k, d)

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()