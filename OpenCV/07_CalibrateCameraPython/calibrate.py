#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import datetime
import argparse
from logging import getLogger

import numpy as np
import cv2

logger = getLogger(__name__)

def main(args):
    pattern_size = (args.horizontal, args.vertical)
    square_size = args.size
    reference_img = args.count
    output = args.output

    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32 )
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    objpoints = []
    imgpoints = []

    height = 1080
    width = 1920

    capture = cv2.VideoCapture(0, cv2.CAP_MSMF)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    while len(objpoints) < reference_img:
        now = datetime.datetime.now()
        ret, img = capture.read()

        ret, corner = cv2.findChessboardCorners(img, pattern_size)
        cv2.drawChessboardCorners(img, pattern_size, corner, ret)

        cv2.imshow('Find Chessboard Corners', img)
        cv2.waitKey(10)

        if output:
            directory = "outputs"
            os.makedirs(directory, exist_ok=True)
            name = now.strftime('%Y%m%d%H%M%S%f')
            path = os.path.join(directory, f"{name}.jpg")
            cv2.imwrite(path, img)

        if not ret:
            logger.error("Failed to detect chessboard corners")
            continue

        if ret:
            imgpoints.append(corner.reshape(-1, 2))
            objpoints.append(pattern_points)
            logger.info(f"Succeeded to detect chessboard corners. [{len(imgpoints)}/{reference_img}]")
        
        if len(imgpoints) == reference_img:
            logger.info("Finish to collect chessboard corners")
            break


    logger.info("CalibrateCamera...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,
                                                       imgpoints,
                                                       (width, height),
                                                       None,
                                                       None)

    np.save("mtx", mtx)
    np.save("dist", dist.ravel())

    print(f"Overall RMS re-projection error: {ret}")
    print(f"        Camera Intrinsic Matrix:\n{mtx}")
    print(f"        Distortion Coefficients: {dist.ravel()}")

def parse_command_args():
    parser = argparse.ArgumentParser(description='calibrate camera', add_help=False)
    parser.add_argument('-h', '--horizontal', type=int, required=True, help='Number of inner corners (horizontal)')
    parser.add_argument('-v', '--vertical', type=int, required=True, help='Number of inner corners (vertical)')
    parser.add_argument('-s', '--size', type=float, required=True, help='Square size (cm)')
    parser.add_argument('-c', '--count', type=int, required=True, help='Reference count')
    parser.add_argument('-o', '--output', action='store_true',help='Output captured image')

    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_command_args()
    main(args)