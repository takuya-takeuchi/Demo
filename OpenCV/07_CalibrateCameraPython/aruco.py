#!/usr/bin/env python
# -*- coding: utf-8 -*
import sys
import os
import datetime
import argparse
import logging

import numpy as np
import cv2
from cv2 import aruco

import matplotlib.pyplot as plt
import moviepy.editor as mpy
from tqdm import tqdm
from mpl_toolkits.mplot3d import axes3d, Axes3D
from IPython.display import Image

logger = logging.getLogger(__name__)
handler = logging.StreamHandler(sys.stdout)
logger.setLevel(logging.INFO)
fmt = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s", "%Y-%m-%dT%H:%M:%S")
handler.setFormatter(fmt)
logger.addHandler(handler)

XYZ = []
RPY = []
V_x = []
V_y = []
V_z = []

def npy_to_gif(npy, filename):
    clip = mpy.ImageSequenceClip(list(npy), fps=10)
    clip.write_gif(filename)

def plot_all_frames(elev=90, azim=270, swapYZ=False):
    frames = []

    for t in tqdm(range(len(XYZ))):
        fig = plt.figure(figsize=(8,6))
        ax = Axes3D(fig)
        ax.view_init(elev=elev, azim=azim)

        if swapYZ:
            ax.set_xlim(-5, 5); ax.set_ylim(5, -5); ax.set_zlim(-5, 5)
            ax.set_xlabel("x"); ax.set_ylabel("z"); ax.set_zlabel("y")
        else:
            ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_zlim(-5, 5)
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")

        x, y, z = XYZ[t]
        ux, vx, wx = V_x[t]
        uy, vy, wy = V_y[t]
        uz, vz, wz = V_z[t]

        if swapYZ:
            z, y = y, z
            uz, uy = uy, uz
            vz, vy = vy, vz
            wz, wy = vy, wz

        # draw marker
        ax.scatter(0, 0, 0, color="k")
        if swapYZ:
            ax.quiver(0, 0, 0, 1, 0, 0, length=1, color="r")
            ax.quiver(0, 0, 0, 0, 0, 1, length=1, color="g")
            ax.quiver(0, 0, 0, 0, 1, 0, length=1, color="b")
            ax.plot([-1,1,1,-1,-1], [0,0,0,0,0], [-1,-1,1,1,-1], color="k", linestyle=":")
        else:
            ax.quiver(0, 0, 0, 1, 0, 0, length=1, color="r")
            ax.quiver(0, 0, 0, 0, 1, 0, length=1, color="g")
            ax.quiver(0, 0, 0, 0, 0, 1, length=1, color="b")
            ax.plot([-1,1,1,-1,-1], [-1,-1,1,1,-1], [0,0,0,0,0], color="k", linestyle=":")

        # draw camera
        if swapYZ:
            if t < 5:
                ax.quiver(x, y, z, ux, vx, wx, length=0.5, color="k")
                ax.quiver(x, y, z, uy, vy, wy, length=0.5, color="k")
                ax.quiver(x, y, z, uz, vz, wz, length=0.5, color="k")
            else:
                ax.quiver(x, y, z, ux, vx, wx, length=0.5, color="r")
                ax.quiver(x, y, z, uy, vy, wy, length=0.5, color="g")
                ax.quiver(x, y, z, uz, vz, wz, length=0.5, color="b")
        else:
            if t < 5:
                ax.quiver(x, y, z, ux, vx, wx, length=0.5, color="k")
                ax.quiver(x, y, z, uy, vy, wy, length=0.5, color="k")
                ax.quiver(x, y, z, uz, vz, wz, length=0.5, color="k")
            else:
                ax.quiver(x, y, z, ux, vx, wx, length=0.5, color="r")
                ax.quiver(x, y, z, uy, vy, wy, length=0.5, color="g")
                ax.quiver(x, y, z, uz, vz, wz, length=0.5, color="b")

        # save for animation
        fig.canvas.draw()
        frames.append(np.array(fig.canvas.renderer.buffer_rgba()))
        plt.close()

    return frames

def main(args):
    dictionary_name = args.dictionary
    marker_length = args.size
    output = args.output

    height = 1080
    width = 1920

    capture = cv2.VideoCapture(0, cv2.CAP_MSMF)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    dictionaries = {
        "DICT_4X4_50":aruco.DICT_4X4_50,
        "DICT_4X4_100":aruco.DICT_4X4_100,
        "DICT_4X4_250":aruco.DICT_4X4_250,
        "DICT_4X4_1000":aruco.DICT_4X4_1000,
        "DICT_5X5_50":aruco.DICT_5X5_50,
        "DICT_5X5_100":aruco.DICT_5X5_100,
        "DICT_5X5_250":aruco.DICT_5X5_250,
        "DICT_5X5_1000":aruco.DICT_5X5_1000,
        "DICT_6X6_50":aruco.DICT_6X6_50,
        "DICT_6X6_100":aruco.DICT_6X6_100,
        "DICT_6X6_250":aruco.DICT_6X6_250,
        "DICT_6X6_1000":aruco.DICT_6X6_1000,
        "DICT_7X7_50":aruco.DICT_7X7_50,
        "DICT_7X7_100":aruco.DICT_7X7_100,
        "DICT_7X7_250":aruco.DICT_7X7_250,
        "DICT_7X7_1000":aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL":aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5":aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9":aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10":aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11":aruco.DICT_APRILTAG_36h11
    }
    dictionary = aruco.getPredefinedDictionary(dictionaries[dictionary_name])

    camera_matrix = np.load("mtx.npy")
    distortion_coeff = np.load("dist.npy")

    newCamera, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeff, (width, height), 1)

    while True:
        now = datetime.datetime.now()
        ret, img = capture.read()

        undistorted = cv2.undistort(img, newCamera, distortion_coeff, newCamera);
        corners, ids, rejectedImgPoints = aruco.detectMarkers(undistorted, dictionary)

        if len(corners) > 0:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distortion_coeff)

            R = cv2.Rodrigues(rvec)[0]  # rotation vector -> rotation matrix
            R_T = R.T
            T = tvec[0].T

            xyz = np.dot(R_T, - T).squeeze()
            XYZ.append(xyz)
            logger.info(xyz)

            rpy = np.deg2rad(cv2.RQDecomp3x3(R_T)[0])
            RPY.append(rpy)
            # print(rpy)

            # rpy = cv2.decomposeProjectionMatrix(np.hstack([R_T, -T]))[6]  # no use [0~5]
            # rpy = np.deg2rad(rpy.squeeze())
            # print(rpy)

            # r = np.arctan2(-R_T[2][1], R_T[2][2])
            # p = np.arcsin(R_T[2][0])
            # y = np.arctan2(-R_T[1][0], R_T[0][0])
            # rpy = - np.array([r, p, y])
            # print(rpy)

            # from scipy.spatial.transform import Rotation
            # diff = eulerAnglesToRotationMatrix(rpy) - R_T
            # print(diff.astype(np.float16))
            # diff = Rotation.from_euler('xyz', rpy).as_matrix() - R_T
            # print(diff.astype(np.float16))
            
            V_x.append(np.dot(R_T, np.array([1, 0, 0])))
            V_y.append(np.dot(R_T, np.array([0, 1, 0])))
            V_z.append(np.dot(R_T, np.array([0, 0, 1])))

            aruco.drawDetectedMarkers(undistorted, corners, ids, (0, 255, 0))
            aruco.drawAxis(undistorted, camera_matrix, distortion_coeff, rvec, tvec, marker_length / 2)

        cv2.imshow('Find ArUco markers', undistorted)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

        if output:
            directory = "outputs"
            os.makedirs(directory, exist_ok=True)
            name = now.strftime('%Y%m%d%H%M%S%f')
            path = os.path.join(directory, f"{name}.jpg")
            cv2.imwrite(path, undistorted)

    cv2.destroyAllWindows()

def parse_command_args():
    parser = argparse.ArgumentParser(description='detect ArUco marker')
    parser.add_argument('-d', '--dictionary', type=str, required=True, help='ArUco predefine dictinary name')
    parser.add_argument('-s', '--size', type=float, required=True, help='Square size (m)')
    parser.add_argument('-o', '--output', action='store_true',help='Output captured image')

    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_command_args()
    main(args)

    frames = plot_all_frames(elev=105, azim=270)
    npy_to_gif(frames, "sample1.gif");
    frames = plot_all_frames(elev=165, azim=270)
    npy_to_gif(frames, "sample2.gif");
    frames = plot_all_frames(elev=None, azim=None, swapYZ=True)
    npy_to_gif(frames, "sample3.gif");
