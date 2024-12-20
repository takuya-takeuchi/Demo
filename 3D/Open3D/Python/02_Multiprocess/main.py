import argparse
import multiprocessing
from multiprocessing import Process
import os

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--eps", type=float, default=0.05)
    parser.add_argument("-p", "--min_points", type=int, default=10)
    return parser.parse_args()

def worker(name: str, eps: float, min_points: int):
    print(f"Worker {name} started, Process ID: {os.getpid()}")

    # load sample data
    pcd = o3d.io.read_point_cloud(o3d.data.PLYPointCloud().path)

    # apply dbscan clustering
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

    # get cluster num (-1 is noize)
    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print(f"Number of clusters detected: {num_clusters}")

    print(f"Worker {name} finished")

if __name__ == '__main__':
    # parse args
    args = get_args()
    eps        = args.eps
    min_points = args.min_points

    print("Arguments")
    print("               eps: {}".format(eps))
    print("        min_points: {}".format(min_points))

    # https://github.com/isl-org/Open3D/issues/1552
    # set 'forkserver' or 'spawn'
    if os.name == 'posix':
        multiprocessing.set_start_method('spawn')

    processes = []
    for i in range(5):
        p = Process(target=worker, args=(f"Task-{i}", eps, min_points,))
        processes.append(p)
        p.start()

    for p in processes:
        p.join()

    print("All processes completed.")
