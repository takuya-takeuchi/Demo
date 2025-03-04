import argparse

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--eps", type=float, default=0.05)
    parser.add_argument("-p", "--min_points", type=int, default=10)
    return parser.parse_args()

def run(eps: float, min_points: int):
    # load sample data
    pcd = o3d.io.read_point_cloud(o3d.data.PLYPointCloud().path)

    # apply dbscan clustering
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

    # get cluster num (-1 is noize)
    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print(f"Number of clusters detected: {num_clusters}")

    # colorize for each cluster
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))[:, :3]
    colors[labels < 0] = 0  # noize is black
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # visualize
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    # parse args
    args = get_args()
    eps        = args.eps
    min_points = args.min_points

    print("Arguments")
    print("               eps: {}".format(eps))
    print("        min_points: {}".format(min_points))

    run(eps, min_points)
