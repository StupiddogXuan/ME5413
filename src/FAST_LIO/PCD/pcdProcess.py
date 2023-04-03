import numpy as np 
import open3d as o3d

file_data = np.fromfile("scans_2.pcd", dtype=np.float32)
points = file_data.reshape((-1, 5))[:, :4]
points = points[:,3]

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# o3d.visualization.draw_geometries([pcd])

# using RANSAC in open3d to filter the plane point
plane_model, inliers = pcd.segment_plane(distance_threshold=0.4, ransac_n=3, num_iterations=1000)
pcd_post = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([pcd_post])

# convert point clouds in open3d to numpy
points_processed = np.asarray(pcd_post.points)