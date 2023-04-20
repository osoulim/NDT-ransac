import open3d as o3d
import numpy as np

# Load point cloud data
pcd = o3d.io.read_point_cloud("models/fasad_A.ply")

pcd = pcd.voxel_down_sample(voxel_size=0.1)

# print(len(pcd.points))
# exit(0)

pcd_copy = o3d.geometry.PointCloud()
pcd_copy.points = o3d.utility.Vector3dVector(pcd.points)
pcd_copy.colors = o3d.utility.Vector3dVector([(.3, 0.3, 0.3) for _ in pcd_copy.points])


# Normalisation:
# Define RANSAC parameters
distance_threshold = 0.5  # distance threshold for plane model fitting
ransac_n = 4  # number of points required to fit a plane model
num_iterations = 5000  # number of RANSAC iterations

visualizer = o3d.visualization.VisualizerWithVertexSelection()

visualizer.create_window()
visualizer.add_geometry(pcd_copy)

visualizer.run()
visualizer.destroy_window()

points = visualizer.get_picked_points()
print(points[0].coord, pcd_copy.points[points[0].index])


points = list(map(lambda x: x.coord, points))
if len(points) == 0:
    exit(1)

selection_pcd = o3d.geometry.PointCloud()
selection_pcd.points = o3d.utility.Vector3dVector(points)
inliers = []
planes = []
while len(inliers) / len(selection_pcd.points) < .70:
    plane_model, inliers = selection_pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
    planes.append(plane_model)


plane_eq = planes[0][:-1]
plane_d = planes[0][-1]
colors = [(0.3, 0.3, 0.3) for _ in pcd_copy.points]
for index, point in enumerate(pcd_copy.points):

    if abs(np.dot(plane_eq, point) - plane_d) <= distance_threshold:
        colors[index] = (0, 0, 0.7)
pcd_copy.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries([pcd_copy])


# Run RANSAC to fit planes to the point cloud
# planes = []
# for i in range(100):
#     # Fit a plane to the sampled points
#     plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
#
#     # Check if the inlier ratio meets the threshold
#     inlier_ratio = len(inliers) / len(pcd.points)
#     print(len(inliers), len(pcd.points), inlier_ratio)
#     if inlier_ratio > inlier_threshold:
#         # Save the plane model if it's valid
#         planes.append(plane_model)
#         pcd = pcd.select_by_index(inliers, invert=True)
#
#     # Check if we have enough planes
#     if len(planes) >= 4:
#         break
#
# # Visualize the planes
# colors = [[0.7, 0.7, 0.7] for i in range(len(pcd_copy.points))]
# colormap = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1), (0, 1, 1)]
# for pindex, plane in enumerate(planes):
#     plane_eq = plane[:-1]
#     plane_d = plane[-1]
#     plane_points = []
#     for index, point in enumerate(pcd_copy.points):
#         if abs(np.dot(plane_eq, point) - plane_d) <= distance_threshold:
#             colors[index] = colormap[pindex]
# pcd_copy.colors = o3d.utility.Vector3dVector(colors)
#
# o3d.visualization.draw_geometries_with_vertex_selection([pcd_copy])
