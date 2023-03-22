import open3d as o3d
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.cluster import KMeans, DBSCAN, OPTICS
import matplotlib.pyplot as plt


# Load point cloud data
pcd = o3d.io.read_point_cloud("models/house.ply")

points = np.asarray(pcd.points).copy()

# Normalisation:
scaled_points = StandardScaler().fit_transform(points)
# Clustering:
model = DBSCAN(eps=0.15, min_samples=10)
model.fit(scaled_points)


labels = model.labels_
# Get the number of colors:
n_clusters = len(set(labels))

# Mapping the labels classes to a color map:
colors = plt.get_cmap("tab20")(labels / (n_clusters if n_clusters > 0 else 1))
# Attribute to noise the black color:
colors[labels < 0] = 0
# Update points colors:
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

# Display:
o3d.visualization.draw_geometries([pcd])
