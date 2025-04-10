# -*- coding: utf-8 -*-
"""
General Information
* Created by: 🦊 Florent Poux. 
* Copyright: Florent Poux.
* License: (c) learngeodata.eu
* Status: Hidden Perk

Dependencies:
* Anaconda or Miniconda
* An Anaconda new environment
* Python 3.10 (Open3D Compatible)
* Libraries as described

Tested on Windows 11 and MacOS

🎵 Note: Have fun with this Code Solution.
"""

import numpy as np
import open3d as o3d
import os
import argparse
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

#%% 1. Definition of the read ply function
def read_ply_with_labels(filename):
    """Read PLY file and extract points and labels"""
    # First try using Open3D to read the file structure
    pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(pcd.points)
    
    # Read raw data to extract labels
    with open(filename, 'rb') as f:
        ply_data = f.read()
    
    # Parse the header to find where the binary data begins
    header_end = ply_data.find(b'end_header\n') + len(b'end_header\n')
    
    # Check if there's a label property in the header
    if b'property uchar label' in ply_data[:header_end]:
        # Read the binary data using numpy
        dtype = np.dtype([
            ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
            ('label', 'u1')
        ])
        
        data = np.frombuffer(ply_data[header_end:], dtype=dtype)
        labels = data['label']
    else:
        # If no labels found, assign all points to 'undefined'
        labels = np.zeros(points.shape[0], dtype=np.uint8)
        print("No labels found in PLY file, assuming all points are undefined (0)")
    
    return points, labels

#%% 2. Definition of the visualization function
def visualize_point_cloud(points, labels, show_ui=True, save_image=None):
    """Visualize point cloud with color-coded labels"""
    # Define colors for each class
    color_map = {
        0: [0.5, 0.5, 0.5],  # Gray for undefined
        1: [0.8, 0.7, 0.5],  # Tan for horizontal planes
        2: [0.7, 0.2, 0.2],  # Red for vertical planes
        3: [0.2, 0.7, 0.2],  # Green for spheres
        4: [0.2, 0.2, 0.7]   # Blue for cubes
    }
    
    # Apply colors based on labels
    colors = np.zeros((points.shape[0], 3))
    for i, label in enumerate(labels):
        colors[i] = color_map.get(label, [0.5, 0.5, 0.5])
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    if show_ui:
        # Visualize
        print("Class labels:")
        print("- Gray: Undefined (0)")
        print("- Tan: Horizontal planes (1)")
        print("- Red: Vertical planes (2)")
        print("- Green: Spheres (3)")
        print("- Blue: Cubes (4)")
        
        o3d.visualization.draw_geometries([pcd], 
                                         window_name="Synthetic Room Visualization",
                                         width=1024, height=768)
    
    if save_image:
        # Create a visualization using matplotlib for better control
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Define label names for legend
        label_names = {
            0: "Undefined",
            1: "Horizontal Plane",
            2: "Vertical Plane",
            3: "Sphere",
            4: "Cube"
        }
        
        # Plot each class separately for legend
        unique_labels = np.unique(labels)
        for label in unique_labels:
            mask = labels == label
            if np.any(mask):
                ax.scatter(
                    points[mask, 0], 
                    points[mask, 1], 
                    points[mask, 2],
                    c=[color_map[label]],
                    label=label_names.get(label, f"Class {label}"),
                    alpha=0.7,
                    s=1
                )
        
        # Set axis limits and labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Synthetic Room Point Cloud')
        
        # Add legend
        ax.legend()
        
        # Adjust view
        ax.view_init(elev=30, azim=45)
        
        # Save figure
        plt.savefig(save_image, dpi=300, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved visualization to {save_image}")
        
#%% 3. Definition of the statistics exporter function
def generate_stats(labels):
    """Generate statistics about the point cloud labels"""
    unique_labels, counts = np.unique(labels, return_counts=True)
    total_points = len(labels)
    
    print("\nPoint Cloud Statistics:")
    print(f"Total points: {total_points}")
    print("Label distribution:")
    
    # Define label names
    label_names = {
        0: "Undefined",
        1: "Horizontal Plane",
        2: "Vertical Plane",
        3: "Sphere",
        4: "Cube"
    }
    
    # Print statistics for each label
    for label, count in zip(unique_labels, counts):
        percentage = (count / total_points) * 100
        name = label_names.get(label, f"Unknown ({label})")
        print(f"- {name}: {count} points ({percentage:.2f}%)")

