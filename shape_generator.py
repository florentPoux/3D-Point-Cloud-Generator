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

#%% 1. Environment + setup

import numpy as np
import open3d as o3d

# import random
import os
# import math

import matplotlib.pyplot as plt
# from matplotlib.colors import ListedColormap

# from pathlib import Path

#%% 2. 3D Visualizer utility

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
                                         width=1980, height=1080)
    
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

#%% 3. Horizontal Shape Generation Function
def generate_horizontal_plane(center, width, length, point_density=100, noise_level=0.02):
    """Generate a horizontal plane with noise"""
    # Calculate number of points based on density and area
    num_points_x = int(np.sqrt(point_density * width))
    num_points_y = int(np.sqrt(point_density * length))
    
    # Create grid
    x = np.linspace(center[0] - width/2, center[0] + width/2, num_points_x)
    y = np.linspace(center[1] - length/2, center[1] + length/2, num_points_y)
    xx, yy = np.meshgrid(x, y)
    
    # Create flat points (z constant)
    points = np.zeros((num_points_x * num_points_y, 3))
    points[:, 0] = xx.flatten()
    points[:, 1] = yy.flatten()
    points[:, 2] = center[2]
    
    # Add random noise
    noise = np.random.normal(0, noise_level, size=points.shape)
    points += noise
    
    # Create labels (1 for horizontal plane)
    labels = np.ones(points.shape[0], dtype=int)
    
    return points, labels

# How can we have fun with this:
floor_center = [5, 5, 0]
floor_points, floor_labels = generate_horizontal_plane(floor_center, width=10, length=10)
visualize_point_cloud(floor_points, floor_labels, show_ui=True, save_image=None)

#%% 4. Vertical Shape Generation Function
def generate_vertical_plane(center, width, height, orientation=0, point_density=100, noise_level=0.02):
    """Generate a vertical plane with noise"""
    # Calculate number of points based on density and area
    num_points_x = int(np.sqrt(point_density * width))
    num_points_y = int(np.sqrt(point_density * height))
    
    # Create grid
    x = np.linspace(-width/2, width/2, num_points_x)
    y = np.linspace(-height/2, height/2, num_points_y)
    xx, yy = np.meshgrid(x, y)
    
    # Create flat points
    points = np.zeros((num_points_x * num_points_y, 3))
    
    # Apply orientation (rotation around Z axis)
    theta = np.radians(orientation)
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    
    # For vertical planes, we rotate around the vertical axis
    points[:, 0] = center[0] + xx.flatten() * cos_theta - 0 * sin_theta
    points[:, 1] = center[1] + xx.flatten() * sin_theta + 0 * cos_theta
    points[:, 2] = center[2] + yy.flatten()
    
    # Add random noise
    noise = np.random.normal(0, noise_level, size=points.shape)
    points += noise
    
    # Create labels (2 for vertical plane)
    labels = np.ones(points.shape[0], dtype=int) * 2
    
    return points, labels

# How can we have fun with this:
wall_center = [0, 5, 1.5]
wall_points, wall_labels = generate_vertical_plane(wall_center, width=10, height=3, orientation=90)
visualize_point_cloud(wall_points, wall_labels, show_ui=True, save_image=None)


#%% 5. Spherical Shape Generation Function
def generate_sphere(center, radius, point_density=100, noise_level=0.02):
    """Generate a sphere with noise"""
    # Calculate approximate number of points based on surface area and density
    surface_area = 4 * np.pi * radius**2
    num_points = int(surface_area * point_density)
    
    # Generate random points on a sphere using spherical coordinates
    theta = np.random.uniform(0, 2*np.pi, num_points)
    phi = np.random.uniform(0, np.pi, num_points)
    
    # Convert to Cartesian coordinates
    x = center[0] + radius * np.sin(phi) * np.cos(theta)
    y = center[1] + radius * np.sin(phi) * np.sin(theta)
    z = center[2] + radius * np.cos(phi)
    
    points = np.column_stack((x, y, z))
    
    # Add random noise
    noise = np.random.normal(0, noise_level, size=points.shape)
    points += noise
    
    # Create labels (3 for sphere)
    labels = np.ones(points.shape[0], dtype=int) * 3
    
    return points, labels

# How can we have fun with this:
sphere_center = [5, 5, 1.5]
sphere_points, sphere_labels = generate_sphere(sphere_center, radius=0.5, point_density=1000)
visualize_point_cloud(sphere_points, sphere_labels, show_ui=True, save_image=None)


#%% 6. Cube Shape Generation Function

def generate_cube(center, size, point_density=100, noise_level=0.02):
    """Generate a cube with noise"""
    half_size = size / 2
    
    # Calculate number of points per face based on density and area
    points_per_face = int(point_density * size**2)
    
    # Create empty arrays for points and labels
    points = np.zeros((points_per_face * 6, 3))
    labels = np.ones(points_per_face * 6, dtype=int) * 4
    
    # Generate points for each face
    for i, (axis, direction) in enumerate([
        (0, 1), (0, -1),  # X+, X-
        (1, 1), (1, -1),  # Y+, Y-
        (2, 1), (2, -1)   # Z+, Z-
    ]):
        # Calculate points per side
        # side_points = int(np.sqrt(points_per_face))
        
        # Calculate start and end index for this face
        start_idx = i * points_per_face
        end_idx = start_idx + points_per_face
        
        # Create grid for this face
        grid = np.zeros((points_per_face, 3))
        
        # Calculate the coordinates for the two axes that aren't constant
        axes = [0, 1, 2]
        axes.remove(axis)
        axis1, axis2 = axes
        
        # Create a grid for the face
        coords1 = np.random.uniform(-half_size, half_size, points_per_face)
        coords2 = np.random.uniform(-half_size, half_size, points_per_face)
        
        # Assign coordinates
        grid[:, axis] = direction * half_size
        grid[:, axis1] = coords1
        grid[:, axis2] = coords2
        
        # Add points to array with center offset
        points[start_idx:end_idx] = grid + center
    
    # Add random noise
    noise = np.random.normal(0, noise_level, size=points.shape)
    points += noise
    
    return points, labels

# How can we have fun with this:
cube_center = [3, 4, 1]
cube_points, cube_labels = generate_cube(cube_center, size=0.8, point_density=1000)
visualize_point_cloud(cube_points, cube_labels, show_ui=True, save_image=None)


#%% 7. Random Noise Generation

def generate_random_noise(room_min, room_max, num_points=1000, label=0):
    """Generate random noise points throughout the room"""
    # Create random points within the room bounds
    x = np.random.uniform(room_min[0], room_max[0], num_points)
    y = np.random.uniform(room_min[1], room_max[1], num_points)
    z = np.random.uniform(room_min[2], room_max[2], num_points)
    
    points = np.column_stack((x, y, z))
    
    # Create labels (0 for undefined/noise)
    labels = np.ones(num_points, dtype=int) * label
    
    return points, labels

# How can we have fun with this:
room_min = [0, 0, 0]
room_max = [10, 8, 3]
noise_points, noise_labels = generate_random_noise(room_min, room_max, num_points=500)
visualize_point_cloud(noise_points, noise_labels, show_ui=True, save_image=None)

#%% 8. Generating simulated training

def generate_room(
    room_size=(10, 8, 3),
    num_spheres=2,
    num_cubes=3,
    base_point_density=50,
    noise_percentage=0.1,
    noise_level=0.02
):
    """Generate a complete room with shapes and noise"""
    all_points = []
    all_labels = []
    
    # Determine room boundaries
    room_min = np.array([0, 0, 0])
    room_max = np.array(room_size)
    room_center = (room_min + room_max) / 2
    
    # Add room elements with some variation
    
    # Floor (horizontal plane at bottom)
    floor_height = room_min[2] + np.random.uniform(0, 0.1)
    floor_center = [room_center[0], room_center[1], floor_height]
    points, labels = generate_horizontal_plane(
        floor_center, 
        room_size[0] * np.random.uniform(0.95, 1.0),
        room_size[1] * np.random.uniform(0.95, 1.0), 
        point_density=base_point_density * np.random.uniform(0.8, 1.2),
        noise_level=noise_level
    )
    all_points.append(points)
    all_labels.append(labels)
    
    # Ceiling (horizontal plane at top)
    ceiling_height = room_max[2] - np.random.uniform(0, 0.1)
    ceiling_center = [room_center[0], room_center[1], ceiling_height]
    points, labels = generate_horizontal_plane(
        ceiling_center, 
        room_size[0] * np.random.uniform(0.95, 1.0), 
        room_size[1] * np.random.uniform(0.95, 1.0), 
        point_density=base_point_density * np.random.uniform(0.8, 1.2),
        noise_level=noise_level
    )
    all_points.append(points)
    all_labels.append(labels)
    
    # Walls (vertical planes)
    # Wall 1 (back)
    back_wall_center = [room_center[0], room_min[1] + np.random.uniform(0, 0.1), room_center[2]]
    points, labels = generate_vertical_plane(
        back_wall_center, 
        room_size[0] * np.random.uniform(0.95, 1.0), 
        room_size[2] * np.random.uniform(0.9, 1.0), 
        orientation=0,
        point_density=base_point_density * np.random.uniform(0.8, 1.2),
        noise_level=noise_level
    )
    all_points.append(points)
    all_labels.append(labels)
    
    # Wall 2 (front)
    front_wall_center = [room_center[0], room_max[1] - np.random.uniform(0, 0.1), room_center[2]]
    points, labels = generate_vertical_plane(
        front_wall_center, 
        room_size[0] * np.random.uniform(0.95, 1.0), 
        room_size[2] * np.random.uniform(0.9, 1.0), 
        orientation=0,
        point_density=base_point_density * np.random.uniform(0.8, 1.2),
        noise_level=noise_level
    )
    all_points.append(points)
    all_labels.append(labels)
    
    # Wall 3 (left)
    left_wall_center = [room_min[0] + np.random.uniform(0, 0.1), room_center[1], room_center[2]]
    points, labels = generate_vertical_plane(
        left_wall_center, 
        room_size[1] * np.random.uniform(0.95, 1.0), 
        room_size[2] * np.random.uniform(0.9, 1.0), 
        orientation=90,
        point_density=base_point_density * np.random.uniform(0.8, 1.2),
        noise_level=noise_level
    )
    all_points.append(points)
    all_labels.append(labels)
    
    # Wall 4 (right)
    right_wall_center = [room_max[0] - np.random.uniform(0, 0.1), room_center[1], room_center[2]]
    points, labels = generate_vertical_plane(
        right_wall_center, 
        room_size[1] * np.random.uniform(0.95, 1.0), 
        room_size[2] * np.random.uniform(0.9, 1.0), 
        orientation=90,
        point_density=base_point_density * np.random.uniform(0.8, 1.2),
        noise_level=noise_level
    )
    all_points.append(points)
    all_labels.append(labels)
    
    # Add random spheres
    for _ in range(num_spheres):
        # Random position for sphere (keep away from walls)
        margin = 0.5  # Minimum distance from walls
        sphere_center = [
            np.random.uniform(room_min[0] + margin, room_max[0] - margin),
            np.random.uniform(room_min[1] + margin, room_max[1] - margin),
            np.random.uniform(room_min[2] + margin, room_max[2] - margin)
        ]
        
        # Random radius (not too big)
        radius = np.random.uniform(0.2, min(room_size) / 5)
        
        points, labels = generate_sphere(
            sphere_center, 
            radius, 
            point_density=base_point_density * np.random.uniform(0.8, 1.5),
            noise_level=noise_level * np.random.uniform(0.8, 1.2)
        )
        all_points.append(points)
        all_labels.append(labels)
    
    # Add random cubes
    for _ in range(num_cubes):
        # Random position for cube (keep away from walls)
        margin = 0.5  # Minimum distance from walls
        cube_center = [
            np.random.uniform(room_min[0] + margin, room_max[0] - margin),
            np.random.uniform(room_min[1] + margin, room_max[1] - margin),
            np.random.uniform(room_min[2] + margin, room_max[2] - margin)
        ]
        
        # Random size (not too big)
        size = np.random.uniform(0.3, min(room_size) / 4)
        
        points, labels = generate_cube(
            cube_center, 
            size, 
            point_density=base_point_density * np.random.uniform(0.8, 1.5),
            noise_level=noise_level * np.random.uniform(0.8, 1.2)
        )
        all_points.append(points)
        all_labels.append(labels)
    
    # Add additional horizontal planes (tables, shelves, etc.)
    num_horizontal_planes = np.random.randint(1, 4)
    for _ in range(num_horizontal_planes):
        # Random position for horizontal plane
        margin = 0.5  # Minimum distance from walls
        plane_center = [
            np.random.uniform(room_min[0] + margin, room_max[0] - margin),
            np.random.uniform(room_min[1] + margin, room_max[1] - margin),
            np.random.uniform(room_min[2] + 0.5, room_max[2] - 0.5)  # Keep away from floor and ceiling
        ]
        
        # Random size
        width = np.random.uniform(0.5, room_size[0] / 3)
        length = np.random.uniform(0.5, room_size[1] / 3)
        
        points, labels = generate_horizontal_plane(
            plane_center, 
            width, 
            length, 
            point_density=base_point_density * np.random.uniform(0.8, 1.2),
            noise_level=noise_level * np.random.uniform(0.8, 1.2)
        )
        all_points.append(points)
        all_labels.append(labels)
    
    # Add random noise points
    total_shape_points = sum(p.shape[0] for p in all_points)
    num_noise_points = int(total_shape_points/10 * noise_percentage)
    
    noise_points, noise_labels = generate_random_noise(
        room_min, room_max, num_noise_points, label=0
    )
    all_points.append(noise_points)
    all_labels.append(noise_labels)
    
    # Combine all points and labels
    combined_points = np.vstack(all_points)
    combined_labels = np.concatenate(all_labels)
    
    return combined_points, combined_labels

# How can we have fun with this:
room_points, room_labels = generate_room(room_size=(8, 6, 2.5), num_spheres=3, num_cubes=2)
visualize_point_cloud(room_points, room_labels, show_ui=True, save_image=None)


#%% 9. Custom Export Function: PLY Writer

def write_ply_with_labels(points, labels, filename):
    """Write point cloud with labels to PLY file"""
    # Ensure points and labels have the same length
    if len(points) != len(labels):
        raise ValueError(f"Points array length ({len(points)}) doesn't match labels array length ({len(labels)})")
    
    # Create vertex with point coordinates and label
    vertex = np.zeros(len(points), dtype=[
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('label', 'u1')
    ])
    
    vertex['x'] = points[:, 0]
    vertex['y'] = points[:, 1]
    vertex['z'] = points[:, 2]
    vertex['label'] = labels
    
    # Write to file
    ply_header = '\n'.join([
        'ply',
        'format binary_little_endian 1.0',
        f'element vertex {len(points)}',
        'property float x',
        'property float y',
        'property float z',
        'property uchar label',
        'end_header'
    ])
    
    with open(filename, 'wb') as f:
        f.write(bytes(ply_header + '\n', 'utf-8'))
        vertex.tofile(f)
    
    print(f"Saved point cloud with {len(points)} points to {filename}")

# How can we have fun with this:
write_ply_with_labels(room_points, room_labels, "output_scalars.ply")

#%% 10. Export Function: Save with Open3D
def save_with_open3d(points, labels, filename):
    """Save point cloud with labels using Open3D"""
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Add labels as scalar field
    # We'll convert the labels to colors for visualization
    # Create a custom color map
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
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Save directly with our custom function to include labels
    o3d.io.write_point_cloud(filename, pcd)
    
    print(f"Saved point cloud with {points.shape[0]} points to {filename}")

# How can we have fun with this:
save_with_open3d(room_points, room_labels, "output_colored.ply")

#%% 11. Main function to generate room scenes

def generate_room_scenes(num_scenes=5, output_dir="synthetic_rooms"):
    """Generate multiple room scenes and save them as PLY files"""
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    for i in range(num_scenes):
        # Generate random room parameters
        room_size = (
            np.random.uniform(5, 15),  # width
            np.random.uniform(5, 15),  # length
            np.random.uniform(2.5, 4)   # height
        )
        
        num_spheres = np.random.randint(3, 20)
        num_cubes = np.random.randint(1, 20)
        base_point_density = np.random.uniform(100, 300)
        noise_percentage = np.random.uniform(0.05, 0.2)
        noise_level = np.random.uniform(0.01, 0.05)
        
        print(f"Generating room {i+1}/{num_scenes}...")
        print(f"- Room size: {room_size}")
        print(f"- Spheres: {num_spheres}, Cubes: {num_cubes}")
        print(f"- Base point density: {base_point_density:.1f}")
        print(f"- Noise percentage: {noise_percentage:.2f}")
        print(f"- Noise level: {noise_level:.3f}")
        
        # Generate room
        points, labels = generate_room(
            room_size=room_size,
            num_spheres=num_spheres,
            num_cubes=num_cubes,
            base_point_density=base_point_density,
            noise_percentage=noise_percentage,
            noise_level=noise_level
        )
        
        # Save PLY file
        filename = os.path.join(output_dir, f"synthetic_room_{i+1}.ply")
        write_ply_with_labels(points, labels, filename)
        
        print(f"Generated room with {points.shape[0]} points")
        print("- Label counts:")
        for label, name in {0: "Undefined", 1: "Horizontal plane", 2: "Vertical plane", 
                           3: "Sphere", 4: "Cube"}.items():
            count = np.sum(labels == label)
            percentage = (count / len(labels)) * 100
            print(f"  - {name}: {count} points ({percentage:.1f}%)")
        print()
    
    print(f"Generated {num_scenes} room scenes in {output_dir} directory")

# How can we have fun with this:
generate_room_scenes(num_scenes=15, output_dir="output_rooms_3")