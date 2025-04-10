# API Reference

This document provides detailed information about all available functions in the 3D Point Cloud Room Generator.

## Table of Contents

- [Shape Generation](#shape-generation)
  - [generate_horizontal_plane](#generate_horizontal_plane)
  - [generate_vertical_plane](#generate_vertical_plane)
  - [generate_sphere](#generate_sphere)
  - [generate_cube](#generate_cube)
  - [generate_random_noise](#generate_random_noise)
- [Room Generation](#room-generation)
  - [generate_room](#generate_room)
  - [generate_room_scenes](#generate_room_scenes)
- [File I/O](#file-io)
  - [write_ply_with_labels](#write_ply_with_labels)
  - [save_with_open3d](#save_with_open3d)
  - [read_ply_with_labels](#read_ply_with_labels)
- [Visualization](#visualization)
  - [visualize_point_cloud](#visualize_point_cloud)
  - [generate_stats](#generate_stats)

---

## Shape Generation

### generate_horizontal_plane

Generates a horizontal plane (floor, ceiling, table) with controlled noise.

```python
def generate_horizontal_plane(center, width, length, point_density=100, noise_level=0.02):
    """
    Generate a horizontal plane with noise.
    
    Parameters:
    -----------
    center : list or array [x, y, z]
        3D coordinates of the plane's center
    width : float
        Width of the plane (X-axis)
    length : float
        Length of the plane (Y-axis)
    point_density : float, default=100
        Points per square unit
    noise_level : float, default=0.02
        Standard deviation of the Gaussian noise
        
    Returns:
    --------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of the plane points
    labels : ndarray of shape (n_points,)
        Semantic labels (1 for horizontal plane)
    """
```

**Example:**
```python
# Generate a 10x10 floor centered at [5,5,0]
floor_points, floor_labels = generate_horizontal_plane(
    center=[5, 5, 0],
    width=10,
    length=10
)
```

### generate_vertical_plane

Generates a vertical plane (wall) with controlled noise.

```python
def generate_vertical_plane(center, width, height, orientation=0, point_density=100, noise_level=0.02):
    """
    Generate a vertical plane with noise.
    
    Parameters:
    -----------
    center : list or array [x, y, z]
        3D coordinates of the plane's center
    width : float
        Width of the plane
    height : float
        Height of the plane (Z-axis)
    orientation : float, default=0
        Rotation angle around Z-axis in degrees
    point_density : float, default=100
        Points per square unit
    noise_level : float, default=0.02
        Standard deviation of the Gaussian noise
        
    Returns:
    --------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of the plane points
    labels : ndarray of shape (n_points,)
        Semantic labels (2 for vertical plane)
    """
```

**Example:**
```python
# Generate a 10x3 wall centered at [0,5,1.5] facing north
wall_points, wall_labels = generate_vertical_plane(
    center=[0, 5, 1.5],
    width=10, 
    height=3,
    orientation=90
)
```

### generate_sphere

Generates a sphere with controlled noise.

```python
def generate_sphere(center, radius, point_density=100, noise_level=0.02):
    """
    Generate a sphere with noise.
    
    Parameters:
    -----------
    center : list or array [x, y, z]
        3D coordinates of the sphere's center
    radius : float
        Radius of the sphere
    point_density : float, default=100
        Points per square unit of surface area
    noise_level : float, default=0.02
        Standard deviation of the Gaussian noise
        
    Returns:
    --------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of the sphere points
    labels : ndarray of shape (n_points,)
        Semantic labels (3 for sphere)
    """
```

**Example:**
```python
# Generate a sphere with radius 0.5 centered at [5,5,1.5]
sphere_points, sphere_labels = generate_sphere(
    center=[5, 5, 1.5],
    radius=0.5
)
```

### generate_cube

Generates a cube with controlled noise.

```python
def generate_cube(center, size, point_density=100, noise_level=0.02):
    """
    Generate a cube with noise.
    
    Parameters:
    -----------
    center : list or array [x, y, z]
        3D coordinates of the cube's center
    size : float
        Length of each side of the cube
    point_density : float, default=100
        Points per square unit of surface area
    noise_level : float, default=0.02
        Standard deviation of the Gaussian noise
        
    Returns:
    --------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of the cube points
    labels : ndarray of shape (n_points,)
        Semantic labels (4 for cube)
    """
```

**Example:**
```python
# Generate a cube with size 0.8 centered at [3,4,1]
cube_points, cube_labels = generate_cube(
    center=[3, 4, 1],
    size=0.8
)
```

### generate_random_noise

Generates random noise points throughout a defined volume.

```python
def generate_random_noise(room_min, room_max, num_points=1000, label=0):
    """
    Generate random noise points throughout the room.
    
    Parameters:
    -----------
    room_min : list or array [x_min, y_min, z_min]
        Minimum room coordinates
    room_max : list or array [x_max, y_max, z_max]
        Maximum room coordinates
    num_points : int, default=1000
        Number of noise points to generate
    label : int, default=0
        Semantic label for noise points
        
    Returns:
    --------
    points : ndarray of shape (num_points, 3)
        XYZ coordinates of the noise points
    labels : ndarray of shape (num_points,)
        Semantic labels (default 0 for undefined/noise)
    """
```

**Example:**
```python
# Generate 500 noise points in a 10x8x3 room
room_min = [0, 0, 0]
room_max = [10, 8, 3]
noise_points, noise_labels = generate_random_noise(
    room_min,
    room_max,
    num_points=500
)
```

## Room Generation

### generate_room

Generates a complete room with all components.

```python
def generate_room(
    room_size=(10, 8, 3),
    num_spheres=2,
    num_cubes=3,
    base_point_density=50,
    noise_percentage=0.1,
    noise_level=0.02
):
    """
    Generate a complete room with shapes and noise.
    
    Parameters:
    -----------
    room_size : tuple of float, default=(10, 8, 3)
        Room dimensions as (width, length, height)
    num_spheres : int, default=2
        Number of spherical objects to place in the room
    num_cubes : int, default=3
        Number of cubic objects to place in the room
    base_point_density : float, default=50
        Base points per square unit
    noise_percentage : float, default=0.1
        Percentage of total points to be random noise
    noise_level : float, default=0.02
        Standard deviation of Gaussian noise
        
    Returns:
    --------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of all points
    labels : ndarray of shape (n_points,)
        Semantic labels for all points
    """
```

**Example:**
```python
# Generate a room with custom parameters
points, labels = generate_room(
    room_size=(15, 12, 3.5),
    num_spheres=5,
    num_cubes=7,
    base_point_density=200,
    noise_percentage=0.15,
    noise_level=0.03
)
```

### generate_room_scenes

Generates multiple room scenes with random variations.

```python
def generate_room_scenes(num_scenes=5, output_dir="synthetic_rooms"):
    """
    Generate multiple room scenes and save them as PLY files.
    
    Parameters:
    -----------
    num_scenes : int, default=5
        Number of room scenes to generate
    output_dir : str, default="synthetic_rooms"
        Directory where to save the generated PLY files
        
    Returns:
    --------
    None
        Saves PLY files to the specified directory
    """
```

**Example:**
```python
# Generate 10 different rooms in the "my_rooms" directory
generate_room_scenes(
    num_scenes=10,
    output_dir="my_rooms"
)
```

## File I/O

### write_ply_with_labels

Writes a point cloud with semantic labels to a PLY file.

```python
def write_ply_with_labels(points, labels, filename):
    """
    Write point cloud with labels to PLY file.
    
    Parameters:
    -----------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of points
    labels : ndarray of shape (n_points,)
        Semantic labels for each point
    filename : str
        Output PLY filename
        
    Returns:
    --------
    None
        Writes to the specified file
    """
```

**Example:**
```python
# Save generated points and labels to a PLY file
write_ply_with_labels(points, labels, "room.ply")
```

### save_with_open3d

Saves a point cloud with color-coded labels using Open3D.

```python
def save_with_open3d(points, labels, filename):
    """
    Save point cloud with labels using Open3D.
    
    Parameters:
    -----------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of points
    labels : ndarray of shape (n_points,)
        Semantic labels for each point
    filename : str
        Output PLY filename
        
    Returns:
    --------
    None
        Writes to the specified file with colors based on labels
    """
```

**Example:**
```python
# Save colored point cloud
save_with_open3d(points, labels, "colored_room.ply")
```

### read_ply_with_labels

Reads a PLY file with semantic labels.

```python
def read_ply_with_labels(filename):
    """
    Read PLY file and extract points and labels.
    
    Parameters:
    -----------
    filename : str
        Path to the PLY file
        
    Returns:
    --------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of points
    labels : ndarray of shape (n_points,)
        Semantic labels for each point
    """
```

**Example:**
```python
# Read points and labels from a PLY file
points, labels = read_ply_with_labels("room.ply")
```

## Visualization

### visualize_point_cloud

Visualizes a point cloud with color-coded semantic labels.

```python
def visualize_point_cloud(points, labels, show_ui=True, save_image=None):
    """
    Visualize point cloud with color-coded labels.
    
    Parameters:
    -----------
    points : ndarray of shape (n_points, 3)
        XYZ coordinates of points
    labels : ndarray of shape (n_points,)
        Semantic labels for each point
    show_ui : bool, default=True
        Whether to show the interactive visualization window
    save_image : str, default=None
        If provided, saves visualization to the specified image file
        
    Returns:
    --------
    None
        Shows visualization and/or saves image
    """
```

**Example:**
```python
# Visualize point cloud and save as image
visualize_point_cloud(
    points, 
    labels, 
    show_ui=True,
    save_image="room_visualization.png"
)
```

### generate_stats

Generates statistics about the point cloud labels.

```python
def generate_stats(labels):
    """
    Generate statistics about the point cloud labels.
    
    Parameters:
    -----------
    labels : ndarray of shape (n_points,)
        Semantic labels for each point
        
    Returns:
    --------
    None
        Prints statistics to console
    """
```

**Example:**
```python
# Print statistics about the point cloud composition
generate_stats(labels)
```

---

For additional information, refer to:
- [Getting Started Guide](getting_started.md)
- [Examples and Tutorials](examples.md)
- [Advanced Usage](advanced_usage.md)
