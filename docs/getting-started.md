# Getting Started with 3D Point Cloud Room Generator

This guide will help you set up and start using the 3D Point Cloud Room Generator to create synthetic 3D environments.

## Installation

### Option 1: Using Conda (Recommended)

```bash
# Clone the repository
git clone https://github.com/florentpoux/3d-room-generator.git
cd 3d-room-generator

# Create a conda environment
conda create -n room_generator python=3.10
conda activate room_generator

# Install dependencies
pip install -r requirements.txt
```

### Option 2: Using pip

```bash
# Clone the repository
git clone https://github.com/florentpoux/3d-room-generator.git
cd 3d-room-generator

# Install dependencies
pip install numpy open3d matplotlib
```

## Verify Installation

Run the following script to verify that everything is working correctly:

```python
# test_installation.py
import numpy as np
import open3d as o3d
from shape_generator import generate_horizontal_plane
from read_ply import visualize_point_cloud

# Generate a simple horizontal plane
points, labels = generate_horizontal_plane(
    center=[0, 0, 0], 
    width=5, 
    length=5
)

# Visualize the result
visualize_point_cloud(points, labels)

print("Installation successful!")
```

Run it with:

```bash
python test_installation.py
```

If you see a visualization window with a horizontal plane and the success message, your installation is working correctly.

## Basic Usage

### Generate a Simple Room

```python
from shape_generator import generate_room, write_ply_with_labels
from read_ply import visualize_point_cloud, generate_stats

# Generate a room with default parameters
points, labels = generate_room()

# Save to PLY file
write_ply_with_labels(points, labels, "simple_room.ply")

# Visualize the result
visualize_point_cloud(points, labels)

# Generate statistics about the point cloud
generate_stats(labels)
```

This will:
1. Generate a synthetic room with default parameters
2. Save it to a PLY file named "simple_room.ply"
3. Open a visualization window to view the room
4. Print statistics about the room's point cloud composition

### Generate Multiple Rooms

To generate multiple room variations:

```python
from shape_generator import generate_room_scenes

# Generate 5 different rooms in the "synthetic_rooms" directory
generate_room_scenes(
    num_scenes=5,
    output_dir="synthetic_rooms"
)

print("Generated 5 room variations in the 'synthetic_rooms' directory")
```

## Customizing Room Parameters

You can customize various parameters when generating rooms:

```python
from shape_generator import generate_room, write_ply_with_labels

# Generate a custom room
points, labels = generate_room(
    room_size=(15, 12, 3.5),  # Width, length, height
    num_spheres=5,            # Number of spherical objects
    num_cubes=7,              # Number of cubic objects
    base_point_density=200,   # Points per square unit
    noise_percentage=0.15,    # Percentage of random points
    noise_level=0.03          # Standard deviation of noise
)

# Save to PLY file
write_ply_with_labels(points, labels, "custom_room.ply")
```

## Visualizing Saved PLY Files

To visualize a previously saved PLY file:

```python
from read_ply import read_ply_with_labels, visualize_point_cloud

# Read PLY file
points, labels = read_ply_with_labels("simple_room.ply")

# Visualize
visualize_point_cloud(points, labels)
```

## Saving Visualizations as Images

You can save visualizations as images for documentation or presentations:

```python
from read_ply import read_ply_with_labels, visualize_point_cloud

# Read PLY file
points, labels = read_ply_with_labels("simple_room.ply")

# Visualize and save as image
visualize_point_cloud(
    points, 
    labels, 
    show_ui=True,  # Show the visualization window
    save_image="room_visualization.png"  # Save as image
)
```

## Understanding Semantic Labels

The room generator assigns semantic labels to each point:

| Label | Semantic Meaning     | Visualization Color |
|-------|----------------------|---------------------|
| 0     | Undefined/Noise      | Gray                |
| 1     | Horizontal Plane     | Tan                 |
| 2     | Vertical Plane       | Red                 |
| 3     | Sphere               | Green               |
| 4     | Cube                 | Blue                |

## Next Steps

- Check the [API Reference](api_reference.md) for detailed function documentation
- Explore [Examples and Tutorials](examples.md) for more advanced usage
- Review [Advanced Usage](advanced_usage.md) for machine learning applications

## Troubleshooting

### Common Issues

**Issue**: "ImportError: No module named 'open3d'"

**Solution**: Make sure you've installed the Open3D library. Try running:
```bash
pip install open3d
```

**Issue**: Visualization doesn't show up

**Solution**: Check if you have a display server running. On headless systems, you might need to use an alternative visualization method or save directly to image files.

**Issue**: Low memory errors when generating large rooms

**Solution**: Reduce the point density or room size. You can also try generating rooms in batches or using a machine with more memory.

## Getting Help

If you encounter any issues:

1. Check the [documentation](./README.md)
2. Look through [existing issues](https://github.com/florentpoux/3d-room-generator/issues)
3. Create a new issue with a detailed description of your problem

For additional resources, check out:
- [3D Data Science with Python Book](https://www.oreilly.com/library/view/3d-data-science/9781098161323/)
- [3D Geodata Academy](https://learngeodata.eu)
