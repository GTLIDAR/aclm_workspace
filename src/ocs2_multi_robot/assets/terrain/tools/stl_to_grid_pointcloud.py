#!/usr/bin/env python3
"""
STL to Grid Point Cloud Converter

Converts an STL mesh file to an evenly-spaced grid point cloud by ray casting
from all 6 directions (±X, ±Y, ±Z) to capture all surfaces including vertical walls.
Optionally, a ground plane can be added at Z=0 with a specified margin around the mesh bounds.
"""

import argparse
import numpy as np
import sys
import os

try:
    import trimesh
except ImportError:
    print("Error: trimesh is required. Install with: pip install trimesh")
    sys.exit(1)

try:
    import rtree
except ImportError:
    print("Error: rtree is required. Install with: pip install rtree")
    sys.exit(1)

import struct


def export_ply_binary_vtk_format(points, output_path):
    """
    Export point cloud as binary PLY in VTK-compatible format.
    This format is compatible with PCL's PointXYZRGBNormal reader.
    
    Args:
        points: numpy array of shape (N, 3) with XYZ coordinates
        output_path: path to output PLY file
    """
    import numpy as np
    
    # Convert to float32 and fix negative zeros
    # Negative zero (0x80000000) can cause issues with some software
    points_f32 = np.array(points, dtype=np.float32)
    # Replace -0.0 with +0.0 by adding 0.0
    points_f32 = np.where(points_f32 == 0, np.float32(0.0), points_f32)
    
    num_vertices = len(points_f32)
    
    # Write header matching VTK format (same as obstacle_course.ply)
    header = f"""ply
format binary_little_endian 1.0
comment VTK generated PLY File
obj_info vtkPolyData points and polygons: vtk4.0
element vertex {num_vertices}
property float x
property float y
property float z
element face 0
property list uchar int vertex_indices
end_header
"""
    
    with open(output_path, 'wb') as f:
        f.write(header.encode('ascii'))
        # Write all vertices as contiguous binary data
        f.write(points_f32.tobytes())


def estimate_spacing_for_target_size(mesh, target_size_mb):
    """
    Estimate grid spacing to achieve approximately the target file size.
    
    Args:
        mesh: trimesh mesh object
        target_size_mb: target file size in megabytes
    
    Returns:
        float: estimated spacing value
    """
    # PLY ASCII format: ~12 bytes per point for coordinates
    # Account for header and some overhead
    bytes_per_point = 12
    target_points = int((target_size_mb * 1024 * 1024) / bytes_per_point)
    
    # Estimate based on surface area
    surface_area = mesh.area
    spacing = np.sqrt(surface_area / target_points)
    
    return spacing


def generate_ground_plane(bounds, spacing, margin=0.5, z_height=0.0):
    """
    Generate a ground plane at Z=0 with specified margin around the mesh bounds.
    
    Args:
        bounds: mesh bounds [min_bound, max_bound]
        spacing: grid spacing
        margin: extra margin around the mesh bounds (default: 0.5m)
        z_height: height of the ground plane (default: 0.0)
    
    Returns:
        numpy array of ground plane points
    """
    min_bound, max_bound = bounds[0], bounds[1]
    
    x_min = min_bound[0] - margin
    x_max = max_bound[0] + margin
    y_min = min_bound[1] - margin
    y_max = max_bound[1] + margin
    
    x = np.arange(x_min, x_max + spacing, spacing)
    y = np.arange(y_min, y_max + spacing, spacing)
    xx, yy = np.meshgrid(x, y)
    
    ground_points = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, z_height)])
    
    return ground_points


def cast_rays_from_direction(mesh, spacing, bounds, direction):
    """
    Cast rays from one direction and return intersection points.
    
    Args:
        mesh: trimesh mesh object
        spacing: grid spacing
        bounds: mesh bounds [min_bound, max_bound]
        direction: 'z+', 'z-', 'x+', 'x-', 'y+', 'y-'
    
    Returns:
        numpy array of intersection points
    """
    min_bound, max_bound = bounds[0], bounds[1]
    
    if direction in ['z+', 'z-']:
        # XY grid
        x = np.arange(min_bound[0], max_bound[0] + spacing, spacing)
        y = np.arange(min_bound[1], max_bound[1] + spacing, spacing)
        xx, yy = np.meshgrid(x, y)
        
        if direction == 'z+':
            origins = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, max_bound[2] + 1)])
            dirs = np.tile([0, 0, -1], (xx.size, 1))
        else:
            origins = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, min_bound[2] - 1)])
            dirs = np.tile([0, 0, 1], (xx.size, 1))
            
    elif direction in ['x+', 'x-']:
        # YZ grid
        y = np.arange(min_bound[1], max_bound[1] + spacing, spacing)
        z = np.arange(min_bound[2], max_bound[2] + spacing, spacing)
        yy, zz = np.meshgrid(y, z)
        
        if direction == 'x+':
            origins = np.column_stack([np.full(yy.size, max_bound[0] + 1), yy.ravel(), zz.ravel()])
            dirs = np.tile([-1, 0, 0], (yy.size, 1))
        else:
            origins = np.column_stack([np.full(yy.size, min_bound[0] - 1), yy.ravel(), zz.ravel()])
            dirs = np.tile([1, 0, 0], (yy.size, 1))
            
    elif direction in ['y+', 'y-']:
        # XZ grid
        x = np.arange(min_bound[0], max_bound[0] + spacing, spacing)
        z = np.arange(min_bound[2], max_bound[2] + spacing, spacing)
        xx, zz = np.meshgrid(x, z)
        
        if direction == 'y+':
            origins = np.column_stack([xx.ravel(), np.full(xx.size, max_bound[1] + 1), zz.ravel()])
            dirs = np.tile([0, -1, 0], (xx.size, 1))
        else:
            origins = np.column_stack([xx.ravel(), np.full(xx.size, min_bound[1] - 1), zz.ravel()])
            dirs = np.tile([0, 1, 0], (xx.size, 1))
    else:
        raise ValueError(f"Invalid direction: {direction}")
    
    locations, _, _ = mesh.ray.intersects_location(origins, dirs)
    return locations


def stl_to_grid_pointcloud(stl_path, output_path, spacing=None, target_size_mb=None, 
                           ground_plane_margin=None, verbose=True):
    """
    Convert STL mesh to evenly-spaced grid point cloud.
    
    Args:
        stl_path: path to input STL file
        output_path: path to output file (PLY, XYZ, etc.)
        spacing: grid spacing (if None, will use target_size_mb or default)
        target_size_mb: target file size in MB (used if spacing is None)
        ground_plane_margin: if set, adds a ground plane at Z=0 with this margin (in meters)
        verbose: print progress information
    
    Returns:
        numpy array of point cloud vertices
    """
    if verbose:
        print(f"Loading mesh from {stl_path}...")
    
    mesh = trimesh.load(stl_path)
    bounds = mesh.bounds
    dimensions = bounds[1] - bounds[0]
    
    if verbose:
        print(f"Mesh bounds: {bounds[0]} to {bounds[1]}")
        print(f"Mesh dimensions: {dimensions}")
        print(f"Surface area: {mesh.area:.2f}")
    
    # Determine spacing
    if spacing is None:
        if target_size_mb is not None:
            spacing = estimate_spacing_for_target_size(mesh, target_size_mb)
            if verbose:
                print(f"Auto-calculated spacing for ~{target_size_mb}MB: {spacing:.6f}")
        else:
            spacing = 0.01  # default
            if verbose:
                print(f"Using default spacing: {spacing}")
    else:
        if verbose:
            print(f"Using specified spacing: {spacing}")
    
    # Cast rays from all 6 directions
    directions = ['z+', 'z-', 'x+', 'x-', 'y+', 'y-']
    all_points = []
    
    for direction in directions:
        if verbose:
            print(f"Casting rays from {direction}...", end=" ")
        
        points = cast_rays_from_direction(mesh, spacing, bounds, direction)
        all_points.append(points)
        
        if verbose:
            print(f"{len(points):,} hits")
    
    # Combine all points
    all_points = [p for p in all_points if len(p) > 0]
    combined = np.vstack(all_points)
    
    if verbose:
        print(f"\nTotal raw points from mesh: {len(combined):,}")
    
    # Add ground plane if margin is specified
    if ground_plane_margin is not None:
        if verbose:
            print(f"Adding ground plane at Z=0 with {ground_plane_margin}m margin...")
        ground_points = generate_ground_plane(bounds, spacing, margin=ground_plane_margin, z_height=0.0)
        combined = np.vstack([combined, ground_points])
        if verbose:
            print(f"Ground plane points: {len(ground_points):,}")
            print(f"Total combined points: {len(combined):,}")
    
    # Remove duplicates by snapping to grid
    if verbose:
        print("Removing duplicates...")
    
    snap_resolution = spacing * 0.5
    rounded = np.round(combined / snap_resolution) * snap_resolution
    unique_points = np.unique(rounded, axis=0)
    
    if verbose:
        print(f"Unique points: {len(unique_points):,}")
    
    # Export in VTK-compatible binary PLY format (compatible with PCL PointXYZRGBNormal)
    if verbose:
        print(f"\nSaving to {output_path}...")
    
    export_ply_binary_vtk_format(unique_points, output_path)
    
    file_size = os.path.getsize(output_path) / (1024 * 1024)
    
    if verbose:
        print(f"Output file size: {file_size:.2f} MB")
        print("Done!")
    
    return unique_points


def main():
    parser = argparse.ArgumentParser(
        description="Convert STL mesh to evenly-spaced grid point cloud",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s model.stl model.ply
  %(prog)s model.stl model.ply --spacing 0.05
  %(prog)s model.stl model.ply --target-size 25
  %(prog)s model.stl model.ply --ground-margin 0.5
  %(prog)s model.stl model.xyz --spacing 0.1 --quiet
        """
    )
    
    parser.add_argument("input", help="Input STL file path")
    parser.add_argument("output", help="Output point cloud file path (PLY, XYZ, etc.)")
    
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--spacing", "-s",
        type=float,
        help="Grid spacing between points (default: 0.01)"
    )
    group.add_argument(
        "--target-size", "-t",
        type=float,
        dest="target_size",
        help="Target output file size in MB (auto-calculates spacing)"
    )
    
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Suppress progress output"
    )
    
    parser.add_argument(
        "--ground-margin", "-g",
        type=float,
        dest="ground_margin",
        help="Add ground plane at Z=0 with specified margin around mesh bounds (in meters)"
    )
    
    args = parser.parse_args()
    
    # Validate input file
    if not os.path.exists(args.input):
        print(f"Error: Input file not found: {args.input}")
        sys.exit(1)
    
    # Run conversion
    try:
        stl_to_grid_pointcloud(
            stl_path=args.input,
            output_path=args.output,
            spacing=args.spacing,
            target_size_mb=args.target_size,
            ground_plane_margin=args.ground_margin,
            verbose=not args.quiet
        )
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
