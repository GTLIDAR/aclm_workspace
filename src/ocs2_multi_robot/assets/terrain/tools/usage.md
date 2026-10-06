## STL to Point Cloud Converter

### Usage
```bash
# Basic usage (default spacing 0.01)
python stl_to_grid_pointcloud.py gap_slope_course.stl gap_slope_course.ply --spacing 0.05 --ground-margin 0.5
python stl_to_grid_pointcloud.py gap_slope_turn_course.stl gap_slope_turn_course.ply --spacing 0.05 --ground-margin 0.5
python stl_to_grid_pointcloud.py stairs_turn_course.stl stairs_turn_course.ply --spacing 0.05 --ground-margin 0.5
python stl_to_grid_pointcloud.py turn_course.stl turn_course.ply --spacing 0.01 --ground-margin 0.5
python stl_to_grid_pointcloud.py circle_course.stl circle_course.ply --spacing 0.02 --ground-margin 0.5
```

## Installation
```bash
pip install trimesh numpy rtree plyfile scipy matplotlib
```
