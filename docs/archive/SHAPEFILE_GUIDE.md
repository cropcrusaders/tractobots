# 🗺️ SHAPEFILE INTEGRATION GUIDE

## Overview

This guide explains how to use shapefiles (.shp) from your operation center to define field boundaries for your Tractobots autonomous system. Shapefiles are a common GIS format that contains geographic data including field boundaries, obstacles, and other spatial information.

## Quick Start

### 1. Install Dependencies
```bash
# In your ROS2 workspace
cd ~/ros2_tractobots
source install/setup.bash
source /opt/ros/jazzy/setup.bash

# Install shapefile processing dependencies
ros2 run tractobots_mission_ui shapefile_manager --install-deps
```

### 2. Load Shapefile via GUI
```bash
# Launch the enhanced GUI
ros2 run tractobots_mission_ui enhanced_gui

# Or launch shapefile GUI directly
ros2 run tractobots_mission_ui shapefile_gui
```

**Steps in GUI:**
1. Go to **Field Management** → **Load Shapefile...**
2. Browse and select your .shp file
3. Click **Load Shapefile**
4. Review loaded fields in the Field Management tab
5. Generate coverage paths in the Path Planning tab
6. Export to ROS2 format in Export & Integration tab

### 3. Command Line Usage
```bash
# Process shapefile and generate field data
ros2 run tractobots_mission_ui shapefile_manager --shapefile /path/to/your/fields.shp

# Generate coverage path for specific field
ros2 run tractobots_mission_ui shapefile_manager --shapefile /path/to/fields.shp --field-id field_0 --swath-width 6.0
```

## Detailed Usage

### Shapefile Requirements

Your shapefiles should contain:
- **Polygon geometries** representing field boundaries
- **Coordinate system** (preferably WGS84, but others are auto-converted)
- **Attributes** like field names, IDs, or other metadata

**Supported formats:**
- Single polygon fields
- Multi-polygon complex fields
- Any coordinate reference system (automatically converted to WGS84)

### Field Boundary Processing

The system processes shapefiles to extract:

1. **Field Boundaries**: Coordinate points defining field perimeter
2. **Field Centers**: Calculated centroid for each field
3. **Areas**: Calculated area in hectares
4. **Metadata**: Field names, IDs, and custom attributes

**Example processed field data:**
```json
{
  "fields": [
    {
      "field_id": "field_0",
      "attributes": {
        "NAME": "North Field",
        "CROP": "Corn",
        "AREA": "45.2"
      },
      "area_hectares": 45.2,
      "center": {"lat": 40.123456, "lon": -95.654321},
      "boundary_points": [
        {"lat": 40.120000, "lon": -95.650000},
        {"lat": 40.125000, "lon": -95.650000},
        ...
      ]
    }
  ]
}
```

### Coverage Path Generation

The system generates coverage paths using:
- **Implement width**: Width of your tractor implement (default: 5m)
- **Overlap percentage**: Overlap between passes (default: 10%)
- **Pattern**: Back-and-forth parallel lines

**Parameters:**
```python
swath_width = 5.0      # meters
overlap = 0.1          # 10% overlap
```

**Generated waypoints:**
```json
{
  "field_id": "field_0",
  "waypoints": [
    {"lat": 40.120500, "lon": -95.650000},
    {"lat": 40.120500, "lon": -95.655000},
    {"lat": 40.121000, "lon": -95.655000},
    ...
  ]
}
```

## ROS2 Integration

### Field Boundary Service

Start the ROS2 service for field management:

```bash
ros2 run tractobots_mission_ui field_boundary_service
```

**Available services:**
- `/load_field_boundaries` - Load shapefile
- `/get_field_list` - Get available fields

**Available topics:**
- `/field_boundary` (PoseArray) - Field boundary points
- `/coverage_path` (Path) - Generated coverage path
- `/field_info` (String) - Field information

### Using with Navigation

1. **Load field boundaries:**
```bash
ros2 service call /load_field_boundaries std_msgs/srv/String "data: '/path/to/fields.shp'"
```

2. **Get field list:**
```bash
ros2 service call /get_field_list std_msgs/srv/String
```

3. **Subscribe to boundaries:**
```bash
ros2 topic echo /field_boundary
```

### Integration with Nav2

The generated paths are compatible with Nav2:

```yaml
# field_config.yaml
field_boundaries:
  metadata:
    source_file: "/path/to/fields.shp"
    num_fields: 3
  fields:
    field_0:
      name: "North Field"
      area_hectares: 45.2
      center_lat: 40.123456
      center_lon: -95.654321
      boundary_coordinates: [...]
```

## VS Code Integration

### Using VS Code Tasks

1. **Open Command Palette**: `Ctrl+Shift+P`
2. **Type**: "Tasks: Run Task"
3. **Select**: 
   - "🗺️ Load Shapefile" - Process shapefile
   - "🎯 Generate Coverage Path" - Create waypoints
   - "🚀 Launch Field Service" - Start ROS2 service

### Debug Integration

Set breakpoints in:
- `shapefile_manager.py` - Processing logic
- `field_boundary_service.py` - ROS2 service
- `shapefile_gui.py` - GUI interactions

## Command Reference

### Shapefile Manager CLI

```bash
# Install dependencies
ros2 run tractobots_mission_ui shapefile_manager --install-deps

# Process shapefile
ros2 run tractobots_mission_ui shapefile_manager --shapefile /path/to/fields.shp

# Generate coverage path
ros2 run tractobots_mission_ui shapefile_manager \
  --shapefile /path/to/fields.shp \
  --field-id field_0 \
  --swath-width 6.0

# Example with specific field
ros2 run tractobots_mission_ui shapefile_manager \
  --shapefile ~/Downloads/farm_fields.shp \
  --field-id north_field \
  --swath-width 5.5
```

### ROS2 Commands

```bash
# Launch field boundary service
ros2 run tractobots_mission_ui field_boundary_service

# Launch shapefile GUI
ros2 run tractobots_mission_ui shapefile_gui

# Check available services
ros2 service list | grep field

# Check field topics
ros2 topic list | grep field

# Monitor field boundaries
ros2 topic echo /field_boundary

# Monitor coverage path
ros2 topic echo /coverage_path
```

## File Formats

### Input Formats
- **Shapefile (.shp)** - Primary input format
- **Associated files**: .dbf, .shx, .prj (automatically handled)

### Output Formats
- **JSON** - Human-readable field data
- **YAML** - ROS2 parameter format
- **CSV** - Waypoint coordinates
- **ROS2 messages** - Live topic data

### Example File Structure
```
field_data/
├── shapefiles/
│   ├── farm_fields.shp          # Input shapefile
│   ├── farm_fields.dbf          # Attributes
│   ├── farm_fields.shx          # Index
│   └── farm_fields.prj          # Projection
├── field_boundaries.json        # Processed data
├── field_config.yaml           # ROS2 config
└── coverage_path_field_0.json  # Generated paths
```

## Troubleshooting

### Common Issues

1. **Missing dependencies:**
```bash
# Install manually if auto-install fails
pip install --user geopandas shapely pyproj fiona PyYAML
```

2. **Coordinate system issues:**
   - System auto-converts to WGS84
   - Verify your shapefile has valid projection info (.prj file)

3. **Large shapefiles:**
   - Processing may take time for complex fields
   - Consider simplifying geometry if needed

4. **Memory issues:**
   - For very large datasets, process fields individually
   - Use field selection in GUI

### Validation

```bash
# Verify shapefile is valid
ros2 run tractobots_mission_ui shapefile_manager --shapefile /path/to/fields.shp

# Check ROS2 integration
ros2 topic echo /field_info

# Test coverage path generation
ros2 service call /load_field_boundaries std_msgs/srv/String "data: '/path/to/fields.shp'"
```

## Advanced Usage

### Custom Coverage Patterns

Extend `shapefile_manager.py` to implement:
- Spiral patterns
- Offset boundaries
- Obstacle avoidance
- Custom turn patterns

### Integration with GPS/RTK

Configure coordinate transformations:
```python
# Convert to local coordinate system
transformer = pyproj.Transformer.from_crs('EPSG:4326', 'EPSG:32614')
x, y = transformer.transform(lat, lon)
```

### Real-time Updates

Monitor shapefile changes:
```python
# Watch for file updates
from watchdog.observers import Observer
# Implement file monitoring for live updates
```

## Examples

### Farm Operation Center Workflow

1. **Export from farm management software** to shapefile
2. **Transfer to tractor** via USB/network
3. **Load in Tractobots** using GUI or command line
4. **Generate coverage paths** for specific implements
5. **Execute autonomous mission** with generated waypoints

### Multi-Field Operation

```bash
# Process farm with multiple fields
ros2 run tractobots_mission_ui shapefile_manager --shapefile farm_2024.shp

# Generate paths for each field
for field in field_0 field_1 field_2; do
  ros2 run tractobots_mission_ui shapefile_manager \
    --shapefile farm_2024.shp \
    --field-id $field \
    --swath-width 6.0
done
```

## Support

For issues or questions:
1. Check logs in GUI export tabs
2. Run system diagnostics: `python3 diagnose_problems.py`
3. Verify ROS2 services: `ros2 service list`
4. Check documentation: `GUI_GUIDE.md`, `SETUP_SUCCESS.md`

Your shapefile integration is now ready for production use! 🚜✨
