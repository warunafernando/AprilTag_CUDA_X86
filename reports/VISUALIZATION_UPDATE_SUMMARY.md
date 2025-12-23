# Visualization Update Summary

## Date: December 2024

## Overview
This document summarizes the visualization improvements made to the AprilTag CUDA detector video visualization tool (`video_visualize_fixed.cu`).

## Key Changes

### 1. Table Layout and Positioning
- **Location**: Moved table from top-right corner to top-left corner
- **Width**: Reduced from 380px to 220px for more compact display
- **Column Spacing**: Optimized spacing between columns:
  - ID: 0px
  - X: 35px
  - Y: 70px  
  - Z: 105px
  - Dist: 140px
  - Prob: 175px

### 2. Font Size Optimization
- **Body text font scale**: Reduced to 0.38 (from 0.45)
- **FPS font scale**: Reduced to 0.45 (from 0.5)
- **Line height**: 25px
- **Text thickness**: 1px

### 3. Data Display Improvements
- **Numeric Precision**: All numeric values display with 2 decimal places (X, Y, Z, Distance, Probability)
- **Distance Display**: Removed "m" suffix, showing raw distance values
- **Probability Normalization**: Decision margin normalized to 0-1 range (divided by 150, capped at 1.0)
  - Formula: `normalized_prob = min(decision_margin / 150.0, 1.0)`

### 4. Table Structure
The table displays the following information in the top-left corner:
- **FPS**: Current processing frame rate (1 decimal place)
- **Column Headers**: ID, X, Y, Z, Dist, Prob
- **Tag Rows**: One row per detected tag with valid pose estimation

### 5. Visual Elements
- **Background**: Black rectangle with white border
- **Text Color**: White (255, 255, 255)
- **Table Border**: 1px white border around background
- **Header Separator**: Horizontal line under column headers

## Technical Details

### Table Rendering Function
- Function: `draw_info_table(Mat &im, const vector<TagPoseInfo> &tags, double fps)`
- Position: Top-left corner (table_x = 10, start_y = 10)
- Dynamic Height: Adjusts based on number of detected tags

### Data Flow
1. Tags are detected and pose information extracted
2. Only tags with valid pose estimation are included in the table
3. Distance calculated as: `sqrt(x² + y² + z²)`
4. Decision margin normalized before display
5. Table rendered on BGR color frame before display

## Configuration

All detector parameters remain configurable via `config.txt`, including:
- Camera intrinsics
- Distortion coefficients
- Tag size
- Detector parameters (quad_decimate, filtering thresholds, etc.)

## Output Format

### Default Behavior
- **Display only**: Video displayed on screen with real-time visualization
- **No file output**: Use `--output <path>` to enable video file writing

### Visualization Elements on Frame
1. **Tag Outline**: Yellow rectangle around detected tags
2. **3D Axes**: Color-coded axes showing tag orientation
3. **Tag ID**: Green text showing tag ID number
4. **Information Table**: Compact table in top-left corner with all tag data

## Performance Notes

- Table rendering overhead: Minimal (~0.1-0.2ms per frame)
- All drawing operations performed in main thread
- Thread-safe writer thread available for file output (when enabled)

## Future Improvements

Potential enhancements for consideration:
- Configurable table position (left/right, top/bottom)
- Configurable font sizes via config file
- Additional filtering/display options
- Configurable normalization factor for probability display










