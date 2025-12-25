# AprilTag Debug GUI - Recent Changes

## Date: December 2024

This document summarizes all the changes and improvements made to the AprilTag Debug GUI application.

## Major Features Added

### 1. Tab Reorganization
- **Capture Tab** moved to first position (default tab on startup)
- **Algorithms Tab** (second position)
- **Fisheye Correction Tab** (third position)
- **Settings Tab** (new, fourth position)
- **Processing Tab** (fifth position)
- Camera selection defaults to "None" on startup

### 2. Settings Tab
A new comprehensive settings tab that consolidates all application settings:

#### Fisheye Calibration Settings
- **Load calibration on startup**: Checkbox to automatically load fisheye calibration when GUI starts
- **Calibration file path**: Configurable path to calibration YAML file
- **Enable for MindVision cameras by default**: Automatically apply fisheye correction to MindVision cameras

#### Camera Settings (Per Camera)
- **Camera selector**: Dropdown to select which camera's settings to edit
- **Editable settings**:
  - Exposure (slider + spinbox)
  - Gain (slider + spinbox)
  - Brightness (slider + spinbox)
  - Contrast (slider + spinbox)
  - Saturation (slider + spinbox)
  - Sharpness (slider + spinbox)
  - Mode (dropdown)
- Settings are saved per camera to `camera_settings.txt`
- Settings are automatically loaded when a camera is selected
- Save/Load buttons to persist settings to config file (`~/.apriltag_debug_gui_config.yaml`)

### 3. Camera Settings Management
- Camera settings are now editable in the Settings tab
- Settings are automatically saved/loaded per camera
- Settings file format: `camera_settings.txt` (text-based, per-camera sections)
- Main config file: `~/.apriltag_debug_gui_config.yaml` (YAML format)
- Settings persist across application restarts

### 4. Pattern Visualization Enhancements (Capture Tab)

#### Layout Changes
- **One tag per row**: Each detected tag is displayed on its own row
- **Three boxes per tag** (arranged horizontally):
  1. **Warped Image** (left): Original warped tag image with border/extraction region visualization
  2. **Gray Color Box** (middle): 8x8 grid showing actual grayscale pixel values (0-255) as colors
  3. **Digitized Pattern** (right): 8x8 grid with black border and 6x6 data region (black/white only, no text)

#### Gray Color Box (New)
- 8x8 grid structure matching the digitized pattern
- Black border cells (top, bottom, left, right rows/columns)
- 6x6 data region in center showing actual grayscale values (0-255) as visual colors
- Each cell displays the exact pixel value from the extracted pattern

#### Digitized Pattern Box
- Removed numeric text labels ("1" and "0")
- Clean black and white visualization only
- 8x8 grid with black border and 6x6 data region

#### Auto-Sizing
- Tags are automatically sized to fit all three boxes in the available space
- Box size calculated as: `(available_width - 2*spacing) / 3`
- Responsive to number of tags detected

### 5. Processing Tab Improvements

#### Stage Selection
- **Moved to top**: Stage Selection group box is now the first element in the Processing tab
- **Compact sizing**: Reduced to fit controls only (no extra space)
- **Layout order**:
  1. Stage Selection (top)
  2. Control panel (Load Image buttons)
  3. Image display area (splitter with images and info panel)

#### Stage Selection Controls
- Preprocessing options
- Edge Detection options
- Detection options
- Advanced visualization options
- Quad selection (for warped tags, pattern extraction, hamming decode)
- Mirror options (independent for each image)

### 6. Algorithm Tab Enhancements

#### FPS Display
- **Three-thread FPS tracking**:
  - **Read FPS**: Frame capture/reading thread
  - **Decode FPS**: Detection/decoding thread
  - **Display FPS**: Display/rendering thread
- Real-time FPS updates displayed in the timing label

#### Preview Display
- Fixed camera preview to show video feed
- Correct tag location rendering (matches detection coordinates)
- Mirror support for display frame

### 7. Capture Tab Improvements

#### Pattern Analysis
- Removed text-based 6x6 pattern output from "Pattern analysis test output"
- Full pattern visualization available in the Pattern Visualization section
- Hamming code information still displayed

#### Camera Selection
- Default to "None" on startup
- "None" option added as first item in camera combo box
- Selecting "None" shows "No camera selected" message

## Technical Changes

### Code Structure
- Added Settings tab UI components and management functions
- Implemented camera settings save/load functionality
- Enhanced pattern visualization rendering
- Improved layout management for responsive sizing

### File Changes
- `Tools/apriltag_debug_gui.cu`: Main GUI application file
  - Added Settings tab implementation
  - Enhanced pattern visualization
  - Improved tab ordering and layout
  - Added camera settings management

### Configuration Files
- `~/.apriltag_debug_gui_config.yaml`: Main settings file (YAML format)
  - Stores fisheye calibration settings
  - References camera settings file
- `camera_settings.txt`: Per-camera settings (text format)
  - Stores exposure, gain, brightness, contrast, saturation, sharpness, mode
  - Format: `[Camera]` and `[Settings]` sections per camera

## Bug Fixes

### Segmentation Fault Fixes
- Fixed crash on GUI startup related to uninitialized Settings tab UI pointers
- Added null checks for Settings tab widgets before access
- Proper initialization order for UI elements

### Camera Settings
- Fixed camera index mapping between Capture and Algorithm tabs
- Algorithm tab now correctly skips "None" option when populating camera list
- Proper handling of camera selection changes

### Pattern Visualization
- Fixed bounds checking for pattern grid rendering
- Improved error handling for invalid pattern data
- Fixed coordinate mapping for multi-pattern displays

## Usage Notes

### Settings Tab
1. Open Settings tab
2. Select a camera from the dropdown
3. Adjust settings (exposure, gain, etc.)
4. Click "Save Settings" to persist changes
5. Settings are automatically applied when camera is selected in Capture/Algorithm tabs

### Pattern Visualization
- Pattern visualization appears automatically when tags are detected in Capture tab
- Each tag shows three boxes: Warped Image, Gray Values, Digitized Pattern
- Gray color box shows actual pixel values (0-255) as visual colors
- Digitized pattern shows clean black/white pattern without text labels

### Camera Selection
- Default selection is "None" on startup
- Select a camera from the dropdown to start preview
- Camera settings are loaded automatically when camera is selected

## Future Enhancements

Potential improvements for future versions:
- Export pattern visualizations to image files
- Batch processing of multiple images
- Advanced pattern analysis tools
- Customizable visualization layouts
- Pattern comparison tools


