# Git Repository Setup Guide

This document provides instructions for initializing and pushing this project to git.

## Files to Commit

### Core Project Files
- `README.md` - Main project documentation
- `.gitignore` - Git ignore rules
- `setup_env.sh` - Environment setup script
- `CMakeLists.txt` (if exists at root)

### Source Code
- `src/apriltags_cuda/` - Main CUDA detector source code
  - `src/` - Core detection implementation
  - `tools/` - Utility tools (video_visualize_fixed.cu, compare_detectors.cu, etc.)
  - `CMakeLists.txt` - Build configuration
- `src/apriltag_cgpadwick/` - CPU AprilTag library (dependency)

### Documentation
- `instructions.md` - Original instructions (if needed)
- `instructions2.md` - Updated instructions (if needed)

## Files Excluded (via .gitignore)

- `build/` - Build artifacts
- `output/*.avi` - Output videos
- `output/*.csv` - Benchmark CSV files
- `output/*.txt` - Analysis reports
- `src/apriltags_cuda/build/` - Build directory
- `src/apriltag_cgpadwick/build/` - Build directory
- All compiled binaries (`.o`, `.so`, `.a`)
- CMake cache files
- Temporary files

## Initializing Git Repository

1. **Initialize git repository**:
```bash
cd /home/nav/Apriltag/StandAlone
git init
```

2. **Add all files** (respects .gitignore):
```bash
git add .
```

3. **Check what will be committed**:
```bash
git status
```

4. **Create initial commit**:
```bash
git commit -m "Initial commit: CUDA AprilTag detector with coordinate fixes and visualization"
```

5. **Add remote repository** (if you have one):
```bash
git remote add origin <your-repo-url>
```

6. **Push to remote**:
```bash
git branch -M main
git push -u origin main
```

## Recommended Commit Structure

For a clean history, consider separate commits:

1. **Initial project structure**:
```bash
git commit -m "Add project structure and build configuration"
```

2. **Core detection code**:
```bash
git commit -m "Add CUDA AprilTag detection implementation"
```

3. **Coordinate fixes**:
```bash
git commit -m "Fix coordinate scaling from decimated to full resolution"
```

4. **Visualization tools**:
```bash
git commit -m "Add 3D visualization with distance measurement"
```

5. **Documentation**:
```bash
git commit -m "Add comprehensive README and documentation"
```

## Pre-commit Checklist

- [ ] All build artifacts excluded via .gitignore
- [ ] Output videos not committed (should be in .gitignore)
- [ ] README.md is up to date
- [ ] .gitignore is comprehensive
- [ ] No sensitive information (API keys, passwords) in code
- [ ] License information included (if applicable)

## Repository Size Considerations

The source code is relatively small (~10-50 MB). However:
- **Build artifacts** are excluded (can be 1-5 GB)
- **Output videos** are excluded (can be 100-500 MB each)
- **External dependencies** (OpenCV, etc.) are built during CMake configuration, not stored in git

## Notes

- The `input/` directory contains video files. Consider whether to include these in git or use git-lfs for large files.
- Build instructions are in README.md - users will build dependencies during CMake configuration.
- System libraries (apriltag) are expected to be installed separately.

