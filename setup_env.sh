#!/bin/bash
# Setup environment for CUDA AprilTag detector

# Set OpenCV library path (built as ExternalProject)
export LD_LIBRARY_PATH=/home/nav/Apriltag/StandAlone/src/apriltags_cuda/build/OpenCV-install/lib:$LD_LIBRARY_PATH

# Set CUDA paths
export CUDA_HOME=/usr/local/cuda-12.2
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

echo "Environment set up for CUDA AprilTag detector"
echo "OpenCV libraries: $LD_LIBRARY_PATH"
echo ""
echo "To use, run: source setup_env.sh"
