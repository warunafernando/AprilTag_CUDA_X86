# apriltags_cuda
A standalone version of the First Robotics Team 971 cuda april tag detection code.

### Why A Standalone Version?

A few reasons:
  * don't want to pollute the 971 code base with cmake
  * the 971 code depends on a lot of internal code like AOS (Autonomous Operating System).  Clients may not want to integrate with AOS and the standalone version is simpler.
  * this is probably temporary and a better way will emerge soon.

## Building The Code On A Ubuntu/Debian Machine (not in docker)

1. Install the cuda toolkit and appropriate nvidia driver on your system.  Recommend version 11.8 of the cuda toolkit.  Complete instructions for cuda toolkit install for Ubuntu are here: <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#prepare-ubuntu>

2. Run the install_deps script as follows to install the dependencies: `sudo ./install_deps.sh`.  This script will try to detect whether cuda is installed or not and will try to install it for you. 

3. For builds on the Orin platform, the compute capability is 8.7.  For other non-embedded platforms you can determine your cuda compute capability of your GPU as follows: `nvidia-smi --query-gpu compute_cap --format=csv` .  On embedded platforms like the Orin, `nvidia-smi` does not exist.

4. Build the code as follows.  Use the compute capability determined above, e.g. 8.7 translates to 87 for CMake. For clang compilation use:
```bash
cmake -B build -DCMAKE_CUDA_COMPILER=clang++-17 -DCMAKE_CXX_COMPILER=clang++-17 -DCMAKE_CUDA_ARCHITECTURES=87 -DNUM_PROCESSORS=2
cmake --build build
```
If you are on a computer with lots of cores and RAM you could omit the DNUM_PROCESSORS directive and it will use all available cores for the build.


Leaving the CMAKE_BUILD_TYPE undefined will results in a Release build.  If you want a debug build add `-DCMAKE_BUILD_TYPE=Debug` to the command lines above.

The build process down pulls down a lot of packages and builds them.  This build can take a while - good time for an extended coffee break.  If the build completes successfully you can try to run the code as shown in the next section.  If not, then try debugging what is failing by adding the VERBOSE flag to make as follows `cd build; make VERBOSE=1`.

### Building The Code in a Docker Container

1. Follow the instructions for [installing the nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

2. Edit the nvidia container config file `sudo vim /etc/nvidia-container-runtime/config.toml` and ensure that `no-cgroups = false`, and save the file.

3. Restart the docker daemon: `sudo systemctl restart docker`

4. Run the docker build command in the current directory as follows: `docker build -t cuda-build:latest .`

5. When the docker build completes, run the docker in interactive mode with the following command: 
```bash
docker run -it -v/tmp:/tmp --runtime=nvidia --gpus all cuda-build:latest /bin/bash
```
This command maps the /tmp drive on the host to the /tmp drive in the docker container.

6. At the container cmd line:

```bash
cd /tmp
mkdir code
cd code
git clone https://github.com/cgpadwick/apriltags_cuda.git
cd apriltags_cuda
cmake -B build -DCMAKE_CUDA_COMPILER=clang++-17 -DCMAKE_CXX_COMPILER=clang++-17 -DCMAKE_CUDA_ARCHITECTURES=75
cmake --build build
```

The code should build and the build artifacts will be put in the `build` directory.  Test and see if it is working by running the test: `cd build; ./gpu_detector_test`

## Running The OpenCV Demo Code

Plug in a USB web cam into your system.  Then run the code as follows:

```bash
cd build
./opencv_cuda_demo
```

A window should pop up with the webcam feed displayed.  If you hold an april tag in front of the webcam then it should be detected and the outlines of the tag should be drawn.

## Running visualize

There is a utility called `visualize` that visualizes the imagery at a few points in the gpu detection pipeline.  To run it:

```bash
cd build
./visualize
```

Press any key to cycle through the different visualizations.

## Running The Tests

There is a test called gpu_detector_test.  This runs a suite of gtest test fixtures that test various parts of the code.  To run the test:

```bash
cd build
./gpu_detector_test
```

The output should look something like this:

```bash
[==========] Running 4 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 4 tests from GpuDetectorTest
[ RUN      ] GpuDetectorTest.GpuDetectsAprilTag
[       OK ] GpuDetectorTest.GpuDetectsAprilTag (179 ms)
[ RUN      ] GpuDetectorTest.GpuNoAprilTagDetections
[       OK ] GpuDetectorTest.GpuNoAprilTagDetections (114 ms)
[ RUN      ] GpuDetectorTest.CpuDetectsAprilTag
[       OK ] GpuDetectorTest.CpuDetectsAprilTag (110 ms)
[ RUN      ] GpuDetectorTest.CpuNoAprilTagDetections
[       OK ] GpuDetectorTest.CpuNoAprilTagDetections (92 ms)
[----------] 4 tests from GpuDetectorTest (496 ms total)

[----------] Global test environment tear-down
[==========] 4 tests from 1 test suite ran. (496 ms total)
[  PASSED  ] 4 tests.
```

## Running The Detection System

This code ships with a GPU apriltag detection pipeline, and a flask based web viewer.  To run the detection system do the following:

* Plug in a USB webcam to your system.

* From the root directory (e.g. apriltags_cuda):
```bash
# Start the GPU Detection Pipeline:
./build/ws_server -camera_idx 0 -cal_file data/calibrationmatrix.json
```

* This will start the GPU detection pipeline running off of frames captured from /dev/video0.  You can also set other indices if your camera mounts on /dev/video1 or a different device.  This command launches a websocket server accessible from port 8080 on the local machine.

* Now bring up a web browser and navigate to `http://localhost:8080` and you should see something like shown below

Flask App: ![Alt](/res/webserver.png "Webserver Screenshot")

* You can adjust between manual and auto exposure.  When manual exposure is selected you can adjust the exposure and brightness of the image.  If you hold an apriltag of type 36h11 it should be detected by the system and outlines of the detection will be shown.

Information for how data is sent to and from the Orin on NetworkTables is stored on the following Google Doc <a> https://docs.google.com/document/d/1zhl0dlSLXOld302rhOQrhItLp2thuDD308Gv3yvkHMY/edit?usp=sharing</a>

If you are an M-A Student looking for access to the build server, please fill out the form <a> https://docs.google.com/document/d/14MWXYbr9kazaDxuQmgdHVRad1crJv6h6GJaEA4p0HTE/edit?usp=sharing </a>