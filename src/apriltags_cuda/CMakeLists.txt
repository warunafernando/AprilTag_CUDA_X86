include(ExternalProject)
include(ProcessorCount)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)  # Ensure user has a compatible version of CMake

project(ApriltagCuda LANGUAGES CUDA CXX)  # Set project name and specify the languages used

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# CUDA standard (CUDA20 not supported in older CMake, use 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Look for required packages.
find_package(CUDA REQUIRED)
find_package(glog REQUIRED)
find_package(GTest REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)
pkg_check_modules(LIBAV REQUIRED libavformat libavcodec libavutil libswscale)

#set(GLOG_INSTALL_DIR ${CMAKE_BINARY_DIR}/glog-install)
#set(GTEST_INSTALL_DIR ${CMAKE_BINARY_DIR}/gtest-install)
set(OPENCV_INSTALL_DIR ${CMAKE_BINARY_DIR}/OpenCV-install)
set(WPILIB_INSTALL_DIR ${CMAKE_BINARY_DIR}/wpilib-install)
set(CCCL_INSTALL_DIR ${CMAKE_BINARY_DIR}/cccl-install)
set(SEASOCKS_INSTALL_DIR ${CMAKE_BINARY_DIR}/seasocks-install)
set(JSON_INSTALL_DIR ${CMAKE_BINARY_DIR}/json-install)
# Using system-installed apriltag via pkg-config
# set(APRILTAG_INSTALL_DIR ${CMAKE_BINARY_DIR}/apriltag-install)

# Determine number of processors for the parallel builds.
if(NOT DEFINED NUM_PROCESSORS)
    ProcessorCount(NUM_PROCESSORS)
    if(NUM_PROCESSORS EQUAL 0)
        set(NUM_PROCESSORS 1)
    endif()
endif()

# Add OPENCV package
ExternalProject_Add(
    OpenCV
    PREFIX ${CMAKE_BINARY_DIR}/OpenCV
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.9.0
    CMAKE_GENERATOR Ninja
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR}
    BUILD_COMMAND ninja -C <BINARY_DIR> -j${NUM_PROCESSORS}
    INSTALL_COMMAND ninja -C <BINARY_DIR> install
)

set(CMAKE_PREFIX_PATH ${OPENCV_INSTALL_DIR} ${CMAKE_PREFIX_PATH})
message(STATUS "OPENCV_INSTALL_DIR: ${OPENCV_INSTALL_DIR} CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# Add wpilib package
ExternalProject_Add(
    wpilib
    PREFIX ${CMAKE_BINARY_DIR}/wpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
    GIT_TAG v2024.3.2
    CMAKE_GENERATOR Ninja
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${WPILIB_INSTALL_DIR} -DWITH_JAVA=OFF -DWITH_GUI=OFF -DOpenCV_DIR=${OPENCV_INSTALL_DIR}/lib/cmake/opencv4 -DCMAKE_CXX_STANDARD=20
    BUILD_COMMAND ninja -C <BINARY_DIR> -j${NUM_PROCESSORS}
    INSTALL_COMMAND ninja -C <BINARY_DIR> install
)

# Add CCCL package
ExternalProject_Add(
    CCCL 
    PREFIX ${CMAKE_BINARY_DIR}/cccl
    GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
    GIT_TAG v2.3.2
    CMAKE_GENERATOR Ninja
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
ExternalProject_Get_Property(CCCL SOURCE_DIR)
set(CCCL_SOURCE_DIR ${SOURCE_DIR})

# Add seasocks package
ExternalProject_Add(
    seasocks
    PREFIX ${CMAKE_BINARY_DIR}/seasocks
    GIT_REPOSITORY https://github.com/mattgodbolt/seasocks.git
    GIT_TAG v1.4.6
    CMAKE_GENERATOR Ninja
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${SEASOCKS_INSTALL_DIR}
    BUILD_COMMAND ninja -C <BINARY_DIR> -j${NUM_PROCESSORS}
    INSTALL_COMMAND ninja -C <BINARY_DIR> install
)

ExternalProject_Add(
    json
    PREFIX ${CMAKE_BINARY_DIR}/json
    GIT_REPOSITORY https://github.com/nlohmann/json
    GIT_TAG v3.11.3
    CMAKE_GENERATOR Ninja
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${JSON_INSTALL_DIR} -DJSON_BuildTests=OFF
    BUILD_COMMAND ninja -C <BINARY_DIR> -j${NUM_PROCESSORS}
    INSTALL_COMMAND ninja -C <BINARY_DIR> install
)

# Using system-installed apriltag instead of ExternalProject
# ExternalProject_Add(
#     apriltag
#     PREFIX ${CMAKE_BINARY_DIR}/apriltag
#     GIT_REPOSITORY https://github.com/cgpadwick/apriltag.git
#     GIT_TAG 3.3.0 
#     CMAKE_GENERATOR Ninja
#     CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${APRILTAG_INSTALL_DIR} -DOpenCV_DIR=${OPENCV_INSTALL_DIR}
#     BUILD_COMMAND ninja -C <BINARY_DIR> -j${NUM_PROCESSORS}
#     INSTALL_COMMAND ninja -C <BINARY_DIR> install
# )

add_dependencies(wpilib OpenCV)
# add_dependencies(apriltag OpenCV wpilib CCCL seasocks json)

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CUDA_COMPILER)
    message(STATUS "CUDA Compiler: ${CMAKE_CUDA_COMPILER}")

    if(CMAKE_CUDA_COMPILER MATCHES "nvcc")
        message(STATUS "Using NVCC as CUDA compiler")
        if(CMAKE_BUILD_TYPE MATCHES "Debug")
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 --expt-relaxed-constexpr -g")
        else()
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 --expt-relaxed-constexpr -Xptxas -O3")
        endif()
    else()
        message(STATUS "Using a different CUDA compiler")
        if(CMAKE_BUILD_TYPE MATCHES "Debug")
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 -g -gdwarf-4 -O0")
        else()
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 -O3")
        endif()
    endif()
else()
    message(FATAL_ERROR "CUDA compiler not found. Please set CMAKE_CUDA_COMPILER.")
endif()

if(ENABLE_ASAN)
    message(STATUS "AddressSanitizer is enabled")
    set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${COMMON_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ASAN_FLAGS}")
endif()

if(ENABLE_MSAN)
    message(STATUS "MemorySanitizer is enabled")
    set(MSAN_FLAGS "-fsanitize=memory -fsanitize-memory-track-origins=2 -fno-omit-frame-pointer -fsanitize-memory-use-after-dtor")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} ${MSAN_FLAGS}")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${COMMON_FLAGS} ${MSAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MSAN_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${MSAN_FLAGS}")
endif()

# Gather all source files in the current directory
set(CUDA_LIB_SOURCES 
    src/apriltag_detect.cu 
    src/apriltag_gpu.cu
    src/cuda_frc971.cu
    src/labeling_allegretti_2019_BKE.cu
    src/line_fit_filter.cu
    src/points.cu
    src/threshold.cu
    src/apriltag_utils.cu
    src/video_processor.cu)

# Add FFmpeg video reader library
add_library(ffmpeg_video_reader STATIC tools/ffmpeg_video_reader.cpp)
target_include_directories(ffmpeg_video_reader PUBLIC ${CMAKE_SOURCE_DIR}/tools)
target_link_libraries(ffmpeg_video_reader ${LIBAV_LIBRARIES})
target_include_directories(ffmpeg_video_reader PUBLIC ${LIBAV_INCLUDE_DIRS})

# Add a library with the above source files
add_library(apriltag_cuda ${CUDA_LIB_SOURCES})

# Explicitly add apriltag headers for CUDA compilation  
target_include_directories(apriltag_cuda PRIVATE /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

# No dependency on ExternalProject apriltag - using system version
# add_dependencies(apriltag_cuda apriltag)

# Include directories for the compiler
include_directories( 
    /usr/local/include
    ${CCCL_SOURCE_DIR}/cub
    ${CCCL_SOURCE_DIR}/libcudacxx/include
    ${CCCL_SOURCE_DIR}/thrust
    /usr/local/cuda-12.2/include
    ${OPENCV_INSTALL_DIR}/include/opencv4
    ${SEASOCKS_INSTALL_DIR}/include
    ${JSON_INSTALL_DIR}/include)
link_directories(/usr/local/lib)

# Add executable for OpenCV CUDA demo
add_executable(opencv_cuda_demo src/opencv_cuda_demo.cu)
target_include_directories(opencv_cuda_demo PRIVATE /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)
target_link_libraries(opencv_cuda_demo 
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)

# Add executable for visualize
add_executable(visualize src/visualize.cu)
target_include_directories(visualize PRIVATE /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)
target_link_libraries(visualize
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)



# Add the test executable
add_executable(gpu_detector_test src/gpu_detector_test.cu)
target_include_directories(gpu_detector_test PRIVATE /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)
target_link_libraries(gpu_detector_test
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog
    GTest::GTest)

add_executable(json_test src/json_test.cpp)
target_link_libraries(json_test
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${SEASOCKS_INSTALL_DIR}/lib/libseasocks.a
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog
    Threads::Threads
    ZLIB::ZLIB)

add_executable(video_bench tools/video_bench.cu)
target_link_libraries(video_bench
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)
target_include_directories(video_bench PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(video_visualize tools/video_visualize.cu)
target_link_libraries(video_visualize
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so
    glog::glog)
target_include_directories(video_visualize PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(video_visualize_fixed tools/video_visualize_fixed.cu)
target_link_libraries(video_visualize_fixed
    apriltag_cuda
    ffmpeg_video_reader
    ${APRILTAG_LIBRARIES}
    ${LIBAV_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so
    MVSDK
    glog::glog
    Threads::Threads
    rt)  # For shared memory (shm_open, etc.)
target_include_directories(video_visualize_fixed PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tools ${LIBAV_INCLUDE_DIRS} /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)
# Suppress nvcc warning 611 (MultiBandBlender override) for this CUDA TU
set_source_files_properties(tools/video_visualize_fixed.cu PROPERTIES
    COMPILE_OPTIONS "-Xcudafe;--diag_suppress=611")

add_executable(camera_gpu_detect tools/camera_gpu_detect.cu)
target_link_libraries(camera_gpu_detect
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    MVSDK
    glog::glog
    Threads::Threads
    rt)
target_include_directories(camera_gpu_detect PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tools /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)
set_source_files_properties(tools/camera_gpu_detect.cu PROPERTIES
    COMPILE_OPTIONS "-Xcudafe;--diag_suppress=611")

add_executable(video_visualize_cpu tools/video_visualize_cpu.cu src/apriltag_utils.cu)
target_link_libraries(video_visualize_cpu
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so
    glog::glog)
target_include_directories(video_visualize_cpu PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(compare_detectors tools/compare_detectors.cu src/apriltag_utils.cu)
target_link_libraries(compare_detectors
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)
target_include_directories(compare_detectors PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(debug_coordinates tools/debug_coordinates.cu src/apriltag_utils.cu)
target_link_libraries(debug_coordinates
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)
target_include_directories(debug_coordinates PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(extract_test_frames tools/extract_test_frames.cu src/apriltag_utils.cu)
target_link_libraries(extract_test_frames
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)
target_include_directories(extract_test_frames PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(test_specific_frames tools/test_specific_frames.cu src/apriltag_utils.cu)
target_link_libraries(test_specific_frames
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)
target_include_directories(test_specific_frames PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(debug_coordinate_pipeline tools/debug_coordinate_pipeline.cu src/apriltag_utils.cu)
target_link_libraries(debug_coordinate_pipeline
    apriltag_cuda
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)
target_include_directories(debug_coordinate_pipeline PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(camera_gui tools/camera_gui.cpp)
target_link_libraries(camera_gui
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    glog::glog)
target_include_directories(camera_gui PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(camera_gui_qt tools/camera_gui_qt.cpp src/apriltag_utils.cu)
target_link_libraries(camera_gui_qt
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so
    MVSDK
    Qt5::Widgets
    ${APRILTAG_LIBRARIES}
    glog::glog
    rt)  # For shared memory (shm_open, etc.)
target_include_directories(camera_gui_qt PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(test_camera_read tools/test_camera_read.cpp)
target_link_libraries(test_camera_read
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    MVSDK
    glog::glog)
target_include_directories(test_camera_read PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(test_camera_read_optimized tools/test_camera_read_optimized.cpp)
target_link_libraries(test_camera_read_optimized
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    MVSDK
    glog::glog
    Threads::Threads)
target_include_directories(test_camera_read_optimized PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(test_camera_read_threaded tools/test_camera_read_threaded.cpp)
target_link_libraries(test_camera_read_threaded
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    MVSDK
    glog::glog
    Threads::Threads)
target_include_directories(test_camera_read_threaded PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(simple_camera_detect tools/simple_camera_detect.cpp)

add_executable(test_image_detect tools/test_image_detect.cpp)
target_link_libraries(test_image_detect
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(test_image_detect PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(debug_undistorted_detection tools/debug_undistorted_detection.cpp)
target_link_libraries(debug_undistorted_detection
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(debug_undistorted_detection PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(debug_tag_filtering tools/debug_tag_filtering.cpp)
target_link_libraries(debug_tag_filtering
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(debug_tag_filtering PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(debug_detection_stages tools/debug_detection_stages.cpp)
target_link_libraries(debug_detection_stages
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(debug_detection_stages PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(debug_edge_detection tools/debug_edge_detection.cpp)
target_link_libraries(debug_edge_detection
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(debug_edge_detection PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(debug_contour_detection tools/debug_contour_detection.cpp)
target_link_libraries(debug_contour_detection
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(debug_contour_detection PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(debug_corner_homography tools/debug_corner_homography.cpp)
target_link_libraries(debug_corner_homography
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(debug_corner_homography PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(debug_pattern_extraction tools/debug_pattern_extraction.cpp)
target_link_libraries(debug_pattern_extraction
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(debug_pattern_extraction PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(analyze_tag_decode tools/analyze_tag_decode.cpp)
target_link_libraries(analyze_tag_decode
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(analyze_tag_decode PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(compare_quads_side_by_side tools/compare_quads_side_by_side.cpp)
target_link_libraries(compare_quads_side_by_side
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(compare_quads_side_by_side PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(adjust_image_brightness tools/adjust_image_brightness.cpp)
target_link_libraries(adjust_image_brightness
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(adjust_image_brightness PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(test_detection_on_adjusted tools/test_detection_on_adjusted.cpp)
target_link_libraries(test_detection_on_adjusted
    ${APRILTAG_LIBRARIES}
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(test_detection_on_adjusted PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

add_executable(compare_homography tools/compare_homography.cpp)
target_link_libraries(compare_homography
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(compare_homography PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(compare_pattern_extraction tools/compare_pattern_extraction.cpp)
target_link_libraries(compare_pattern_extraction
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(compare_pattern_extraction PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(hamming_decode tools/hamming_decode.cpp)
target_link_libraries(hamming_decode
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(hamming_decode PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(visualize_hamming_decode tools/visualize_hamming_decode.cpp)
target_link_libraries(visualize_hamming_decode
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(visualize_hamming_decode PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(visualize_hamming_mirror tools/visualize_hamming_mirror.cpp)
target_link_libraries(visualize_hamming_mirror
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so)
target_include_directories(visualize_hamming_mirror PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(simple_camera_detect
    ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_highgui.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so
    ${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so
    MVSDK
    ${APRILTAG_LIBRARIES}
    Threads::Threads)
target_include_directories(simple_camera_detect PRIVATE ${CMAKE_SOURCE_DIR}/src /usr/local/include /usr/local/include/apriltag /usr/local/include/apriltag/common)

# Add a custom target to format all source files
add_custom_target(format_all
	COMMAND clang-format -i -style=Google ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    COMMENT "Running clang-format on all source files"
)
