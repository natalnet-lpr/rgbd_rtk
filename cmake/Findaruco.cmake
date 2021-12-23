# ===================================================================================
#  aruco CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(aruco REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME )
#
#    This file will define the following variables:
#      - aruco_LIBS          : The list of libraries to links against.
#      - aruco_LIB_DIR       : The directory where lib files are. Calling LINK_DIRECTORIES
#                                with this path is NOT needed.
#      - aruco_VERSION       : The  version of this PROJECT_NAME build. Example: "1.2.0"
#      - aruco_VERSION_MAJOR : Major version part of VERSION. Example: "1"
#      - aruco_VERSION_MINOR : Minor version part of VERSION. Example: "2"
#      - aruco_VERSION_PATCH : Patch version part of VERSION. Example: "0"
#
# ===================================================================================
INCLUDE_DIRECTORIES("/usr/local/include")
INCLUDE_DIRECTORIES("/usr/local/include/aruco")
SET(aruco_INCLUDE_DIRS "/usr/local/include")

LINK_DIRECTORIES("/usr/local/lib")
SET(aruco_LIB_DIR "/usr/local/lib")

SET(aruco_LIBS opencv_imgproc;opencv_videostab;opencv_ml;opencv_photo;opencv_stitching;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_shape;opencv_superres;opencv_highgui;opencv_imgcodecs;opencv_dnn;opencv_flann;opencv_core;opencv_video;opencv_videoio;opencv_dpm;opencv_bgsegm;opencv_aruco;opencv_hfs;opencv_optflow;opencv_bioinspired;opencv_xfeatures2d;opencv_phase_unwrapping;opencv_ximgproc;opencv_surface_matching;opencv_plot;opencv_xobjdetect;opencv_saliency;opencv_dnn_objdetect;opencv_datasets;opencv_hdf;opencv_fuzzy;opencv_reg;opencv_img_hash;opencv_tracking;opencv_rgbd;opencv_text;opencv_ccalib;opencv_line_descriptor;opencv_xphoto;opencv_face;opencv_structured_light;opencv_stereo aruco) 

SET(aruco_FOUND YES)
SET(aruco_FOUND "YES")
SET(aruco_VERSION        3.1.12)
SET(aruco_VERSION_MAJOR  3)
SET(aruco_VERSION_MINOR  1)
SET(aruco_VERSION_PATCH  12)
