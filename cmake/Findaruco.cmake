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
message(WARNING "Aruco is required for the SLAM module. To find Aruco, replace the file rgbd_rtk/cmake/Findaruco.cmake by the one generated after compiling Aruco in your machine.")
