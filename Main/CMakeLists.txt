
cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED ) 

include_directories( ${ARUCO_PARCEL_MARKER_SOURCE_DIR}/UDPPublisher )
include_directories( ${ARUCO_PARCEL_MARKER_SOURCE_DIR}/ModuleInterface )
include_directories( ${ARUCO_PARCEL_MARKER_SOURCE_DIR}/UDPImage )

link_directories( ${ARUCO_PARCEL_MARKER_BINARY_DIR}/UDPPublisher )
link_directories( ${ARUCO_PARCEL_MARKER_BINARY_DIR}/ModuleInterface )
link_directories( ${ARUCO_PARCEL_MARKER_BINARY_DIR}/UDPImage )

add_executable( Aruco_Parcel_Module Main.cpp )

target_link_libraries( Aruco_Parcel_Module UDPPublisher ModuleInterface UDPImage ${OpenCV_LIBS} -pthread )

