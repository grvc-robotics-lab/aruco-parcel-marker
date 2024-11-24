
cmake_minimum_required(VERSION 2.8)

project( Aruco_Parcel_Module )

find_package(OpenCV REQUIRED ) 

add_subdirectory( UDPPublisher )

add_subdirectory( ModuleInterface )

add_subdirectory( UDPImage )

add_subdirectory( Main )

