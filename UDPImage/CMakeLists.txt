cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )

add_library( UDPImage UDPImage.h UDPImage.cpp )

target_link_libraries( UDPImage ${OpenCV_LIBS} )
