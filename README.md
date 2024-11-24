# aruco-parcel-marker
This software module is used for the relative localization of the mobile manipulator and for grasing parcels labeled with Aruco tags.

# Description

This software module serves to detect and localize parcels to be grasped by a mobile robotic platform using Aruco markers. It is based on the following tutorial: // https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

The module provides three interfaces with other modules:

	- ModuleInterface: class used to send message or signals and to receive requests to/from the operation manager, controlling the execution of the programa.
  - UDPPublisher: socket interface equivalent to ModuleInterface, used to send the pose of the detected marker to the specified hosts given by IP address and UDP port.
	- UDPImage: class used to send JPEG encoded images through UDP sockets to provide visual feedback to the human supervisor. The degree of compression is defined by the constant ENCODED_IMAGE_JPEG_QUALITY, set to 15% quality by default in order to reduce the packet size sent through the socket.

Grasping points will be sent through UDP sockets in data packets corresponding to C-style data structures.


# About data packets

Data packets are the information units interchanged by software modules. These are implemented as C/C++ style data structures within the ModuleInterface.h file. The data structure (packet) is DATA_PACKET_MODULE_MSG. It includes the following fields:
	(1) Three bytes (characters) header for recognizing the type of data packet.
	(2) One byte for the ID of the module.
	(3) One byte for the ID of the remote host that sends/receives the message.
	(4) One byte indicating the code of the message, whose meaning is defined by the user.
	(5) One byte indicating the option of the message, whose meaning is defined by the user.
	(6) A data buffer of MAX_SIZE_USER_BUFFER bytes that can be used to attach user data.
	(7) Two bytes (uint16_t) indicating the length in bytes of the user buffer.
	(8) A checksum byte corresponding to the XOR operation over the bytes from filed (2) to last byte of field (7). 
	
This data packet is intended to be generic and customizable according to user needs. For example, in this module it can be used to change the ID of the parcel that should be detected, or for sending to another module the pose of the marker relative to the camera.

It is very important to include the modifier __attribute__((packed)) in the definition of the structure to avoid zero padding, which may affect the size of the packet and prevent correct communication.


# Requirements

This software module requies OpenCV 4.10, making use of the C++ API. It may be compiled with previous versions (4.9 or earlier).


# Installation

Download or clone the project in your desired folder. Create a build forlder (mkdir build), go inside the folder (cd build), generate compilation files (cmake ..) and compile (make). The executable can be fund in the buil/Main folder.


# Usage

To run the module, it is necessary to provide as input arguments the camera index (typically 4 for RealSense D435i), the ID of the Aruco marker, and a binary flag (0 or 1) to disable or enable the visual feedback of JPEG encoded images sent through UDP socket. Example:

./Aruco_Parcel_Module 4 3 1


# Customization

This software is designed to be customized as required by the project. It is important to ensure that the data structure representing the data packet sent/received is exactly the same in for both sender and receiver. Otherwise, messages/requests will not be correctly interchanged.


# Contact

This sofware has been developed by Alejandro Suarez (asuarezfm@us.es) from the GRVC Robotics Lab of the University of Seville (Spain). Please, cite or include link of this repository in your publications.


# Disclaimer

This open-source software comes with absolutely no warranty. 

