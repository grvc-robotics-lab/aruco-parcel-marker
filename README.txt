Module name:	Aruco Parcel Module
Developer: 		Alejandro Suarez, asuarezfm@us.es
Organization: 	University of Seville
Date: 			12 November 2024
Update:			19 November 2024
Project: 		euROBIN

DESCRIPTION

This software module serves to detect and localize parcels to be grasped by a mobile robotic platform using Aruco markers.

The module provides two interfaces:

	- ManagerInterface: class used to send signals and receive requests to/from the operation manager, controlling the execution of the programa.
	- UDPImage: class used to send JPEG encoded images through UDP sockets to provide visual feedback to the human supervisor.

Grasping points will be sent through UDP sockets in data packets corresponding to C-style data structures.


PARAMETERS

FILE		PARAMETER		DEFAULT VALUE
-------------------------------------------------------
Main.cpp	cameraIndex		4
Main.cpp	markerSize		30 mm


REQUIREMENTS

Open CV version: 4.10


UPDATES

- Create UDPPublisher interface for sending grasping points and robot pose.
- Modified to require three parameters: (1) camera index, (2) parcel Aruco ID, (3) UDP image flag.
- Added thread for keyboard read, including "exit" and "quit" commands for terminating the program.


DOCUMENTATION

// https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
