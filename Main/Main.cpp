/*
 * Main.cpp
 *
 * Aruco Parcel Module
 * 
 * Author: Alejandro Suarez, asuarezfm@us.es
 * Date: November 2024
 * Project: euROBIN
 *
 * Code base on: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
 */

// Standard library
#include <iostream>
#include <string>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

// OpenCV library
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>

// Custom library
#include "../ModuleInterface/ModuleInterface.h"
#include "../UDPPublisher/UDPPublisher.h"
#include "../UDPImage/UDPImage.h"

using namespace std;
using namespace cv;


// Constant definition
#define IP_ADDRESS_LiCAS		"127.0.0.1"
#define UDP_PORT_LiCAS 			32002

#define IP_ADDRESS_MULTIROTOR	"192.168.0.171"
#define UDP_PORT_MULTIROTOR		28000

#define IP_ADDRESS_ADROM		"192.168.0.219"
#define UDP_PORT_TX_ADROM		25003
#define UDP_PORT_RX_ADROM		24003

#define IP_ADDRESS_VIEWER		"192.168.0.171"
#define UDP_PORT_VIEWER			28001

#define IMAGE_BUFFER_LENGTH		10244

#define UPDATE_RATE				20.0F	// Hz

#define TARGET_MARKER_ID		1


// Data packet sent through UDP socket containing the grasping points of the parcel
typedef struct
{
	uint8_t header[3];		// "GRP" character sequence (Aruco Marker Detector)
	uint8_t validL;
	uint8_t validR;
	float pL[3];
	float pR[3];
} __attribute__((packed)) DATA_PACKET_GRASPING_POINTS;

typedef struct
{
	uint8_t header[3];		// "RPP" character sequence (Robot Pose Packet)
	uint8_t updated;
	float pos[3];
	float rot[3];
} __attribute__((packed)) DATA_PACKET_ROBOT_POSE;


// Global
float markerPosition[3] = {0.0, 0.0, 0.0};
float markerRotation[3] = {0.0, 0.0, 0.0};
uint8_t updatedFlag = 0;
int endThreadSignal = 0;

// Function declaration
void keyboardThreadFunction();
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);


/*
 * Main function
 */
int main(int argc, char ** argv)
{
	cv::Mat inputImage;
	cv::VideoCapture inputVideo;
	Mat camMatrix, distCoeffs;
	
	int cameraIndex = 0;
	float markerSize = 48;
	int targetMarker = 0;
	int targetMarkedFound = 0;
	
	struct timeval tini;
	struct timeval tend;
	double delta_t = 0;
	double updatePeriod = 1/30.0;
	int key = 0;
	int errorCode = 0;
	int k = 0;
	int i = 0;
	
	// Threads
	thread keyboardThread;
	
	DATA_PACKET_GRASPING_POINTS dataPacketGraspingPoints;
	DATA_PACKET_ROBOT_POSE dataPacketRobotPose;
	UDPPublisher publisherLiCAS = UDPPublisher();
	UDPPublisher publisherMulti = UDPPublisher();
	// float pL_handle_offset[3] = {0.02, 0.0, -0.06};
	// float pR_handle_offset[3] = {0.02, 0.0, -0.06};
	// Offset for big DHL parcel ID 3
	float pL_handle_offset[3] = {0.0, 0.0, 0.14};
	float pR_handle_offset[3] = {0.0, 0.0, 0.14};
	
	
	cout << "Aruco Parcel Module" << endl << endl;
	cout << "Alejandro Suarez, asuarezfm@us.es" << endl;
	cout << "GRVC Robotics Lab, University of Seville" << endl;
	cout << endl;

	if(argc != 4)
	{
		cout << "ERROR [in main]: invalid number of arguments.\nSpecify camera index, parcel ID, and UDP iamge flag." << endl;
		errorCode = 1;
		
		return errorCode;
	}
	
	// Open video capture
	cameraIndex = atoi(argv[1]);
	targetMarker = atoi(argv[2]);
	
	inputVideo.open(cameraIndex);
	if(!inputVideo.isOpened())
	{
		cout << "ERROR [in main]: could not open video device. Check camera index." << endl;
		errorCode = 2;
		
		return errorCode;
	}
	
	// Configure videocapture properties
	inputVideo.set(CAP_PROP_FRAME_WIDTH, 640.0);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, 480.0); 
	inputVideo.set(CAP_PROP_FPS, 30);

	// Read camera parameters from file
	readCameraParameters("CameraCalibration.yml", camMatrix, distCoeffs);
		
	// Create Aruco variables
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
	cv::aruco::ArucoDetector detector(dictionary, detectorParams);
		
	// Set coordinate system for Aruco markers
	cv::Mat objPoints(4, 1, CV_32FC3);
	objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerSize/2.f, markerSize/2.f, 0);
	objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerSize/2.f, markerSize/2.f, 0);
	objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerSize/2.f, -markerSize/2.f, 0);
	objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerSize/2.f, -markerSize/2.f, 0);

	
	// Init data packets
	dataPacketGraspingPoints.header[0] = 'G';
	dataPacketGraspingPoints.header[1] = 'R';
	dataPacketGraspingPoints.header[2] = 'P';
	dataPacketRobotPose.header[0] = 'R';
	dataPacketRobotPose.header[1] = 'P';
	dataPacketRobotPose.header[2] = 'P';
	for(k = 0; k < 3; k++)
	{
		dataPacketGraspingPoints.pL[k] = 0;
		dataPacketGraspingPoints.pR[k] = 0;
		dataPacketRobotPose.pos[k] = 0;
		dataPacketRobotPose.rot[k] = 0;
	}
	dataPacketGraspingPoints.validL = 0;
	dataPacketGraspingPoints.validR = 0;
	dataPacketRobotPose.updated = 0;

	// Open UDP interfaces for sending
	if(publisherLiCAS.openInterface(string(IP_ADDRESS_LiCAS), UDP_PORT_LiCAS))
	{
		errorCode = 1;
		cout << "ERROR [in main]: could not open LiCAS publisher." << endl;
	}
	if(publisherMulti.openInterface(string(IP_ADDRESS_MULTIROTOR), UDP_PORT_MULTIROTOR))
	{
		errorCode = 1;
		cout << "ERROR [in main]: could not open Multirotor publisher." << endl;
	}
	
	
	// Create the module interface
	ModuleInterface miAruco(3, string("Aruco_Module"));
	int requestCode = 0;
	int requestOption = 0;
	
	miAruco.openUDPSocket(IP_ADDRESS_ADROM, UDP_PORT_TX_ADROM, UDP_PORT_RX_ADROM);
	
	// Create keyboard thread
	keyboardThread = thread(&keyboardThreadFunction);
	keyboardThread.detach();

	// Init UDP image publisher
	UDPImage udpImage = UDPImage();
	udpImage.init(IP_ADDRESS_VIEWER, UDP_PORT_VIEWER);
	usleep(10000);
		

	cout << "Type [exit] or [quit] to terminate program: ";
		
	// Main loop
	while(errorCode == 0 && endThreadSignal == 0 && inputVideo.grab() && key != 'q')
	{
		// Get initial time stamp
		gettimeofday(&tini, NULL);
		
		// Check module interface
		if(miAruco.isRequestReceived() != 0)
		{
			miAruco.getRequest(requestCode, requestOption);
			if(requestCode == 1)
			{
				// Change marker ID
				targetMarker = (int)requestOption;
			}
			else if(requestCode == 127)
			{
				endThreadSignal = 1;
			}
		}
		
		// Retrieve image
		cv::Mat image, imageCopy;
		inputVideo.retrieve(image);

		vector<int> ids;
		vector<vector<Point2f> > corners, rejected;
	
		// Detect markers and estimate pose
		detector.detectMarkers(image, corners, ids, rejected);
 	
		size_t nMarkers = corners.size();
		vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);
			
		if(!ids.empty())
		{
			// Calculate pose for each marker
			for(size_t i = 0; i < nMarkers; i++)
				cv::solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
		}
			
		image.copyTo(imageCopy);
		if(!ids.empty())
		{
			cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
				
			// Send the data packet with the position of the target marker
			targetMarkedFound = 0;
			for(i = 0; i < ids.size() && targetMarkedFound == 0; i++)
			{
				if(ids[i] == targetMarker)
				{
					targetMarkedFound = 1;
						
					// Convert from camera axes (Z optical, X down, Y right), to X forward, Y left, Z up
					// Apply a correction factor to the position
					float correctionFactor = 0.9;
					markerPosition[0] = 1e-3 * correctionFactor*tvecs[i][2];
					markerPosition[1] = -1e-3 * correctionFactor*tvecs[i][0];
					markerPosition[2] = -1e-3 * correctionFactor*tvecs[i][1];
					markerRotation[0] = 0.0;
					markerRotation[1] = 0.0;
					markerRotation[2] = 0.0;
					updatedFlag = 1;
					// printf("Marker %d XYZ position: {%.1lf, %.1lf, %.1lf} [cm]\n", ids[i], 100*markerPosition[0], 100*markerPosition[1], 100*markerPosition[2]);
					
					// Upate data packets
					for(k = 0; k < 3; k++)
					{
						dataPacketGraspingPoints.pL[k] = markerPosition[k] + pL_handle_offset[k];
						dataPacketGraspingPoints.pR[k] = markerPosition[k] + pR_handle_offset[k];
						dataPacketRobotPose.pos[k] = markerPosition[k];
						dataPacketRobotPose.rot[k] = 0.0;
						dataPacketRobotPose.updated = 1;
					}
					
					// Send data packets
					publisherLiCAS.sendDataPacket((uint8_t*)&dataPacketGraspingPoints, sizeof(DATA_PACKET_GRASPING_POINTS));
					publisherMulti.sendDataPacket((uint8_t*)&dataPacketRobotPose, sizeof(DATA_PACKET_ROBOT_POSE));
					
					dataPacketRobotPose.updated = 0;
				}
			}
				
			// Draw frame axes on each detected marker
			for(int i = 0; i < ids.size(); i++)
				cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerSize * 1.5f, 2);

			// Send the visual feedback, specifying maximum packet size
			if(atoi(argv[3]) == 1)
				udpImage.sendImage(imageCopy, 8192);
		}
		else
		{
			// Send the visual feedback, specifying maximum packet size
			if(atoi(argv[3]) == 1)
				udpImage.sendImage(image, 8192);
		}
			

		// Get final time stamp and compute elapsed time
		gettimeofday(&tend, NULL);
		delta_t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini.tv_usec);
		// printf("Loop time: %.1lf [ms]\n", 1e3*delta_t);
 	
 		// imshow("Detected Markers", imageCopy);
    	// key = cv::waitKey(10);
		if(delta_t < 1.0/UPDATE_RATE)
			usleep((useconds_t)(1e6*(1.0/UPDATE_RATE - delta_t)));
		else
			cout << "WARNING: update period exceeded: " << 1e3*delta_t << " ms" << endl;
	}

	// Close sockets and interfaces
	publisherLiCAS.closeInterface();
	publisherMulti.closeInterface();
	miAruco.closeInterface();
	udpImage.closeSocket();
	

	return errorCode;
}


/* Thread function for reading commands from keyboard */
void keyboardThreadFunction()
{
	string cmd = "";
	
	while(cmd != "exit" && cmd != "quit")
		cin >> cmd;
		
	endThreadSignal = 1;
	
	// Wait for thread termination
	usleep(100000);
}


static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs)
{
	FileStorage fs(filename, FileStorage::READ);
	if(!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	
	return true;
}



