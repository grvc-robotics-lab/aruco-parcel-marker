/*
 * UDPImage.h
 *
 * Class for JPEG encoded image send through UDP socket
 * 
 * Author: Alejandro Suarez, asuarezfm@us.es
 * Date: January 2014
 * Revision date: November 2024
 */


#ifndef UDPIMAGE_H_
#define UDPIMAGE_H_

// Standard library
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// Specific library


// Constants
#define ENCODED_IMAGE_JPEG_QUALITY	15	// 25% quality


// Namespaces
using namespace std;
using namespace cv;


// Structure definition
typedef struct
{
	char headerStr[8];		// "JPEG_IMG" character sequence
	int imgSize;			// Size in bytes of the image
	int packetSize;			// Size in bytes of the packets
	uint16_t checksum;		// 16-bit XOR checksum applied to the bytes of the encoded image
} __attribute__((packed)) DATA_PACKET_ENCODED_IMAGE;


class UDPImage
{
public:
	/**************** PUBLIC VARIABLES *****************/
	

	/***************** PUBLIC METHODS *****************/

	/* Constructor */
	UDPImage();

	/* Destructor */
	~UDPImage();

	/* Initializes socket */
	int init(const char * hostName, int port);

	/* Send the image passed as argument throught the socket */
	int sendImage(const cv::Mat &img, int packetLength);

	/* Close the socket */
	void closeSocket();



private:
	/***************** PRIVATE VARIABLES *****************/

    struct sockaddr_in addrServer;
    struct hostent * server;
    int socketClient;
    int portNum;


	/***************** PRIVATE METHODS *****************/

	/* Compute the 16 bit checksum applying XOR operation over image buffer */
	uint16_t computeChecksum(const vector<uchar> &imgBuffer);
	
	/* Send the encoded image contained in imgBuffer using the client socket. The maximum size of the UDP
	 * datagram is specified with the third argument. */
	int sendImageInPackets(const vector<uchar> &imgBuffer, int packetLength);


};

#endif

