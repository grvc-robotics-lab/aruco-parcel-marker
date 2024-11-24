/*
 * Copyright (c) 2024 Alejandro Suarez, asuarezfm@us.es
 *
 * University of Seville - Robotics, Vision and Control Group
 *
 * Date: November 2024
 */

#ifndef MODULEINTERFACE_H_
#define MODULEINTERFACE_H_


// Standard library
#include <iostream>
#include <thread>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>


// Constant definition
#define MAX_SIZE_USER_BUFFER	32


using namespace std;


class ModuleInterface
{
public:
	
	typedef struct
	{
		uint8_t header[3];		// "MSG" or "REQ" character sequence for messages and requests
		uint8_t moduleID;		// Module identifier
		uint8_t hostID;			// Host identifier
		uint8_t msgCode;		// Message code
		uint8_t msgOption;		// Message option
		uint8_t userBuffer[MAX_SIZE_USER_BUFFER];
		uint16_t userBufferSize;
		uint8_t checksum;		// XOR of all bytes except header and checksum
	} __attribute__((packed)) DATA_PACKET_MODULE_MSG;

	/***************** PUBLIC VARIABLES *****************/

	
	/***************** PUBLIC METHODS *****************/
	
	/*
	 * Constructor
	 * */
	ModuleInterface(uint8_t _moduleID, const string &_moduleInterfaceName);

	/*
	 * Destructor
	 * */
	virtual ~ModuleInterface();
	
	/*
	 * Open the UDP socket interface for sending/receiving data to/from the host.
	 */
	int openUDPSocket(const string &_hostIP_Address, int _hostUDP_TxPort, int _hostUDP_RxPort);
	
	/*
	 * Send a message to the host, including a pointer to optional user data to be sent.
	 */
	int sendMessage(uint8_t _msgCode, uint8_t _msgOption, uint8_t * _bufferData, size_t _bufferDataSize);
	
	/*
	 * Return 1 if a request was received from maanger, and zero in other case. 
	 */
	int isRequestReceived();
	
	/*
	 * Return the request code when received.
	 */
	void getRequest(int & _requestCode, int & _requestOption);
	
	/*
	 * Close the UDP socket interface.
	 */
	int closeInterface();
	

private:
	/***************** PRIVATE VARIABLES *****************/
	string moduleInterfaceName;
	
	thread udpRxThread;
	
	struct sockaddr_in addrHost;
    struct hostent * host;
	string hostIP_Address;
	int hostUDP_TxPort;
	int hostUDP_RxPort;
	int socketPublisher;
	
	uint8_t moduleID;
	uint8_t requestCode;
	uint8_t requestOption;
	int flagRequestReceived;
	int flagTerminateThread;
	int flagThreadTerminated;
	
	
	/***************** PRIVATE METHODS *****************/
	
	
	void udpRxThreadFunction();
	

};

#endif


