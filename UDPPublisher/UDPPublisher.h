/*
 * UDPPublisher.h
 *
 * Author: Alejandro Suarez, asuarezfm@us.es
 * Date: November 2024
 */


#ifndef UDPPUBLISHER_H_
#define UDPPUBLISHER_H_

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

// Specific library


// Constants


// Namespaces
using namespace std;


class UDPPublisher
{
public:
	/**************** PUBLIC VARIABLES *****************/
	

	/***************** PUBLIC METHODS *****************/

	/* Constructor */
	UDPPublisher();

	/* Destructor */
	~UDPPublisher();

	/* Open interface */
	int openInterface(const string & _hostIP_address, int _hostUDP_TxPort);
	
	/* Send data packet */
	int sendDataPacket(uint8_t * _dataPacketPointer, size_t _dataPacketSize);
	
	/* Close interface */
	void closeInterface();


private:
	/***************** PRIVATE VARIABLES *****************/

    int socketPublisher;
	struct sockaddr_in addrHost;
    struct hostent * host;


	/***************** PRIVATE METHODS *****************/



};

#endif

