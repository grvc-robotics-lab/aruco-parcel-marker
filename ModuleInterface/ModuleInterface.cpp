/*
 * Copyright (c) 2024 Alejandro Suarez, asuarezfm@us.es
 *
 * University of Seville - Robotics, Vision and Control Group
 *
 * Date: November 2024
 */

#include "ModuleInterface.h"


/*
 * Constructor
 * */
ModuleInterface::ModuleInterface(uint8_t _moduleID, const string &_moduleInterfaceName)
{
	this->moduleID = _moduleID;
	this->moduleInterfaceName = _moduleInterfaceName;
	
	// Init variables
	this->hostIP_Address = "";
	this->hostUDP_TxPort = -1;
	this->hostUDP_RxPort = -1;
	this->socketPublisher = -1;
	
	this->moduleID = 0;
	this->flagRequestReceived = 0;
	this->flagTerminateThread = 0;
	this->flagThreadTerminated = 0;
}


/*
 * Destructor
 * */
ModuleInterface::~ModuleInterface()
{
}


/*
 * Open the UDP socket interface for sending/receiving data to/from the manager.
 */
int ModuleInterface::openUDPSocket(const string &_hostIP_Address, int _hostUDP_TxPort, int _hostUDP_RxPort)
{
	int errorCode = 0;
	
	
	// Open the UDP socket for sending requests to the modules
	this->socketPublisher = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(socketPublisher < 0)
    {
    	errorCode = 1;
    	cout << endl << "ERROR: [in ModuleInterface::openUDPSocket] could not open socket." << endl;
   	}
   	else
	{
	   	host = gethostbyname(_hostIP_Address.c_str());
	    if(host == NULL)
		{
		    errorCode = 2;
			close(socketPublisher);
		    cout << "ERROR: [in ModuleInterface::openUDPSocket] could not get host by name" << endl;
		}
		else
		{
			// Set the address of the host
			bzero((char*)&addrHost, sizeof(struct sockaddr_in));
			this->addrHost.sin_family = AF_INET;
			bcopy((char*)host->h_addr, (char*)&addrHost.sin_addr.s_addr, host->h_length);
			this->addrHost.sin_port = htons(_hostUDP_TxPort);
		}
	}
	
	if(errorCode == 0)
	{
		// Copy the IP and UDP ports
		this->hostIP_Address = _hostIP_Address;
		this->hostUDP_TxPort = _hostUDP_TxPort;
		this->hostUDP_RxPort = _hostUDP_RxPort;
		
		// Init the thread for receiving the requests from the manager
		udpRxThread = thread(&ModuleInterface::udpRxThreadFunction, this);
		udpRxThread.detach();
	}
	
	
	return errorCode;
}
	

/*
 * Send a message to the host, including a pointer to optional user data to be sent.
 */
int ModuleInterface::sendMessage(uint8_t _msgCode, uint8_t _msgOption, uint8_t * _bufferData, size_t _bufferDataSize)
{
	DATA_PACKET_MODULE_MSG dataPacketMsg;
	int bytesSent = 0;
	int k = 0;
	int errorCode = 0;
	
	// Set the fields of the request data packet
	dataPacketMsg.header[0] = 'M';
	dataPacketMsg.header[1] = 'S';
	dataPacketMsg.header[2] = 'G';
	dataPacketMsg.moduleID = this->moduleID;
	dataPacketMsg.msgCode = _msgCode;
	dataPacketMsg.msgOption = _msgOption;
	if(_bufferData != NULL)
	{
		if(_bufferDataSize > MAX_SIZE_USER_BUFFER)
		{
			cout << "WARNING: buffer data exceeds limit, max: " << MAX_SIZE_USER_BUFFER << " bytes allowd." << endl;
			dataPacketMsg.userBufferSize = 0;
		}
		else
		{
			for(k = 0; k < _bufferDataSize; k++)
				dataPacketMsg.userBuffer[k] = _bufferData[k];
			dataPacketMsg.userBufferSize = (uint16_t)_bufferDataSize;
		}
	}
	// TODO: compute checksum
	
	// Send the request data packet
	bytesSent = sendto(this->socketPublisher, (char*)&dataPacketMsg, sizeof(DATA_PACKET_MODULE_MSG), 0, (struct sockaddr*)&addrHost, sizeof(struct sockaddr));
	if(bytesSent < 0)
	{
		errorCode = 1;
		cout << "ERROR: [in ModuleInterface::sendMessage] could not send data packet." << endl;
	}
	else if(bytesSent != sizeof(DATA_PACKET_MODULE_MSG))
	{
		errorCode = 1;
		cout << "ERROR: [in ModuleInterface::sendMessage] incorrect number packet." << endl;
	}
	
	
	return errorCode;
}


/*
 * Return 1 if signal was received from module.
 */
int ModuleInterface::isRequestReceived()
{	
	return this->flagRequestReceived;
}

	
/*
 * Return the signal code when received (zero by default).
 */
void ModuleInterface::getRequest(int & _requestCode, int & _requestOption)
{
	_requestCode = this->requestCode;
	_requestOption = this->requestOption;
}


void ModuleInterface::udpRxThreadFunction()
{
	DATA_PACKET_MODULE_MSG * dataPacketRequest;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	int error = 0;
	
	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		error = 1;
		cout << endl << "ERROR: [in ModuleInterface::udpRxThreadFunction] could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(this->hostUDP_RxPort);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		error = 1; 
		cout << endl << "ERROR: [in ModuleInterface::udpRxThreadFunction] could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}

	/******************************** THREAD LOOP START ********************************/

	while(error == 0 && flagTerminateThread == 0)
	{	
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if (dataReceived == sizeof(DATA_PACKET_MODULE_MSG))
		{
			// Check if message is correct
			if(buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'Q')
			{
				// Get the signal from the data packet
				dataPacketRequest = (DATA_PACKET_MODULE_MSG*)buffer;
				
				// Get the request data
				this->requestCode = dataPacketRequest->msgCode;
				this->requestOption = dataPacketRequest->msgOption;
				
				// Set the request received flag
				this->flagRequestReceived = 1;
			}
		}
		else
		{
			// Wait 10 ms
			usleep(10000);
		}
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketReceiver);
	this->flagThreadTerminated = 1;
}

	
/*
 * Close the UDP socket interface
 */
int ModuleInterface::closeInterface()
{
	int errorCode = 0;


	this->flagTerminateThread = 1;
	usleep(10000);	// Waits 10 ms to termiante thread

	// Close publisher socket
	close(this->socketPublisher);

	while(this->flagThreadTerminated == 0)
		usleep(10000);
	
	return errorCode;
}

