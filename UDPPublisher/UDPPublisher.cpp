/*
 * UDPPublisher.CPP
 * 
 * Author: Alejandro Suarez, asuarezfm@us.es
 * date: November 2024
 */

#include "UDPPublisher.h"


/*
 * Constructor
 */
UDPPublisher::UDPPublisher()
{
    socketPublisher = -1;
}


/*
 * Destructor
 */
UDPPublisher::~UDPPublisher()
{
}



/* Open interface */
int UDPPublisher::openInterface(const string & _hostIP_address, int _hostUDP_TxPort)
{
	int errorCode = 0;
	
	
	// Open socket in datagram mode
	socketPublisher = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(socketPublisher < 0)
    {
    	cout << endl << "ERROR [in UDPPublisher::openInterface]: could not open socket." << endl;
    	errorCode = 1;
   	}

   	host = gethostbyname(_hostIP_address.c_str());
    if(host == NULL)
	{
	    cout << "ERROR [in UDPPublisher::openInterface]: could not get host by name" << endl;
	    errorCode = 1;
	}
	
	// Set the address of the host
	if(errorCode == 0)
	{
		bzero((char*)&addrHost, sizeof(struct sockaddr_in));
		addrHost.sin_family = AF_INET;
		bcopy((char*)host->h_addr, (char*)&addrHost.sin_addr.s_addr, host->h_length);
		addrHost.sin_port = htons(_hostUDP_TxPort);
	}
	else
	{
		close(socketPublisher);
		socketPublisher = -1;
	}
		
	
	return errorCode;
}

	
/* Send data packet */
int UDPPublisher::sendDataPacket(uint8_t * _dataPacketPointer, size_t _dataPacketSize)
{
	int bytesSent = 0;
	int errorCode = 0;
	
	
	// Send the data packet
	bytesSent = sendto(socketPublisher, (char*)_dataPacketPointer, _dataPacketSize, 0, (struct sockaddr*)&addrHost, sizeof(struct sockaddr));
	if(bytesSent != _dataPacketSize)
	{
		errorCode = 1;
		cout << "ERROR [in UDPPublisher::sendDataPacket]: could not send data through socket." << endl;
		cout << "Bytes sent: " << bytesSent << " Expected: " << _dataPacketSize << endl;
	}
	
	
	return errorCode;
}

	
/* Close interface */
void UDPPublisher::closeInterface()
{
	if(socketPublisher >= 0)
	{
		close(socketPublisher);
    	socketPublisher = -1;
	}
}


