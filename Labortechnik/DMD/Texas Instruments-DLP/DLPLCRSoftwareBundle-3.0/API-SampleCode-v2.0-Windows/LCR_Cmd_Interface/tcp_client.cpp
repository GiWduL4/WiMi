/*****************************************************************************
**             TEXAS INSTRUMENTS PROPRIETARY INFORMATION
**
**  (c) Copyright, Texas Instruments Incorporated, 2012-2013.
**      All Rights Reserved.
**
**  Property of Texas Instruments Incorporated. Restricted Rights -
**  Use, duplication, or disclosure is subject to restrictions set
**  forth in TI's program license agreement and associated documentation.
******************************************************************************/
/**
*
* @file    tcp_client.c
*
* @brief	TCP implementation using <system supported function > APIs.
**/
/*****************************************************************************/


/*Include header files related to TCP/IP communication */
#include "stdafx.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

#pragma comment(lib, "Ws2_32.lib")

WSADATA wsaData;

/*WinSocket API includes */

#include "tcp_client.h"


int TCP_Connect(char *host, unsigned long int port)
{

	//int socket = 0;
	//1. Declare and Initialize TCP/IP module
	//2. Create a socket for connecting to server
	//3. Connect to server.
	//return socket;

	int iResult;
	char charPort[33];
	char *chPort;

	//Convert the port number into ascii char 
	chPort = _ultoa(port,&charPort[0],10);

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
		return -1;
	}

	//Declare addrinfo object that contains a socketaddr strcuture and initialize these values
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;
	ZeroMemory( &hints, sizeof(hints) );
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(host, chPort, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
		return -1;
	}

	// Attempt to connect to the first address returned by
	// the call to getaddrinfo
	ptr=result;

	// Create a SOCKET for connecting to server
	SOCKET ConnectSocket = INVALID_SOCKET;

	ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
		ptr->ai_protocol);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Error at socket(): %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return -1;
	}


	// Connect to server.
	iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		closesocket(ConnectSocket);
		ConnectSocket = INVALID_SOCKET;
	}

	// Should really try the next address returned by getaddrinfo
	// if the connect call failed
	// But for this simple example we just free the resources
	// returned by getaddrinfo and print an error message

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return -1;
	}

	return ConnectSocket;
}

int TCP_Send(int sock, unsigned char *buffer, int length)
{
	// Send 'length' number of bytes from buffer via provided
	// socket <sock> address
	int iResult;
	iResult = send(sock,(char*) buffer,length,0);
	if (iResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(sock);
		WSACleanup();
		return 1;
	}

	//printf("Bytes Sent: %ld\n", iResult);
	return 0;
}

int TCP_Receive(int sock, unsigned char *buffer, int length)
{
	//Retrieve 'length' number of bytes into 'buffer' from the socket <sock> address
	int iResult = 0;

	while(length)
	{
		iResult = recv(sock, (char*) buffer, length, 0);

		if ( iResult > 0 )
			; //printf("Bytes received: %d\n", iResult);
		else if ( iResult == 0 )
			printf("Connection closed\n");
		else
			printf("recv failed with error: %d\n", WSAGetLastError());

		length -= iResult;
		buffer += iResult;
	}

	return 0;
}

int TCP_Disconnect(int sock)
{
	//free and cleanup socket occupied memory
	closesocket(sock);
	WSACleanup();

	return 0;
}
