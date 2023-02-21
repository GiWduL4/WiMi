/**************************************************************************************************************************************************
*                                 Copyright © 2013 Texas Instruments Incorporated - http://www.ti.com/                                            *
***************************************************************************************************************************************************
*  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: *
*                                                                                                                                                 *
*    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.                 *
*                                                                                                                                                 *
*    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the        *
*    documentation and/or other materials provided with the distribution.                                                                         *
*                                                                                                                                                 *
*    Neither the name of Texas Instruments Incorporated nor the names of its contributors may be used to endorse or promote products derived      *
*    from this software without specific prior written permission.                                                                                *
*                                                                                                                                                 *
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     *
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         *
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    *
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                                           *
***************************************************************************************************************************************************/

/*!
 * \file PacketHandler.cpp
 *
 * \brief This conatins the implementations of the methods to handle the tcp socket connections
 * \n and to manage the formation/verification and communication of the packets through this connection.
 */

#include <QtNetwork>
#include <QString>
#include "PacketHandler.h"

static quint16 hostPort = 0x5555;

/*!
 * \brief PacketHandler::PacketHandler
 * \n Initializes the socket and creates connections from the signals to set the connection status message accordingly.
 * \param parent
 */
PacketHandler::PacketHandler(QObject *parent)
{
    tcpSocket = new QTcpSocket(parent); //create new TCP socket

    // Creates new connection to set the string [setErrorString] accordingly
    connect(tcpSocket, SIGNAL(disconnected()), this, SLOT(connectionClose()));

    connect(tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(handleTcpError(QAbstractSocket::SocketError)));

    connect(tcpSocket, SIGNAL(connected()), this, SLOT(connectionOpen()));

    conStatus  = CON_STATUS_NOT_CONNECTED;

    dataLength = 0;
}

void PacketHandler::connectionOpen()
{
    setErrorString("Connection Open", CON_STATUS_CONNECTED);
    dataLength = 0;
}

void PacketHandler::connectionClose()
{
    setErrorString("Connection Closed", CON_STATUS_NOT_CONNECTED);
    dataLength = 0;
}

/*!
 * \brief PacketHandler::handleTcpError
 * \n Displays the TCP error message.
 * \param err
 */
void PacketHandler::handleTcpError(QAbstractSocket::SocketError err)
{
    QString errorString = "Connection Error = "+QString::number(err);
    QByteArray ba = errorString.toLatin1();
    const char *tempStr = ba.data();
    setErrorString(tempStr , CON_STATUS_NOT_CONNECTED);
    dataLength = 0;
}

/*!
 * \brief PacketHandler::connectToBoard
 * \n Handles connection/disconnection to the board.
 * \param enable 1-> connect; 0->disconnect
 */
void PacketHandler::connectToBoard(bool enable)
{
    if(enable)
    {
        if ( tcpSocket->state() == QTcpSocket::UnconnectedState ) {
            tcpSocket->abort(); // Aborts the connection and resets the socket
            tcpSocket->connectToHost(hostIP, hostPort);
            // Wait up to 1 second to ensure connection occurs,
            // otherwise subsequent getVersion call will fail,
            // but do not use a blocking call or the GUI hangs
            tcpSocket->waitForConnected(1000);
            if ( tcpSocket->state() != QTcpSocket::ConnectedState ) {
                tcpSocket->abort(); //Abort if connection is not successful
            }
        }
    }
    else
    {
        tcpSocket->disconnectFromHost(); //disconnect the socket
        tcpSocket->close();// Close the i/o device and reset the socket (resets name, add, port num and socket descriptor)
    }
    dataLength = 0;
}

/*!
 * \brief PacketHandler::sendCommand
 * \n Sets the type of the packet and the command ID and sends the packet.
 * \param type
 * \param cmdId
 * \return
 */
int PacketHandler::sendCommand(quint8 type, quint16 cmdId)
{
    if(conStatus != CON_STATUS_CONNECTED)
    {
        setErrorString("No Connection", CON_STATUS_ERROR);
        return -1;
    }

    if(type != PKT_TYPE_WRITE && type != PKT_TYPE_READ)
    {
        setErrorString("Internal Error", CON_STATUS_ERROR);
        return -1;
    }

    contFlag = 0;
    packetType = type; // packet type
    commandId = cmdId; // 2-byte command id
    dataLength = 0;

    return 0;
}

/*!
 * \brief PacketHandler::readData
 * \n Reads the data from the socket connection
 * \param data - Buffer to read the data
 * \param size - number of bytes to be read
 * \return
 */
int PacketHandler::readData(quint8 *data, int size)
{
    while(size)
    {
        int readLen = tcpSocket->read((char *)data, size);
        if(readLen == 0)
        {
            tcpSocket->waitForReadyRead(-1); // If no data is read, blocks until new data is available for reading
        }
        else if(readLen < 0) //returns -1 if error occurs while reading
            return -1;
        else
        {
            size -= readLen;
            data += readLen;
        }
    }
    return 0;
}

/*!
 * \brief PacketHandler::recvPacket
 * \n Check for the inetgrity of the packet and recieve the packet.
 * \param firstPkt
 * \return
 */
int PacketHandler::recvPacket(bool firstPkt)
{
    dataLength = 0;
    parseIndex = 0;

    if(firstPkt == 0)
    {
        if(recvFlag == 0 /*single packet*/ ||  recvFlag == 3 /*last packet*/)
        {
            setErrorString("Packet Continuation Error 1", CON_STATUS_ERROR);
            return -1;
        }
    }

    //Read the packet header (6 bytes)
    if(readData(packetBuffer, HEADER_SIZE))
    {
        setErrorString("Command Header Read Failed", CON_STATUS_ERROR);
        return -1;
    }

    //Payload length (data length excluding checksum)
    dataLength = packetBuffer[4] | packetBuffer[5] << 8;

    if(readData(packetBuffer + HEADER_SIZE, dataLength + 1))
    {
        setErrorString("Command Data Read Failed", CON_STATUS_ERROR);
        return -1;
    }

    //Calculate the checksum and compare it with the checksum recieved in the packet
    if(packetBuffer[dataLength + HEADER_SIZE] != calcChecksum())
    {
        setErrorString("RX Checksum error", CON_STATUS_ERROR);
        return -1;
    }

    if(packetBuffer[0] != packetType + 1)
    {
        setErrorString("Command FAILED" /*+ (packetBuffer[0]) + (packetType + 1)*/, CON_STATUS_ERROR);
        return -1;
    }

    //Flag to denote whether the packet is first, last, intermediate or the only packet.
    recvFlag = packetBuffer[3];

    if(firstPkt != (recvFlag == 0 || recvFlag == 1))//flag = 0 ->only one packet ; flag = 1 ->first packet
    {
        setErrorString("Packet Continuation Error", CON_STATUS_ERROR);
        return -1;
    }

    if(recvFlag == 3)//last packet recieved
    {
        setErrorString("Command SUCCESS");
    }

    return 0;
}

/*!
 * \brief PacketHandler::sendPacket
 * \n Prepare and send the packet.
 * \param more
 * \return
 */
int PacketHandler::sendPacket(bool more)
{
    quint8 flag;
    static int count;
    char str[] = "Command Processing..........";

    if(conStatus != CON_STATUS_CONNECTED)
    {
        setErrorString("No Connection", CON_STATUS_ERROR);
        return -1;
    }

    if(contFlag == 0 || count == 10)
    {
        count = 0;
    }
    else
    {
        count++;
    }
    //str[sizeof(str) - 10 + count] = 0;
    str[sizeof(str) - 8 + count] = 0;

    setErrorString(str);

    packetBuffer[0] = packetType;

    //Command ID
    packetBuffer[1] = (commandId >> 8) & 0xFF;
    packetBuffer[2] = (commandId) & 0xFF;

    if(contFlag)
    {
        if(more)
            flag = 2;      //intermediate packet
        else
            flag = 3;      //last packet
    }
    else
    {   if(more)
            flag = 1;      //First packet
        else
            flag = 0;      //Single packet
    }
    contFlag = more;

    packetBuffer[3] = flag;

    //Payload length (excluding checksum)
    packetBuffer[4] = dataLength & 0xFF;
    packetBuffer[5] = (dataLength >> 8) & 0xFF;

    packetBuffer[HEADER_SIZE + dataLength] = calcChecksum(); //Calculate checksum and store in the last byte

    int length = dataLength + HEADER_SIZE + CHECKSUM_SIZE;

    //Send the packet to the tcp socket connection
    if(tcpSocket->write((char *)packetBuffer, length) != length)
    {
        setErrorString("Write Failed", CON_STATUS_ERROR);
        return -1;
    }

    if(recvPacket(1))
        return -1;

    if(more == 0 && recvFlag == 0)
    {
        setErrorString("Command SUCCESS");
    }

    return 0;
}

/*!
 * \brief PacketHandler::putData
 * \n Add the data to the packet.
 * \param data - Data to be put into the packet.
 * \param size - Size of the data to be put.
 * \return - 0 if successful
 */
int PacketHandler::putData(QString data, int size)
{
    if(size < 0)
        size = data.size();

    if(dataLength + size > MAX_PACKET_SIZE)
        return -1;

    if(size)
    {
        int i;
        for(i = 0; i < data.size(); i++)
        {
            packetBuffer[HEADER_SIZE + dataLength + i] = data.at(i).toLatin1();
        }

        while(i < size)
        {
            packetBuffer[HEADER_SIZE + dataLength + i] = 0;
            i++;
        }
        dataLength += size;
    }

    return 0;
}

/*!
 * \brief PacketHandler::putData
 * \n Add the data to the packet.
 * \param data - Data to be put into the packet.
 * \param size - Size of the data to be put.
 * \return - 0 if successful
 */
int PacketHandler::putData(char *data, int size)
{
    if(dataLength + size > MAX_PACKET_SIZE)
        return -1;

    if(size && data)
    {
        for(int i = 0; i < size; i++)
            packetBuffer[HEADER_SIZE + dataLength + i] = data[i];
        dataLength += size;
    }

    return 0;
}

/*!
 * \brief PacketHandler::putData
 * \n Add the data to the packet.
 * \param data - Data to be put into the packet.
 * \param size - Size of the data to be put.
 * \return - 0 if successful
 */
int PacketHandler::putData(quint8 *data, int size)
{
    if(dataLength + size > MAX_PACKET_SIZE)
        return -1;

    if(size && data)
    {
        memcpy(packetBuffer + HEADER_SIZE + dataLength, data, size);
        dataLength += size;
    }

    return 0;
}

/*!
 * \brief PacketHandler::putData
 * \n Add the data to the packet.
 * \param value - Data to be put into the packet.
 * \param size - Size of the data to be put.
 * \return
 */
int PacketHandler::putData(quint32 value, int size)
{
    if(size > 4 || size < 1)
        return -1;

    if(dataLength + size > MAX_PACKET_SIZE)
        return -1;

    for(int i = 0; i < size; i++)
    {
        packetBuffer[HEADER_SIZE + dataLength++] = value&0xFF;
        value >>= 8;
    }
    return 0;
}

int PacketHandler::getBytes(quint8 *data, int size)
{
    return getBytes((char*)data, size);
}

/*!
 * \brief PacketHandler::getBytes
 * \n Gets the data in bytes.
 * \param data - Buffer to get the data
 * \param size -  Number of bytes to be read
 * \return
 */
int PacketHandler::getBytes(char *data, int size)
{
    if(conStatus != CON_STATUS_CONNECTED)
    {
        setErrorString("No Connection", CON_STATUS_ERROR);
        return -1;
    }

    if(packetBuffer[0] != PKT_TYPE_READ_RESP)
    {
        setErrorString("No Read Response", CON_STATUS_ERROR);
        return -1;
    }

    while(size)
    {
        int copySize = dataLength - parseIndex;

        if(copySize == 0)
        {
            if(recvPacket(0))
                return -1;

            copySize = dataLength - parseIndex;
        }

        if(copySize >= size)
        {
            copySize = size;
        }

        memcpy(data, packetBuffer + HEADER_SIZE + parseIndex, copySize);
        parseIndex += copySize;
        size -= copySize;
        data += copySize;
    }

    return 0;
}

quint32 PacketHandler::getInt(int size)
{
    quint32 value = 0;
    quint8 data[4];

    if(size > 4)
        return 0;

    if(getBytes(data, size))
        return 0;

    for(int i = 0; i < size; i++)
    {
        value |= data[i] << (i * 8);
    }

    return value;
}

/*!
 * \brief PacketHandler::remainingResponse
 * \return Number of bytes remaining to be read.
 */
quint32 PacketHandler::remainingResponse(void)
{
    return dataLength - parseIndex;
}

/*!
 * \brief PacketHandler::calcChecksum
 * \n Calculate the checksum of the data in the packet.
 * \n Checksum = (Byte 0 + Byte 1 + … Byte N) MODULO[ 0x100 ]
 * \return - Checksum value
 */
quint8 PacketHandler::calcChecksum(void)
{
    int i;
    int sum = 0;

    for(i = 0; i < dataLength + HEADER_SIZE; i++)
    {
        sum += packetBuffer[i];
    }
    return (quint8)(sum & 0xFF);
}

void PacketHandler::setErrorString(char const *error, ConnectionStatus status)
{
    if(status == CON_STATUS_CONNECTED || status == CON_STATUS_NOT_CONNECTED)
        conStatus = status;

    emit setStatus(error, status);
}
