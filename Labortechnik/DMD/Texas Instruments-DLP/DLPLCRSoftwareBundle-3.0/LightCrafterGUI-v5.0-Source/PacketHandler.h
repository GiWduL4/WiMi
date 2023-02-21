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
  * \file PacketHandler.h
  *
  * \brief Declares the PacketHandler class which manages the tcp connections and
  * \n formation/verification and communication of the packets on this connection.
  */

#ifndef PACKETHANDLER_H
#define PACKETHANDLER_H

#include <QTcpSocket>

/*!
 * \brief The PacketHandler class
 * \n Handles the communication of the packets with the TCP socket connection.
 * \n Also manages the packet formation and verification.
 * \n Handles sending and recieving of the data in the specified packet format.
 *
 * \n Packet format
 * \n ------------------------------------------------------------------------------------
 * \n |                        HEADER                       |       DATA      | CHECKSUM |
 * \n ------------------------------------------------------------------------------------
 * \n | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6...Byte N | Byte N+1 |
 * \n ------------------------------------------------------------------------------------
 * \n |Pkt type| CMD 1  | CMD 2  | Flags  | Payload length  |  Data payload   | Checksum |
 * \n ------------------------------------------------------------------------------------
 * \n
 */
class PacketHandler : public QObject
{
    Q_OBJECT
public:

    enum { HEADER_SIZE = 6,
           MAX_PACKET_SIZE = 0xFFFF,
           CHECKSUM_SIZE = 1 };

    enum PacketType { PKT_TYPE_BUSY,
           PKT_TYPE_ERROR,
           PKT_TYPE_WRITE,
           PKT_TYPE_WRITE_RESP,
           PKT_TYPE_READ,
           PKT_TYPE_READ_RESP
       };

    enum ConnectionStatus {
        CON_STATUS_NONE,
        CON_STATUS_NOT_CONNECTED,
        CON_STATUS_CONNECTED,
        CON_STATUS_ERROR
    };


private:
    QString hostIP;
    ConnectionStatus conStatus;
    QTcpSocket *tcpSocket;
    quint8 packetBuffer[HEADER_SIZE + MAX_PACKET_SIZE + CHECKSUM_SIZE];
    quint8 packetType;
    quint8 contFlag;
    quint8 recvFlag;
    quint16 commandId;
    quint16 dataLength;
    quint16 parseIndex;


    quint8 calcChecksum(void);
    int sendCommand(quint8 type, quint16 cmdId);
    int readData(quint8 *data, int size);
    int recvPacket(bool firstPkt);
signals:
    void setStatus(const char *str, PacketHandler::ConnectionStatus status);

public:
    PacketHandler(QObject *parent=0);
    void setErrorString(char const *error, ConnectionStatus status = CON_STATUS_NONE);
    void connectToBoard(bool enable = true);
    void setIP(QString IP)
    {
        hostIP = IP;
    }

    QString getIP(void)
    {
        return hostIP;
    }

    quint32 remainingResponse(void);

    int getDataLength(void)
    {
        return dataLength;
    }

    int responseSize(void)
    {
        return dataLength;
    }

    ConnectionStatus connectionStatus(void)
    {
        return conStatus;
    }

    int sendPacket(bool more = 0);
    int putData(QString data, int size = -1);
    int putData(char *data, int size);
    int putData(quint8 *data, int size);
    int putData(quint32 value, int size);
    int writeCommand(quint16 cmdId)
    {
        return sendCommand(PKT_TYPE_WRITE, cmdId);
    }
    int readCommand(quint16 cmdId)
    {
        return sendCommand(PKT_TYPE_READ, cmdId);
    }
    quint32 getInt(int size);
    int getBytes(quint8 *data, int size);
    int getBytes(char *data, int size);

private slots:
    void connectionClose();
    void connectionOpen();
    void handleTcpError(QAbstractSocket::SocketError err);
};


#endif // PACKETHANDLER_H
