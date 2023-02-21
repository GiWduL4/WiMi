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

#ifndef	MBMCREADMETXTPARSER_H_
#define MBMCREADMETXTPARSER_H_

//#include <stdio.h>
#include <QString>
#include <QByteArray>

#define ONE_BPP_BLACK_PATTERN_INDEX -1 //This info stored at pat_index_info array

#define MBMCREADMEPARSE_ERROR   0
#define MBMCREADMEPARSE_OK      1


class MBMCReadmeTxtParser
{
public:
    MBMCReadmeTxtParser();
    int error;
    int pattern_seq_1bpp_planes;
    int pattern_count;

    QString Exposure;
    QString MinTrigger;
    QString StartVector;
    QString NumOfVectors;
    QString FileName;

    QByteArray pattern_bit_depth;
    QByteArray pattern_color;


    QByteArray pattern_seq_pattern;
    QByteArray pattern_seq_bitplane;

    void ParseReadmeTextFile(QString fileName);
};


#endif /*MBMCREADMETXTPARSER_H_*/
