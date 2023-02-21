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

#include <stdio.h>
#include <ctype.h>
#include <QIODevice>
#include <QString>
#include <QFile>
#include <QList>
#include <QStringList>


#include "MBMCReadmeTxtParser.h"


MBMCReadmeTxtParser::MBMCReadmeTxtParser():pattern_seq_1bpp_planes(0),pattern_count(0)
{
}

// This function parses the AUTO generated Multiple bit depth and mulitple color pattern Sequence Readme file
void MBMCReadmeTxtParser::ParseReadmeTextFile(QString fileName)
{
    int i;
    int tmpPatterns;

    int tmpBitDepth;
    int tmpColor;
    int tokenItemIndex;

    QString tmpBitPlane;
    QString     line;
    QString     tmpPat;
    QString     tmpString;
    QByteArray  tokenItem;
    QStringList tokenLine;
    QFile       TxtFile(fileName);

    // Reset pattern count
    pattern_count = 0;
    pattern_seq_1bpp_planes = 0;
    pattern_bit_depth.clear();
    pattern_color.clear();
    pattern_seq_pattern.clear();
    pattern_seq_bitplane.clear();

    // Open file for read access
    if(!TxtFile.open(QIODevice::ReadOnly))
    {
        error = MBMCREADMEPARSE_ERROR;
        return;
    }

    FileName = fileName;


    while(!TxtFile.atEnd())
	{
        // Read in a line
        line = TxtFile.readLine();

        //Parse line into the string array
        tokenLine = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);

        // Check if line contains exposure time
        if(line.contains("#   Exposure time       :"))
        {
            // Extract exposure time
            Exposure = tokenLine.at(4);
        }

        // Check if line contains minimum trigger time
        if(line.contains("#   Trig Period         :"))
        {
            // Extract exposure time
            MinTrigger = tokenLine.at(5);
        }

        // Check if line contains sequence equation
        if(line.contains("1. Sequence Equation:"))
        {
            // Extract sequence equation
            tokenItemIndex  = 0;
            pattern_count   = 0;
            tmpString = tokenLine.at(3);

            // Remove period from end of equation
            tmpString.remove(tmpString.length()-5-Exposure.length(),5+Exposure.length());

            // Convert equation to QByteArray
            tokenItem = tmpString.toUtf8();

            while(tokenItemIndex < tokenItem.length())
            {
                // Read the bpp
                tmpBitDepth = tokenItem.at(tokenItemIndex)-48;

                // Read the number of similar patterns
                tokenItemIndex = tokenItemIndex + 5;
                tmpPatterns = 0;

                // Read first digit of pattern number
                tmpPatterns = tokenItem.at(tokenItemIndex)-48;
                tokenItemIndex++;

                if(tokenItem.at(tokenItemIndex) != 'R' && tokenItem.at(tokenItemIndex) != 'G' && tokenItem.at(tokenItemIndex) != 'B')
                {
                    // Convert ASCII digit to INT and add to tmpPatterns
                    tmpPatterns      = (10*tmpPatterns) + (tokenItem.at(tokenItemIndex)-48);
                    tokenItemIndex++;
                }

                // Increment the pattern count
                pattern_count = pattern_count + tmpPatterns;

                // Read the color
                tmpColor = tokenItem.at(tokenItemIndex);

                // Enter the number of patterns into the sequence
                for(i=0;i<tmpPatterns;i++)
                {
                    pattern_bit_depth.append(tmpBitDepth);
                    pattern_color.append(tmpColor);
                }

                // Go to next pattern
                tokenItemIndex++;
                tokenItemIndex++;
            }
        }

        // Check if line contains vector information
        if(line.contains("2. Enable Mulitple Bit Depth and Multiple Color Pattern Sequence using TCP command (0x0A 0x01)"))
        {
            // Extract StartVector
            StartVector = tokenLine.at(18);

            // Remove the comma
            StartVector.remove(StartVector.length()-1,1);

            // Extract NumofVectors
            NumOfVectors = tokenLine.at(21);
        }

        // Check if line marks the begining of the 1bpp pattern sequence
        if(line.contains("Table-1: Split Individual binary patterns placement order"))
        {
            pattern_seq_1bpp_planes = 0;

            // Read three lines to get to begining of information
            line = TxtFile.readLine();
            line = TxtFile.readLine();
            line = TxtFile.readLine();

            // For the total number of patterns rean in the pattern and bit plane information
            while(!TxtFile.atEnd())
            {
                // Read the line with pattern sequence information
                line = TxtFile.readLine();

                // Check if it should be a black filler
                if( line.contains("INSERT_1BPP_BLACK_PATTERN"))
                {
                    pattern_seq_pattern.append(ONE_BPP_BLACK_PATTERN_INDEX);
                    pattern_seq_bitplane.append(ONE_BPP_BLACK_PATTERN_INDEX);
                }
                else
                {
                    //Parse line into the string array
                    tokenLine = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);

                    // Exptract pattern number
                    tmpPat = tokenLine.at(2);
                    pattern_seq_pattern.append((char) tmpPat.toInt());

                    // Extract bitplane
                    tmpBitPlane = tokenLine.at(3);
                    tmpBitPlane.remove(0,3);        // remove "BIT" from string
                    pattern_seq_bitplane.append((char) tmpBitPlane.toInt());
                }
                pattern_seq_1bpp_planes++;

                // Read blank line
                if(!TxtFile.atEnd()) line = TxtFile.readLine();
            }
        }
	}
    // Close the file
    TxtFile.close();
    error = MBMCREADMEPARSE_OK;

    return;
}	
