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

#include "bmpparser.h"
#include <QString>
#include <QFile>
#include <QMessageBox>
#include <QImage>



void BMPParser::blankImage(QString fileName)
{
    QImage blank1bpp(608,684,QImage::Format_Mono);

    // Fill with zeros
    blank1bpp.fill(0);

    // Save the image
    blank1bpp.save(fileName,"BMP");

    return;
}


int BMPParser::LoadBMP(QString fileName, int bitDepth)
{
    QFile   bmpFile(fileName);

    // Check that file exists
    if(bmpFile.exists())
    {
        // Load the bmp file
        bmpOriginal.fill(0);
        bmpOriginal.load(fileName);

        // Check image resolutions
        if(bmpOriginal.height()!=684 && bmpOriginal.width()!=608)
        {
            error = BMPPARSER_ERROR_SIZE;
            return error;
        }

        // Check image format
        if((bmpOriginal.format() == QImage::Format_Indexed8) ||
           (bmpOriginal.format() == QImage::Format_Mono))
        {
            bmpBitDepth = bitDepth;
            error = BMPPARSER_LOADED;
            return error;
        }
        else
        {
            error = BMPPARSER_ERROR_FORMAT;
            return error;
        }
    }
    else
    {
        error = BMPPARSER_ERROR_FILE;
        return error;
    }
}

int BMPParser::Parse()
{
    uint    i,x,y;
    uint    bitPlaneMask = 0x1;
    uint    pixel;

    // Check BMP has been loaded and meets image requirements
    if(error == BMPPARSER_LOADED)
    {
        // Initialize bitplane Qimages
        for(i=0;i<8;i++)
        {
            bmpBitPlanes[i] = QImage(608,684,QImage::Format_Mono);
        }


        // Parse the image into 1bpp bitplanes according to bit depth
        // For each pixel
        for(x=0;x<608;x++)
        {
            for(y=0;y<684;y++)
            {
                // Read pixel value
                pixel = bmpOriginal.pixelIndex(x,y);

                // Send masked value to bitplane
                for(i=0;i<bmpBitDepth;i++)
                {
                    // Mask the pixel if not 1bpp already
                    if(bmpOriginal.format() == QImage::Format_Mono)
                    {
                        bmpBitPlanes[i].setPixel(x,y, !pixel);
                    }
                    else
                    {
                        bmpBitPlanes[i].setPixel(x,y, pixel & bitPlaneMask);

                        // Shift pixel for next bitplane
                        pixel = pixel >> 1;
                    }


                }
            }
        }
        error = BMPPARSER_SUCCESS;
        return error;
    }
    else
    {
        error = BMPPARSER_ERROR_NOTLOADED;
        return error;
    }
    return error;
}

bool BMPParser::SaveBitPlane(QString fileName, int bitPlane)
{
    return bmpBitPlanes[bitPlane].save(fileName,"BMP");
}
