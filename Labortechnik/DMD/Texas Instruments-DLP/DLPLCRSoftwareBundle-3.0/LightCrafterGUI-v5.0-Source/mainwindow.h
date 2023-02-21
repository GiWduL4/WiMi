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
  * \file mainwindow.h
  *
  * \brief Declares the MainWindow class which provides the GUI for the LightCrafter Control Software.
  * \n Provides the methods to perform different functionalities supported by LightCrafter.
  */


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileInfo>
#include <QSettings>
#include "PacketHandler.h"
#include "Helper.h"
#include "MBMCReadmeTxtParser.h"

namespace Ui {
class MainWindow;
}

/*!
 * \brief The MainWindow class
 * \n Provides the GUI controls for LightCrafter Control Software
 * \n Manages the sending of commands according to functionality required and interpreting the response.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    enum { MAX_HW_PATTERNS = 32,
           MAX_PATTERNS = 2000,
           SOLUTION_NAME_LEN = 32,
           PTN_WIDTH = 608,
           PTN_HEIGHT = 684
         };
    Ui::MainWindow *ui;
    PacketHandler packetHandler;
    QFileInfo patternFile[MAX_PATTERNS];
    QFileInfo patternFileMBMC[MAX_PATTERNS];
    uchar *camImageData;
    QImage *camImage;
    bool extendedMode;
    QString imagePath;
    QString seqFilePath;
    QString readmeFilePath;
    QString imagelistFilePath;
    QString ptnImagePath;
    QSettings settings;
    struct {
        uchar number;
        uchar invert;
    } HWPtns[MAX_HW_PATTERNS];
    QString extPatLEDSelect_prevText;
    QString extPatBitDepth_prevText;
    QString extPatFrameRate_prevText;
    int     updateExtPatCombos;


    MBMCReadmeTxtParser TxtFile;

    int putTextIntData(QString text, int size)
    {
        return packetHandler.putData(strToNum(text), size);
    }

    QString getTextIntData(int size, bool pad = 0, bool hex = 0)
    {
        return numToStr(packetHandler.getInt(size), pad, hex);
    }
    void fillNumPatterns(void);
    int sendFile(QString fileName);
    void showError(const char *str);
    void showVerMismatchwarning( QString titleName, QString strMsg);
    void checkItems(void);
    void addExtPatFrameRate(void);
    bool checkPlaying(void);
    void enableHWPatterns(void);
    int sendHWPattern(int start, int count);
    void setHWPattern(void);
    void genHWPattern(QImage *img, int ptnNum, int inv);

public slots:
    void receiveStatus(const char *str, PacketHandler::ConnectionStatus status);

private slots:
    void on_extPatBitDepth_currentIndexChanged(QString );
    void on_extPatLEDSelect_currentIndexChanged(int index);
    void on_changeIP_clicked();
    void on_exposureTime_editingFinished();
    void on_playCount_clicked();
    void on_displayThisButton_clicked();
    void on_uploadThisButton_clicked();
    void on_camImageSave_clicked();
    void on_camCaptureBut_clicked();
    void on_pkgBrowse_clicked();
    void on_Install_clicked();
    void on_videoModeGet_clicked();
    void on_videoModeSet_clicked();
    void on_camTrigGet_clicked();
    void on_camTrigSet_clicked();
    void on_solutionDefaultBut_clicked();
#if QT_VERSION <= 0x040804
    void on_tabWidget_currentChanged(QWidget* );
#else
    void on_tabWidget_currentChanged(int);
#endif
    void on_solutionDeleteBut_clicked();
    void on_solutionLoadBut_clicked();
    void on_solutionGetAllBut_clicked();
    void on_solutionSaveBut_clicked();
    void on_bitDepth_currentIndexChanged(int index);
    void on_staticColorSetBut_clicked();
    void on_pushButton_clicked();
    void on_staticImageBrowseBut_clicked();
    void on_LEDCurSet_clicked();
    void on_LEDCurGet_clicked();
    void on_I2Cwrite_clicked();
    void on_I2Cread_clicked();
    void on_nextSeqBut_clicked();
    void on_stopSeqBut_clicked();
    void on_startSeqBut_clicked();
    void on_FPGASetButton_clicked();
    void on_FPGAGetButton_clicked();
    void on_versionGetBut_clicked();
    void on_dmSetBut_clicked();
    void on_dmGetBut_clicked();
    void on_testPtnSetBut_clicked();
    void on_testPtnGetBut_clicked();
    void on_imageSetSetBug_clicked();
    void on_imageSetGetBut_clicked();
    void on_dispModeSetBut_clicked();
    void on_dispModeGetBut_clicked();
    void on_uploadButton_clicked();
    void on_browseButton_clicked();
    void on_dppSetButton_clicked();
    void on_patternSetSetting_clicked();
    void on_patternGetSetting_clicked();
    void on_dppGetButton_clicked();
    void on_connectButton_clicked();
    void on_trigType_currentIndexChanged();
    void on_numPatterns_currentIndexChanged();
    void setTab(int modeNum);
    void on_PatternType_currentIndexChanged();
    void on_patternNum_currentIndexChanged(int index);
    void on_HWPtnSelect_currentIndexChanged();
    void on_setMBMCSeq_clicked();
    void on_getMBMCSeq_clicked();
    void on_browseMBMCSeq_clicked();
    void on_uploadMBMCSeq_clicked();
    void on_enableCustSeq_clicked(bool checked);
    void on_getMBMCSeqHelp_clicked();
    void on_getVersionInfoButton_clicked();
    void on_camPreview_clicked();
    void on_dlpc300Browse_clicked();
    void on_comboBox_SplashIndex_currentIndexChanged(int index);
    void on_pushButton_ChangeSplashImage_clicked();
    void on_radioVideoMode_clicked();
    void on_radioExtPatternMode_clicked();
    void on_extPatFrameRate_currentIndexChanged(const QString &arg1);
    void on_LEDCurRed_textChanged(const QString &arg1);
    void on_LEDCurGreen_textChanged(const QString &arg1);
    void on_LEDCurBlue_textChanged(const QString &arg1);
    void on_btnMBMCImport_clicked();
    void on_browseMBMCImageList_clicked();
    void on_btn_SendMBMCSettings_clicked();
    void on_radioVideoMode_toggled(bool checked);
    void on_modeList_currentIndexChanged(int index);
    void on_comboBox_MBMCPatterns_currentIndexChanged(int index);
};

#endif // MAINWINDOW_H
