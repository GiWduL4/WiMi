#!/bin/sh
otool -L LightCrafterGUI.app/Contents/MacOS/LightCrafterGUI
mkdir LightCrafterGUI.app/Contents/Frameworks
cp -R /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtCore.framework LightCrafterGUI.app/Contents/Frameworks/
cp -R /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtGui.framework LightCrafterGUI.app/Contents/Frameworks/
cp -R /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtNetwork.framework LightCrafterGUI.app/Contents/Frameworks/
install_name_tool -id @executable_path/../Frameworks/QtCore.framework/Versions/4/QtCore LightCrafterGUI.app/Contents/Frameworks/QtCore.framework/Versions/4/QtCore
install_name_tool -id @executable_path/../Frameworks/QtGui.framework/Versions/4/QtGui LightCrafterGUI.app/Contents/Frameworks/QtGui.framework/Versions/4/QtGui
install_name_tool -id @executable_path/../Frameworks/QtNetwork.framework/Versions/4/QtNetwork LightCrafterGUI.app/Contents/Frameworks/QtNetwork.framework/Versions/4/QtNetwork
install_name_tool -change /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtCore.framework/Versions/4/QtCore @executable_path/../Frameworks/QtCore.framework/Versions/4/QtCore LightCrafterGUI.app/Contents/MacOS/LightCrafterGUI
install_name_tool -change /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtGui.framework/Versions/4/QtGui @executable_path/../Frameworks/QtGui.framework/Versions/4/QtGui LightCrafterGUI.app/Contents/MacOS/LightCrafterGUI
install_name_tool -change /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtNetwork.framework/Versions/4/QtNetwork @executable_path/../Frameworks/QtNetwork.framework/Versions/4/QtNetwork LightCrafterGUI.app/Contents/MacOS/LightCrafterGUI
install_name_tool -change /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtCore.framework/Versions/4/QtCore @executable_path/../Frameworks/QtCore.framework/Versions/4/QtCore LightCrafterGUI.app/Contents/Frameworks/QtGui.framework/Versions/4/QtGui
install_name_tool -change /Users/pedro/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtCore.framework/Versions/4/QtCore @executable_path/../Frameworks/QtCore.framework/Versions/4/QtCore LightCrafterGUI.app/Contents/Frameworks/QtNetwork.framework/Versions/4/QtNetwork 