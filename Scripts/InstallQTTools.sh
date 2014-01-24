#!/usr/bin/env sh

#Install the QT 5 development tools on Ubuntu
sudo add-apt-repository ppa:ubuntu-sdk-team/ppa 
sudo apt-get update

cd 
sudo apt-get install qttools5-dev
sudo apt-get install qt5-default

#Please install in the default home folder
cd 
wget http://download.qt-project.org/official_releases/qt/5.1/5.1.1/qt-linux-opensource-5.1.1-x86_64-offline.run
chmod +x qt-linux-opensource-5.1.1-x86_64-offline.run
./qt-linux-opensource-5.1.1-x86_64-offline.run


#create symbolic link
sudo rm -r /usr/lib/x86_64-linux-gnu/qt5/bin
sudo ln -s $HOME/Qt5.1.1/5.1.1/gcc_64/bin /usr/lib/x86_64-linux-gnu/qt5/