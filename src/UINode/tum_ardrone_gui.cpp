 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "tum_ardrone_gui.h"
#include "RosThread.h"
#include "PingThread.h"
#include "time.h"
#include "../HelperFunctions.h"

#include "ros/ros.h"
#include "ros/package.h"

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

int getdirtxt (std::string dir, std::vector<std::string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    return errno;
  }

  while ((dirp = readdir(dp)) != NULL) {
    std::string f = dirp->d_name;
    
    if(f.size() > 4 && f.substr(f.size()-4) == ".txt")
      files.push_back(std::string(dirp->d_name));
  }
  
  closedir(dp);
  return 0;
}


tum_ardrone_gui::tum_ardrone_gui(QWidget *parent)
  : QWidget(parent)
{
  ui.setupUi(this);
  rosThread = NULL;
  sensGaz = sensYaw = sensRP = 1;
  currentControlSource = CONTROL_NONE;

  QObject::connect( this, SIGNAL( setCountsSignal(unsigned int,unsigned int,unsigned int) ),
                      this, SLOT( setCountsSlot(unsigned int,unsigned int,unsigned int) ) );

  QObject::connect( this, SIGNAL( setPingsSignal(int, int) ),
                      this, SLOT( setPingsSlot(int, int) ) );

  QObject::connect( this, SIGNAL( setControlSourceSignal(int) ),
                      this, SLOT( setControlSourceSlot(int) ) );

  QObject::connect( this, SIGNAL( addLogLineSignal(QString) ),
                      this, SLOT( addLogLineSlot(QString) ) );

  QObject::connect( this, SIGNAL( setAutopilotInfoSignal(QString) ),
                      this, SLOT( setAutopilotInfoSlot(QString) ) );

  QObject::connect( this, SIGNAL( setStateestimationInfoSignal(QString) ),
                      this, SLOT( setStateestimationInfoSlot(QString) ) );

  QObject::connect( this, SIGNAL( setMotorSpeedsSignal(QString) ),
                      this, SLOT( setMotorSpeedsSlot(QString) ) );

  QObject::connect( this, SIGNAL( closeWindowSignal() ),
                      this, SLOT( closeWindowSlot() ) );


  std::vector<std::string> files = std::vector<std::string>();
  getdirtxt(ros::package::getPath("tum_ardrone") + std::string("/flightPlans/"),files);

  ui.comboBoxLoadFile->addItem(QString(""), QVariant());
  for(unsigned int i=0;i<files.size();i++)
    ui.comboBoxLoadFile->addItem(QString(files[i].c_str()), QVariant());
}


tum_ardrone_gui::~tum_ardrone_gui()
{
}


// clicked functions
void tum_ardrone_gui::LandClicked()
{
  manualControl();
  rosThread->sendLand();
}

void tum_ardrone_gui::TakeoffClicked()
{
  manualControl();
  rosThread->sendTakeoff();
}

void tum_ardrone_gui::ToggleCamClicked()
{
  //manualControl();
  rosThread->sendToggleCam();
}

void tum_ardrone_gui::ToggleStateClicked()
{
  manualControl();
  rosThread->sendToggleState();
}

void tum_ardrone_gui::FlatTrimClicked()
{
  manualControl();
  rosThread->sendFlatTrim();
}

void tum_ardrone_gui::ClearClicked()
{
  rosThread->publishCommand("c clearCommands");
}

void tum_ardrone_gui::SendClicked()
{
  QStringList l = ui.plainTextEditSendCommand->toPlainText().split('\n');
  for(int i=0;i<l.length();i++)
  {
    std::string s = l[i].trimmed().toStdString();

    if(s.size() > 0)
      rosThread->publishCommand(std::string("c ")+s);
  }
  
  if (currentControlSource != CONTROL_AUTO)
    setControlSource(CONTROL_AUTO);
}

void tum_ardrone_gui::ClearSendClicked()
{
  ClearClicked();
  SendClicked();
}


void tum_ardrone_gui::LoadFileChanged(QString val)
{
  if(val == "")
    ui.plainTextEditSendCommand->setPlainText("");
  else
  {
    std::string path = ros::package::getPath("tum_ardrone") + std::string("/flightPlans/") + val.toStdString();
    addLogLine("Load File "+ path);

    std::ifstream t;
    t.open(path.c_str());
    std::string buffer = "";
    std::string line;

    while(!t.eof())
    {
      std::getline(t, line);
      buffer = buffer + line + "\n";
    }
    
    t.close();

    ui.plainTextEditSendCommand->setPlainText(buffer.c_str());
  }
}


void tum_ardrone_gui::ToggledPingDrone(int val)
{
  pingThread->measure = (val != 0);
}


// change control source functions
void tum_ardrone_gui::ControlSourceChanged()
{
  ControlSource s = CONTROL_NONE;

  if(ui.radioButtonControlAuto->isChecked())
    s = CONTROL_AUTO;

  if(s != CONTROL_AUTO)
    rosThread->publishCommand("c stop");
  else
    rosThread->publishCommand("c start");

  currentControlSource = s;
}


void tum_ardrone_gui::setControlSourceSlot(int cont)
{
  currentControlSource = (ControlSource)cont;

  if(cont == CONTROL_NONE)
    ui.radioButtonControlNone->setChecked(true);

  if(cont == CONTROL_AUTO)
    ui.radioButtonControlAuto->setChecked(true);

  ControlSourceChanged();
}


void tum_ardrone_gui::setCountsSlot(unsigned int nav,unsigned int control,unsigned int pose)
{
  char buf[100];
  snprintf(buf,100, "Drone Control: %d Hz", control);
  ui.labelControl->setText(buf);

  snprintf(buf,100, "Drone Navdata: %d Hz", nav);
  ui.labelNavdata->setText(buf);

  snprintf(buf,100, "Pose Estimates: %d Hz", pose);
  ui.labelPoseEst->setText(buf);
}


void tum_ardrone_gui::setPingsSlot(int p500, int p20000)
{
  char buf[100];
  snprintf(buf,100, "Pings (RTT): %d (500B), %d (20kB)", p500, p20000);
  ui.labelDronePings->setText(buf);
}

void tum_ardrone_gui::addLogLineSlot(QString s)
{
  ui.plainTextEditMessages->appendPlainText(s);
}

void tum_ardrone_gui::setAutopilotInfoSlot(QString s)
{
  ui.plainTextEditAutopilotStatus->setPlainText(s);
}

void tum_ardrone_gui::setStateestimationInfoSlot(QString s)
{
  ui.plainTextEditStateestimationStatus->setPlainText(s);
}

void tum_ardrone_gui::setMotorSpeedsSlot(QString s)
{
  ui.labelDroneMotors->setText(s);
}

void tum_ardrone_gui::closeWindowSlot()
{
  closeWindow();
}


// these may be called from external thread,
// so they just "forward" the request.
void tum_ardrone_gui::setCounts(unsigned int nav,unsigned int control,unsigned int pose)
{
  emit setCountsSignal(nav, control, pose);
}

void tum_ardrone_gui::setControlSource(ControlSource cont)
{
  emit setControlSourceSignal((int)cont);
}

void tum_ardrone_gui::addLogLine(std::string s)
{
  emit addLogLineSignal(QString(s.c_str()));
}

void tum_ardrone_gui::setAutopilotInfo(std::string s)
{
  emit setAutopilotInfoSignal(QString(s.c_str()));
}

void tum_ardrone_gui::setMotorSpeeds(std::string s)
{
  emit setMotorSpeedsSignal(QString(s.c_str()));
}

void tum_ardrone_gui::setStateestimationInfo(std::string s)
{
  emit setStateestimationInfoSignal(QString(s.c_str()));
}

void tum_ardrone_gui::setPings(int p500, int p20000)
{
  emit setPingsSignal(p500, p20000);
}

void tum_ardrone_gui::closeWindow()
{
  emit closeWindowSignal();
}

void tum_ardrone_gui::manualControl(void)
{
  if (currentControlSource == CONTROL_AUTO)
  {
    setControlSource(CONTROL_NONE);
    rosThread->gui->addLogLine("Warning: Manual override! Autopilot disabled!");
  }
}

