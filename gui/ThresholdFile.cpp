#include "ThresholdFile.h"

ThresholdFile::ThresholdFile()
{
}

ThresholdFile::ThresholdFile(Webcam * cam, QString path)
{
	_cam = cam;
	_path = path;
	QFileInfo fileInfo(_path);
	if (fileInfo.exists()) {
		loadData();
	}
}

void ThresholdFile::load()
{
	QString filename = QFileDialog::getOpenFileName(
		0, 
		"Load Threshold File", 
		_path);
	if (filename != "") {
		_path = filename;
		loadData();
	}
}

void ThresholdFile::save()
{
	QString filename = QFileDialog::getSaveFileName(
				0,
				"Save Thresold File",
				_path,
				"XML (*.xml)");
	if (filename != "") {
		_path = filename;
		saveData();
	}
}

void ThresholdFile::loadData()
{
	_xml = XMLReader(_path.toLocal8Bit().constData());
	_cam->_arenaHmin = atoi(_xml["arenaHmin"].c_str());
	_cam->_arenaHmax = atoi(_xml["arenaHmax"].c_str());
	_cam->_arenaSmin = atoi(_xml["arenaSmin"].c_str());
	_cam->_arenaSmax = atoi(_xml["arenaSmax"].c_str());
	_cam->_arenaVmin = atoi(_xml["arenaVmin"].c_str());
	_cam->_arenaVmax = atoi(_xml["arenaVmax"].c_str());
	_cam->_arenaAmin = atoi(_xml["arenaAmin"].c_str());
	_cam->_arenaAmax = atoi(_xml["arenaAmax"].c_str());
	_cam->_ballHmin = atoi(_xml["ballHmin"].c_str());
	_cam->_ballHmax = atoi(_xml["ballHmax"].c_str());
	_cam->_ballSmin = atoi(_xml["ballSmin"].c_str());
	_cam->_ballSmax = atoi(_xml["ballSmax"].c_str());
	_cam->_ballVmin = atoi(_xml["ballVmin"].c_str());
	_cam->_ballVmax = atoi(_xml["ballVmax"].c_str());
	_cam->_ballAmin = atoi(_xml["ballAmin"].c_str());
	_cam->_ballAmax = atoi(_xml["ballAmax"].c_str());
	_cam->_obstacles1Hmin = atoi(_xml["obstacles1Hmin"].c_str());
	_cam->_obstacles1Hmax = atoi(_xml["obstacles1Hmax"].c_str());
	_cam->_obstacles1Smin = atoi(_xml["obstacles1Smin"].c_str());
	_cam->_obstacles1Smax = atoi(_xml["obstacles1Smax"].c_str());
	_cam->_obstacles1Vmin = atoi(_xml["obstacles1Vmin"].c_str());
	_cam->_obstacles1Vmax = atoi(_xml["obstacles1Vmax"].c_str());
	_cam->_obstacles1Amin = atoi(_xml["obstacles1Amin"].c_str());
	_cam->_obstacles1Amax = atoi(_xml["obstacles1Amax"].c_str());
	_cam->_obstacles2Hmin = atoi(_xml["obstacles2Hmin"].c_str());
	_cam->_obstacles2Hmax = atoi(_xml["obstacles2Hmax"].c_str());
	_cam->_obstacles2Smin = atoi(_xml["obstacles2Smin"].c_str());
	_cam->_obstacles2Smax = atoi(_xml["obstacles2Smax"].c_str());
	_cam->_obstacles2Vmin = atoi(_xml["obstacles2Vmin"].c_str());
	_cam->_obstacles2Vmax = atoi(_xml["obstacles2Vmax"].c_str());
	_cam->_obstacles2Amin = atoi(_xml["obstacles2Amin"].c_str());
	_cam->_obstacles2Amax = atoi(_xml["obstacles2Amax"].c_str());
	_cam->_robot1Hmin = atoi(_xml["robot1Hmin"].c_str());
	_cam->_robot1Hmax = atoi(_xml["robot1Hmax"].c_str());
	_cam->_robot1Smin = atoi(_xml["robot1Smin"].c_str());
	_cam->_robot1Smax = atoi(_xml["robot1Smax"].c_str());
	_cam->_robot1Vmin = atoi(_xml["robot1Vmin"].c_str());
	_cam->_robot1Vmax = atoi(_xml["robot1Vmax"].c_str());
	_cam->_robot1Amin = atoi(_xml["robot1Amin"].c_str());
	_cam->_robot1Amax = atoi(_xml["robot1Amax"].c_str());
	_cam->_robot2Hmin = atoi(_xml["robot2Hmin"].c_str());
	_cam->_robot2Hmax = atoi(_xml["robot2Hmax"].c_str());
	_cam->_robot2Smin = atoi(_xml["robot2Smin"].c_str());
	_cam->_robot2Smax = atoi(_xml["robot2Smax"].c_str());
	_cam->_robot2Vmin = atoi(_xml["robot2Vmin"].c_str());
	_cam->_robot2Vmax = atoi(_xml["robot2Vmax"].c_str());
	_cam->_robot2Amin = atoi(_xml["robot2Amin"].c_str());
	_cam->_robot2Amax = atoi(_xml["robot2Amax"].c_str());
	_cam->_opp1Hmin = atoi(_xml["opp1Hmin"].c_str());
	_cam->_opp1Hmax = atoi(_xml["opp1Hmax"].c_str());
	_cam->_opp1Smin = atoi(_xml["opp1Smin"].c_str());
	_cam->_opp1Smax = atoi(_xml["opp1Smax"].c_str());
	_cam->_opp1Vmin = atoi(_xml["opp1Vmin"].c_str());
	_cam->_opp1Vmax = atoi(_xml["opp1Vmax"].c_str());
	_cam->_opp1Amin = atoi(_xml["opp1Amin"].c_str());
	_cam->_opp1Amax = atoi(_xml["opp1Amax"].c_str());
	_cam->_opp2Hmin = atoi(_xml["opp2Hmin"].c_str());
	_cam->_opp2Hmax = atoi(_xml["opp2Hmax"].c_str());
	_cam->_opp2Smin = atoi(_xml["opp2Smin"].c_str());
	_cam->_opp2Smax = atoi(_xml["opp2Smax"].c_str());
	_cam->_opp2Vmin = atoi(_xml["opp2Vmin"].c_str());
	_cam->_opp2Vmax = atoi(_xml["opp2Vmax"].c_str());
	_cam->_opp2Amin = atoi(_xml["opp2Amin"].c_str());
	_cam->_opp2Amax = atoi(_xml["opp2Amax"].c_str());
}

void ThresholdFile::saveData()
{
	_xml = XMLReader(_path.toLocal8Bit().constData());
	_xml.setValue("arenaHmin",itoa(_cam->_arenaAmin));
	_xml.setValue("arenaHmin",itoa(_cam->_arenaHmin));
	_xml.setValue("arenaHmax",itoa(_cam->_arenaHmax));
	_xml.setValue("arenaSmin",itoa(_cam->_arenaSmin));
	_xml.setValue("arenaSmax",itoa(_cam->_arenaSmax));
	_xml.setValue("arenaVmin",itoa(_cam->_arenaVmin));
	_xml.setValue("arenaVmax",itoa(_cam->_arenaVmax));
	_xml.setValue("arenaAmin",itoa(_cam->_arenaAmin));
	_xml.setValue("arenaAmax",itoa(_cam->_arenaAmax));
	_xml.setValue("ballHmin",itoa(_cam->_ballHmin));
	_xml.setValue("ballHmax",itoa(_cam->_ballHmax));
	_xml.setValue("ballSmin",itoa(_cam->_ballSmin));
	_xml.setValue("ballSmax",itoa(_cam->_ballSmax));
	_xml.setValue("ballVmin",itoa(_cam->_ballVmin));
	_xml.setValue("ballVmax",itoa(_cam->_ballVmax));
	_xml.setValue("ballAmin",itoa(_cam->_ballAmin));
	_xml.setValue("ballAmax",itoa(_cam->_ballAmax));
	_xml.setValue("obstacles1Hmin",itoa(_cam->_obstacles1Hmin));
	_xml.setValue("obstacles1Hmax",itoa(_cam->_obstacles1Hmax));
	_xml.setValue("obstacles1Smin",itoa(_cam->_obstacles1Smin));
	_xml.setValue("obstacles1Smax",itoa(_cam->_obstacles1Smax));
	_xml.setValue("obstacles1Vmin",itoa(_cam->_obstacles1Vmin));
	_xml.setValue("obstacles1Vmax",itoa(_cam->_obstacles1Vmax));
	_xml.setValue("obstacles1Amin",itoa(_cam->_obstacles1Amin));
	_xml.setValue("obstacles1Amax",itoa(_cam->_obstacles1Amax));
	_xml.setValue("obstacles2Hmin",itoa(_cam->_obstacles2Hmin));
	_xml.setValue("obstacles2Hmax",itoa(_cam->_obstacles2Hmax));
	_xml.setValue("obstacles2Smin",itoa(_cam->_obstacles2Smin));
	_xml.setValue("obstacles2Smax",itoa(_cam->_obstacles2Smax));
	_xml.setValue("obstacles2Vmin",itoa(_cam->_obstacles2Vmin));
	_xml.setValue("obstacles2Vmax",itoa(_cam->_obstacles2Vmax));
	_xml.setValue("obstacles2Amin",itoa(_cam->_obstacles2Amin));
	_xml.setValue("obstacles2Amax",itoa(_cam->_obstacles2Amax));
	_xml.setValue("robot1Hmin",itoa(_cam->_robot1Hmin));
	_xml.setValue("robot1Hmax",itoa(_cam->_robot1Hmax));
	_xml.setValue("robot1Smin",itoa(_cam->_robot1Smin));
	_xml.setValue("robot1Smax",itoa(_cam->_robot1Smax));
	_xml.setValue("robot1Vmin",itoa(_cam->_robot1Vmin));
	_xml.setValue("robot1Vmax",itoa(_cam->_robot1Vmax));
	_xml.setValue("robot1Amin",itoa(_cam->_robot1Amin));
	_xml.setValue("robot1Amax",itoa(_cam->_robot1Amax));
	_xml.setValue("robot2Hmin",itoa(_cam->_robot2Hmin));
	_xml.setValue("robot2Hmax",itoa(_cam->_robot2Hmax));
	_xml.setValue("robot2Smin",itoa(_cam->_robot2Smin));
	_xml.setValue("robot2Smax",itoa(_cam->_robot2Smax));
	_xml.setValue("robot2Vmin",itoa(_cam->_robot2Vmin));
	_xml.setValue("robot2Vmax",itoa(_cam->_robot2Vmax));
	_xml.setValue("robot2Amin",itoa(_cam->_robot2Amin));
	_xml.setValue("robot2Amax",itoa(_cam->_robot2Amax));
	_xml.setValue("opp1Hmin",itoa(_cam->_opp1Hmin));
	_xml.setValue("opp1Hmax",itoa(_cam->_opp1Hmax));
	_xml.setValue("opp1Smin",itoa(_cam->_opp1Smin));
	_xml.setValue("opp1Smax",itoa(_cam->_opp1Smax));
	_xml.setValue("opp1Vmin",itoa(_cam->_opp1Vmin));
	_xml.setValue("opp1Vmax",itoa(_cam->_opp1Vmax));
	_xml.setValue("opp1Amin",itoa(_cam->_opp1Amin));
	_xml.setValue("opp1Amax",itoa(_cam->_opp1Amax));
	_xml.setValue("opp2Hmin",itoa(_cam->_opp2Hmin));
	_xml.setValue("opp2Hmax",itoa(_cam->_opp2Hmax));
	_xml.setValue("opp2Smin",itoa(_cam->_opp2Smin));
	_xml.setValue("opp2Smax",itoa(_cam->_opp2Smax));
	_xml.setValue("opp2Vmin",itoa(_cam->_opp2Vmin));
	_xml.setValue("opp2Vmax",itoa(_cam->_opp2Vmax));
	_xml.setValue("opp2Amin",itoa(_cam->_opp2Amin));
	_xml.setValue("opp2Amax",itoa(_cam->_opp2Amax));
	_xml.save();
}

string ThresholdFile::itoa (int value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}