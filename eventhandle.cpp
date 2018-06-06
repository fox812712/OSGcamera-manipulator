#pragma once
#pragma execution_character_set("utf-8")


#include <osgGA/GUIEventAdapter>
#include "sceneViewer.h"
#include "eventhandle.h"

ObserEventhandle::ObserEventhandle(Scene_Viewer* viewer){
	scene_viewer = viewer;
	this->pviewer = scene_viewer->getViewer();
	pviewer->addEventHandler(this);
}

ObserEventhandle::~ObserEventhandle(){}

bool ObserEventhandle::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& es){
	osgGA::GUIEventAdapter::EventType type = ea.getEventType();
	switch (type)
	{
	case osgGA::GUIEventAdapter::EventType::FRAME:
	{
		osg::Vec3d home, center, up;
		this->pviewer->getCamera()->getViewMatrixAsLookAt(home, center, up);
		double left, right, updir, down, neardir, fardir;
		this->pviewer->getCamera()->getProjectionMatrixAsFrustum(left, right, down, updir, neardir, fardir);
		scene_viewer->Set_Position(home, center, up);
		scene_viewer->Set_MatrixasFustrm(left, right, down, updir, neardir, fardir);

		osg::Matrixd mat=this->pviewer->getCamera()->getProjectionMatrix();
		QString str = QString::number(mat(0, 0)).append(" ").append(QString::number(mat(0, 1))).
			append(" ").append(QString::number(mat(0, 2))).append(" ").append(QString::number(mat(0, 3))).append("\n");
		str += QString::number(mat(1, 0)).append(" ").append(QString::number(mat(1, 1))).
			append(" ").append(QString::number(mat(1, 2))).append(" ").append(QString::number(mat(1, 3))).append("\n");
		str += QString::number(mat(2, 0)).append(" ").append(QString::number(mat(2, 1))).
			append(" ").append(QString::number(mat(2, 2))).append(" ").append(QString::number(mat(2, 3))).append("\n");
		str += QString::number(mat(3, 0)).append(" ").append(QString::number(mat(3, 1))).
			append(" ").append(QString::number(mat(3, 2))).append(" ").append(QString::number(mat(3, 3)));

		scene_viewer->Set_Matrixstr_Type(str.toStdString(), 1);



		 mat = this->pviewer->getCamera()->getViewMatrix();
		QString str1 = QString::number(mat(0, 0)).append(" ").append(QString::number(mat(0, 1))).
			append(" ").append(QString::number(mat(0, 2))).append(" ").append(QString::number(mat(0, 3))).append("\n");
		str1 += QString::number(mat(1, 0)).append(" ").append(QString::number(mat(1, 1))).
			append(" ").append(QString::number(mat(1, 2))).append(" ").append(QString::number(mat(1, 3))).append("\n");
		str1 += QString::number(mat(2, 0)).append(" ").append(QString::number(mat(2, 1))).
			append(" ").append(QString::number(mat(2, 2))).append(" ").append(QString::number(mat(2, 3))).append("\n");
		str1 += QString::number(mat(3, 0)).append(" ").append(QString::number(mat(3, 1))).
			append(" ").append(QString::number(mat(3, 2))).append(" ").append(QString::number(mat(3, 3)));

		scene_viewer->Set_Matrixstr_Type(str1.toStdString(), 2);


		mat = osg::Matrix::inverse(mat);
		osg::Vec3d pos = mat.getTrans();
		osg::Quat quat = mat.getRotate();
		osg::Vec3d scale = mat.getScale();

		QString pos_str = QString::number(pos.x()).append(" ").append(QString::number(pos.y())).append(" ").append(QString::number(pos.z()));
		QString rotate_str = QString::number(quat[0]).append(" ").append(QString::number(quat[1])).append(" ").append(
			QString::number(quat[2])).append(" ").append(QString::number(quat[3]));
		QString scale_str = QString::number(scale[0]).append(" ").append(QString::number(scale[1])).append(" ").append(QString::number(scale[2]));


		scene_viewer->Set_DetailInfo_Type(pos_str.toStdString(), rotate_str.toStdString(), scale_str.toStdString(), 2);


		mat = this->pviewer->getCamera()->getViewport()->computeWindowMatrix();
		QString str2 = QString::number(mat(0, 0)).append(" ").append(QString::number(mat(0, 1))).
			append(" ").append(QString::number(mat(0, 2))).append(" ").append(QString::number(mat(0, 3))).append("\n");
		str2 += QString::number(mat(1, 0)).append(" ").append(QString::number(mat(1, 1))).
			append(" ").append(QString::number(mat(1, 2))).append(" ").append(QString::number(mat(1, 3))).append("\n");
		str2 += QString::number(mat(2, 0)).append(" ").append(QString::number(mat(2, 1))).
			append(" ").append(QString::number(mat(2, 2))).append(" ").append(QString::number(mat(2, 3))).append("\n");
		str2 += QString::number(mat(3, 0)).append(" ").append(QString::number(mat(3, 1))).
			append(" ").append(QString::number(mat(3, 2))).append(" ").append(QString::number(mat(3, 3)));

		scene_viewer->Set_Matrixstr_Type(str2.toStdString(), 3);
	}
		break;
	default:
		break;
	}
	return false;
}