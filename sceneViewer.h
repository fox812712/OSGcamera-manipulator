#pragma once
#pragma execution_character_set("utf-8")

#ifndef SCENEVIEWER_H
#define SCENEVIEWER_H

#include <QObject>
#include <functional>
#include <osgViewer/Viewer>
#include <QTimer>

#include "GraphicsWindowQt.h"

class Scene_Viewer :public QObject{
	Q_OBJECT
public:
	Scene_Viewer(osgQt::GLWidget* renderwt);
	~Scene_Viewer();

	typedef  std::function<void(osg::Vec3d, osg::Vec3d, osg::Vec3d)> f;
	typedef std::function<void(double, double, double, double, double, double)> fmatrix;
	typedef std::function<void(std::string, int)> matrix_str;
	typedef std::function<void(std::string, std::string, std::string, int)>detail_str;

	void Set_F(std::function<void(osg::Vec3d, osg::Vec3d, osg::Vec3d)>);
	void Set_Frusm(fmatrix m);
	void Set_Matrixstr(matrix_str m);
	void Set_Detailstr(detail_str m);

	void Load_HumanModel(const QString& path);
	osg::ref_ptr<osg::Node> Get_HumanNode();
	osg::ref_ptr<osg::Node> Find_PartofHuman(QString part_name);
	void Set_Position(osg::Vec3d eye, osg::Vec3d center, osg::Vec3d up);
	void Set_MatrixasFustrm(double left, double right, double down, double up, double near, double far);
	void Set_Matrixstr_Type(std::string, int);
	void Set_DetailInfo_Type(std::string, std::string, std::string, int);
	void Test();
	osgViewer::Viewer* getViewer();
protected:
	void Initlize();
	void Init_Grid();
	void Update_Frame();
	osg::ref_ptr<osg::Node> Find_PartofHuman(osg::ref_ptr<osg::Node> node, QString part_name);

	
public slots:
	void Slot_Frame();
private:


	f  m_f;
	fmatrix mm_matrixfrusm;
	matrix_str  matrixstr;
	detail_str  detailstr;

	osgViewer::Viewer* m_pViewerPtr;
	osg::ref_ptr<osg::Group> m_pGp;
	QTimer m_pFrame;
	osgQt::GLWidget* m_pGLwidgetPtr;
	osg::ref_ptr<osg::Node>  m_pHumanNode;
	osg::ref_ptr<osg::Node>  m_pSlectPartNode;
};
#endif