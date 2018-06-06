#ifndef MODEL_CHANGE_H
#define MODEL_CHANGE_H

#include <QtWidgets/QMainWindow>
#include <QStandardItemModel>
#include <osg/Node>
#include <functional>
#include "ui_model_change.h"

class Scene_Viewer;
class model_change : public QMainWindow
{
	Q_OBJECT

public:
	model_change(QWidget *parent = 0);
	~model_change();


	

	void InitlizeWt();
	void signal_slots();
	void ParseNode();
	void ParseNode(osg::ref_ptr<osg::Node> node, QStandardItem* item);

	void Set_Pos(osg::Vec3d eye, osg::Vec3d center, osg::Vec3d up);
	void Set_Frusm(double l, double r, double d, double u, double n, double f);
	void Set_Matrixstr(std::string str, int type);
	void Set_Detailstr(std::string str1, std::string str2, std::string str3, int type);

public slots:

	void Slot_LoadHumanmodel();
	void Slot_Itemclicked(QModelIndex);

private:
	Ui::model_changeClass ui;
	Scene_Viewer* m_pSceneViewer;
	QStandardItemModel* m_pmodel;
	QStandardItem* m_prootitem;
	osg::ref_ptr<osg::Node> m_pselectednode;
};

#endif // MODEL_CHANGE_H
