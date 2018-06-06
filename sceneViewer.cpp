#pragma once
#pragma execution_character_set("utf-8")

#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/LineWidth>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include "eventhandle.h"
#include "sceneViewer.h"

#include "Handler_WSNavigateManipulator.h"


#define  GRID_WIDTH 20 
static double decime_meter = 0.1f; // 100mm
static double center_meter = 0.02f;// 20mm

Scene_Viewer::Scene_Viewer(osgQt::GLWidget* renderwt) :m_pGLwidgetPtr(renderwt){
	Initlize();

	connect(&m_pFrame, &QTimer::timeout, this, &Scene_Viewer::Slot_Frame);

	m_pFrame.start(10);
}

Scene_Viewer::~Scene_Viewer(){

}

void Scene_Viewer::Set_F(std::function<void(osg::Vec3d, osg::Vec3d, osg::Vec3d)> g){
	m_f = g;
}

void Scene_Viewer::Set_Frusm(fmatrix m){
	mm_matrixfrusm = m;
}

void Scene_Viewer::Set_Matrixstr(matrix_str m){
	matrixstr = m;
}

void Scene_Viewer::Set_Detailstr(detail_str m){
	detailstr = m;
}

void Scene_Viewer::Initlize(){


	QGLFormat format = m_pGLwidgetPtr->format();
	format.setSamples(16);
	format.setSampleBuffers(true);

	m_pGLwidgetPtr->setFormat(format);
	m_pGLwidgetPtr->setAutoFillBackground(true);

	//初始化图形环境
	osg::DisplaySettings::instance()->setNumMultiSamples(16);
	osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);
	osgQt::initQtWindowingSystem();

	m_pViewerPtr = new osgViewer::Viewer;

	osgQt::GraphicsWindowQt* gc = new osgQt::GraphicsWindowQt(m_pGLwidgetPtr);

	m_pViewerPtr->getCamera()->setClearColor(osg::Vec4(0.3, 0.5, 0.6, 1));
	m_pViewerPtr->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	m_pViewerPtr->getCamera()->setClearStencil(0);
	m_pViewerPtr->getCamera()->setViewMatrixAsLookAt(osg::Vec3(0, 1, 0), osg::Vec3(0, 0, 0), osg::Vec3(0, 0, 1));
	m_pViewerPtr->getCamera()->setProjectionMatrixAsPerspective(25.0, 1.0 * m_pGLwidgetPtr->width() / m_pGLwidgetPtr->height(), 1, 100);
	m_pViewerPtr->getCamera()->setViewport(new osg::Viewport(0, 0, m_pGLwidgetPtr->width(), m_pGLwidgetPtr->height()));
	m_pViewerPtr->getCamera()->setGraphicsContext(gc);
	m_pViewerPtr->getCamera()->setCullingMode(osg::CullingSet::NO_CULLING);
	m_pViewerPtr->getCamera()->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	m_pViewerPtr->getCamera()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	m_pViewerPtr->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	m_pViewerPtr->setKeyEventSetsDone(0);
	m_pViewerPtr->realize();

	//初始化操作器
	WSNavigateManipulator* navi = new WSNavigateManipulator(m_pGLwidgetPtr);
	navi->setAllowThrow(false);
	navi->setHomePosition(osg::Vec3(0, -4.81, 1), osg::Vec3(0, 0, 1), osg::Vec3(0, 0, 1));
	m_pViewerPtr->setCameraManipulator(navi);


	
	//初始化节点
	m_pGp = new osg::Group;
	m_pViewerPtr->setSceneData(m_pGp);

	m_pViewerPtr->addEventHandler(new osgGA::StateSetManipulator(m_pGp->getOrCreateStateSet()));
	m_pViewerPtr->addEventHandler(new osgViewer::StatsHandler());

	ObserEventhandle* obs_handle = new ObserEventhandle(this);
	//网格
//	Init_Grid();
}

void Scene_Viewer::Init_Grid(){

	osg::MatrixTransform* mat = new osg::MatrixTransform;
	//m_pGp->addChild(mat);
	mat->getOrCreateStateSet()->setRenderBinDetails(10000, "RenderBin");

	osg::Geode* m_pRulergeode = new osg::Geode;
	{
		osg::Geometry* geom = new osg::Geometry;
		m_pRulergeode->addDrawable(geom);
		osg::Vec3Array* v = new osg::Vec3Array;
		osg::Vec4Array* c = new osg::Vec4Array;
		int row = GRID_WIDTH / center_meter;
		for (int i = -row; i <= row; ++i)
		{
			if (i == 0 || (i % 5) == 0) continue;
			v->push_back(osg::Vec3d(-GRID_WIDTH, 0, i*center_meter));
			v->push_back(osg::Vec3d(GRID_WIDTH, 0, i*center_meter));
			v->push_back(osg::Vec3d(i*center_meter, 0, -GRID_WIDTH));
			v->push_back(osg::Vec3d(i *center_meter, 0, GRID_WIDTH));
		}
		c->push_back(osg::Vec4(0.7, 0.7, 0.7, 1));
		geom->setVertexArray(v);
		geom->setColorArray(c);
		geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v->size()));
	}
	{
		osg::Geometry* geom1 = new osg::Geometry;
		m_pRulergeode->addDrawable(geom1);
		osg::Vec3Array* v1 = new osg::Vec3Array;
		osg::Vec4Array* c1 = new osg::Vec4Array;
		int row = GRID_WIDTH / decime_meter;
		for (int i = -row; i <= row; ++i)
		{
			if (i == 0)continue;
			v1->push_back(osg::Vec3d(-GRID_WIDTH, 0, i*decime_meter));
			v1->push_back(osg::Vec3d(GRID_WIDTH, 0, i*decime_meter));
			v1->push_back(osg::Vec3d(i*decime_meter, 0, -GRID_WIDTH));
			v1->push_back(osg::Vec3d(i *decime_meter, 0, GRID_WIDTH));
		}
		c1->push_back(osg::Vec4(0.6, 0.6, 0.6, 1));
		geom1->setVertexArray(v1);
		geom1->setColorArray(c1);
		geom1->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		geom1->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v1->size()));
		geom1->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(1.5f));
	}
	osg::Geode* m_pAxisGeode = new osg::Geode;
	{
		osg::Geometry* geom3 = new osg::Geometry;
		m_pAxisGeode->addDrawable(geom3);
		int row = GRID_WIDTH;
		osg::Vec3dArray* v3 = new osg::Vec3dArray;
		osg::Vec4Array* c4 = new osg::Vec4Array;
		v3->push_back(osg::Vec3d(-row, 0, 0));
		v3->push_back(osg::Vec3d(row, 0, 0));
		v3->push_back(osg::Vec3d(0, 0, -row));
		v3->push_back(osg::Vec3d(0, 0, row));
		c4->push_back(osg::Vec4d(0.2, 0.2, 0.2, 1));
		geom3->setVertexArray(v3);
		geom3->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v3->size()));
		geom3->setColorArray(c4);
		geom3->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		m_pAxisGeode->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(2));
	}

	mat->addChild(m_pAxisGeode);
	mat->addChild(m_pRulergeode);
}


void Scene_Viewer::Load_HumanModel(const QString& path){
	m_pHumanNode = osgDB::readNodeFile(path.toStdString());
	osg::MatrixTransform* mat = new osg::MatrixTransform;
	osg::Matrix m;
	m_pGp->addChild(m_pHumanNode);
}

osg::ref_ptr<osg::Node> Scene_Viewer::Get_HumanNode(){
	return m_pGp->getChild(0);
}

osg::ref_ptr<osg::Node> Scene_Viewer::Find_PartofHuman(QString part_name){
	return Find_PartofHuman(m_pHumanNode, part_name);
}

void Scene_Viewer::Set_Position(osg::Vec3d eye, osg::Vec3d center, osg::Vec3d up){
	m_f(eye, center, up);
}

void Scene_Viewer::Set_MatrixasFustrm(double left, double right, double down, double up, double near_val, double far_val){
	mm_matrixfrusm(left, right, down, up, near_val, far_val);
}

void Scene_Viewer::Set_Matrixstr_Type(std::string note, int type){
	matrixstr(note, type);
}

void Scene_Viewer::Set_DetailInfo_Type(std::string str1, std::string str2, std::string str3, int type){
	detailstr(str1, str2, str3, type);
}

void Scene_Viewer::Test(){
	if (!m_pHumanNode->asGroup())return;
	osg::Node* node = m_pHumanNode->asGroup()->getChild(1);
	if (!node->asTransform())return;
	osg::MatrixTransform* mat = node->asTransform()->asMatrixTransform();
	osg::Matrix m = mat->getMatrix();
	osg::Matrix mm;
	mm.identity();
	mm.makeScale(osg::Vec3d(1.1, 1.1, 1.1));
	mat->setMatrix(m*mm);
}

osgViewer::Viewer* Scene_Viewer::getViewer(){
	return m_pViewerPtr;
}

osg::ref_ptr<osg::Node> Scene_Viewer::Find_PartofHuman(osg::ref_ptr<osg::Node> node, QString part_name){
	if (!node->asGroup())return NULL;
	int size = node->asGroup()->getNumChildren();
	for (int i = 0; i < size; i++){
		if (node->asGroup()->getChild(i)->asTransform()){
			if (node->asGroup()->getChild(i)->getName() == part_name.toStdString()){
				return node->asGroup()->getChild(i);
			}
			else{
				osg::ref_ptr<osg::Node> n = Find_PartofHuman(node->asGroup()->getChild(i), part_name);
				if (n.get()){ return n; }
			}
		}
	}
	return NULL;
}

void Scene_Viewer::Update_Frame(){
	m_pViewerPtr->frame();
}


void Scene_Viewer::Slot_Frame(){
	Update_Frame();
}
