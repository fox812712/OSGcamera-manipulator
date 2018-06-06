
#include <QFileDialog>
#include <osgFX/Scribe>
#include <osg/MatrixTransform>

#include "sceneViewer.h"
#include "model_change.h"
#include "GraphicsWindowQt.h"

model_change::model_change(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	InitlizeWt();
	signal_slots();
}
model_change::~model_change()
{
}

void model_change::signal_slots(){
	connect(ui.actionAdd_Model, &QAction::triggered, this, &model_change::Slot_LoadHumanmodel);
	connect(ui.treeView, &QTreeView::clicked, this, &model_change::Slot_Itemclicked);
}

void model_change::InitlizeWt(){

	osgQt::GLWidget* w = new osgQt::GLWidget;
	m_pSceneViewer = new Scene_Viewer(w);

	QHBoxLayout* layout= new QHBoxLayout;
	layout->addWidget(w);
	layout->setContentsMargins(0, 0, 0, 0);
	ui.renderwt->setLayout(layout);
	ui.renderwt->setContentsMargins(0, 0, 0, 0);

	m_pmodel = new QStandardItemModel;
	ui.treeView->setModel(m_pmodel);
	m_prootitem = new QStandardItem("Root");
	m_pmodel->appendRow(m_prootitem);

	m_pSceneViewer->Load_HumanModel("E:\\man.OSG");
	ParseNode();

	m_pSceneViewer->Set_F(std::bind(&model_change::Set_Pos, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	m_pSceneViewer->Set_Frusm(std::bind(&model_change::Set_Frusm, this, std::placeholders::_1,
		std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6 ));
	m_pSceneViewer->Set_Matrixstr(std::bind(&model_change::Set_Matrixstr, this, std::placeholders::_1, std::placeholders::_2));
	m_pSceneViewer->Set_Detailstr(std::bind(&model_change::Set_Detailstr, this, std::placeholders::_1, std::placeholders::_2,
		std::placeholders::_3, std::placeholders::_4));

}

void model_change::ParseNode(){
	osg::ref_ptr<osg::Node> node = m_pSceneViewer->Get_HumanNode();
	if (node == NULL)return;
	ParseNode(node, m_prootitem);
	ui.treeView->expandAll();
}

void model_change::Set_Pos(osg::Vec3d eye, osg::Vec3d center, osg::Vec3d up){

	ui.doubleSpinBox->setValue(eye.x());
	ui.doubleSpinBox_2->setValue(eye.y());
	ui.doubleSpinBox_3->setValue(eye.z());

	ui.doubleSpinBox_4->setValue(center.x());
	ui.doubleSpinBox_5->setValue(center.y());
	ui.doubleSpinBox_6->setValue(center.z());

	ui.doubleSpinBox_7->setValue(up.x());
	ui.doubleSpinBox_8->setValue(up.y());
	ui.doubleSpinBox_9->setValue(up.x());
}

void model_change::Set_Frusm(double l, double r, double d, double u, double n, double f){
	ui.left->setValue(l);
	ui.right->setValue(r);
	ui.down->setValue(d);
	ui.up->setValue(u);
	ui.nearval->setValue(n);
	ui.farval->setValue(f);
}

void model_change::Set_Matrixstr(std::string str, int type){
	if (type == 1){
		ui.textEdit_1->setText(QString::fromStdString(str));
	}
	else if (type == 2){
		ui.textEdit_3->setText(QString::fromStdString(str));
	}
	else if (type == 3){
		ui.textEdit_4->setText(QString::fromStdString(str));
	}
}

void model_change::Set_Detailstr(std::string str1, std::string str2, std::string str3, int type){

	if (type == 1){
		ui.lineEdit->setText(QString::fromStdString(str1));
		ui.lineEdit_2->setText(QString::fromStdString(str2));
		ui.lineEdit_3->setText(QString::fromStdString(str3));
	}
	else if (type == 2){
		ui.lineEdit_4->setText(QString::fromStdString(str1));
		ui.lineEdit_5->setText(QString::fromStdString(str2));
		ui.lineEdit_6->setText(QString::fromStdString(str3));
	}
	else if (type == 3){
		ui.lineEdit_7->setText(QString::fromStdString(str1));
		ui.lineEdit_8->setText(QString::fromStdString(str2));
		ui.lineEdit_9->setText(QString::fromStdString(str3));
	}
}

void model_change::ParseNode(osg::ref_ptr<osg::Node> node_parent, QStandardItem* item){
	osg::Group* gp = node_parent->asGroup();
	if (!gp)return;
	int size = gp->getNumChildren();
	for (int i = 0; i < size; i++){
		osg::ref_ptr<osg::Node> node = gp->getChild(i);
		if (node->asTransform()){
			//if (node->asTransform()->asMatrixTransform()){
			//	QStandardItem* item_child = new QStandardItem(QString::fromStdString(node->getName()));
			//	item->appendRow(item_child);
			//	void* data = (void*)(node);
			//	item->setData(QVariant::fromValue(data));
			//	if (node->getName() == "man_01_zhengti_02"){
			//		int a = 123;
			//	}
			//	ParseNode(node, item_child);
			//}
			QStandardItem* item_child = new QStandardItem(QString::fromStdString(node->getName()));
			item->appendRow(item_child);
			void* data = (void*)(node);
			item->setData(QVariant::fromValue(data));
			if (node->getName() == "man_01_zhengti_02"){
				int a = 123;
			}
			ParseNode(node, item_child);
		}
		//else{
		//	QStandardItem* item_child = new QStandardItem(QString::fromStdString(node->getName()));
		//	item->appendRow(item_child);
		//	void* data = (void*)(node);
		//	item->setData(QVariant::fromValue(data));
		//}
	}
}

void model_change::Slot_LoadHumanmodel(){

	//QString path = QFileDialog::getOpenFileName(nullptr, QString(), QString(), QString("Modelfile(*.OSG *.osg)"));
	//if (path.isEmpty())return;
	//m_pSceneViewer->Load_HumanModel(path);
	//ParseNode();
}

void model_change::Slot_Itemclicked(QModelIndex index){

	QStandardItem* item = m_pmodel->itemFromIndex(index);
	if (!item)return;
	osg::ref_ptr<osg::Node> node = m_pSceneViewer->Find_PartofHuman(item->text());
	if (node.get() && node->asTransform()){
		osg::MatrixTransform* mat_node = node->asTransform()->asMatrixTransform();
		osg::Matrix m = mat_node->getMatrix();
		osg::Matrix mm;
		mm.makeScale(osg::Vec3d(1.1,1.1,1.1));
		mat_node->setMatrix(m*mm);
	}

}

