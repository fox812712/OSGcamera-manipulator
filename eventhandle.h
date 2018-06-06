#pragma once
#pragma execution_character_set("utf-8")

#include <osgGA/EventHandler>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

class Scene_Viewer;
class ObserEventhandle :public osgGA::GUIEventHandler{
public:
	ObserEventhandle(Scene_Viewer* viewer);
	~ObserEventhandle();
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& es);
protected:
	osgViewer::Viewer* pviewer;
	Scene_Viewer* scene_viewer;

};