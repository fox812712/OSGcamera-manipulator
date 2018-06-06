#ifndef WSNAVIGATEMANIPULATORSC_H
#define WSNAVIGATEMANIPULATORSC_H

#include "GraphicsWindowQt.h"
#include "Handler_NavigateManipulator.h"

#include <osg/MatrixTransform>

class WSNavigateManipulator : public NavigateManipulator
{
    typedef NavigateManipulator inherited;
public:
    WSNavigateManipulator(osgQt::GLWidget* w);

protected:

    virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us) override;

    virtual bool handleKeyUp(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    virtual bool handleKeyDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    virtual bool handleMouseWheel(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    // push & release
    virtual bool handleMousePush(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    virtual bool handleMouseRelease(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us) override;
    virtual bool performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy) override;
    virtual bool performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy) override;
    virtual bool performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy) override;

private:

private:
    osgQt::GLWidget* mOSGWidget;

    bool mDoWheel; // 是否执行了缩放操作
    double mDoWheelTime; // 执行缩放操作时候的时间
};


#endif
