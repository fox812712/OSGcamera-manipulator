#include "Handler_WSNavigateManipulator.h"

#include <iostream>
#include <osg/io_utils>
#include <osgUtil/LineSegmentIntersector>


static const double RESTORE_CURSOR_TIME = 0.2;


WSNavigateManipulator::WSNavigateManipulator(osgQt::GLWidget* w):mOSGWidget(w)
{
    mDoWheel = false;
    mDoWheelTime = 0;
}

bool WSNavigateManipulator::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
    // 恢复放大镜鼠标样式为默认
    if (mDoWheel && ea.getTime() - mDoWheelTime > RESTORE_CURSOR_TIME)
    {
        mDoWheel = false;
    }

    return inherited::handle(ea, us);
}

bool WSNavigateManipulator::handleKeyUp( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
    return inherited::handleKeyUp(ea, us);
}
bool WSNavigateManipulator::handleKeyDown( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us ){
    return inherited::handleKeyDown(ea, us);
}

bool WSNavigateManipulator::handleMouseWheel( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us ){
    if (mOSGWidget)
    {
        mDoWheel = true;
        mDoWheelTime = ea.getTime();
    }
    return inherited::handleMouseWheel(ea, us);
}

bool WSNavigateManipulator::handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us ){

    return inherited::handleMousePush(ea, us);
}

bool WSNavigateManipulator::handleMouseRelease(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us){

    return inherited::handleMouseRelease(ea, us);
}

bool WSNavigateManipulator::performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy )
{
    return inherited::performMovementLeftMouseButton(eventTimeDelta, dx, dy);
}

bool WSNavigateManipulator::performMovementMiddleMouseButton( const double eventTimeDelta, const double dx, const double dy ){
    return inherited::performMovementMiddleMouseButton(eventTimeDelta, dx, dy);
}

bool WSNavigateManipulator::performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy ){
    return inherited::performMovementRightMouseButton(eventTimeDelta, dx, dy);
}


