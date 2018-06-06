#include "Handler_NavigateManipulator.h"
//#include "../utils/utils.h"
#include <osg/Plane>
#include <osg/Depth>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgViewer/View>
#include <osgUtil/LineSegmentIntersector>

#include <iostream>

using namespace osg;
using namespace osgGA;
using namespace osgUtil;

NavigateManipulator::NavigateManipulator()
    : inherited(),
    m_mode(NONE),
    m_bLockHAxis(false),
    m_bLockAxisCenter(false),
    m_moveXOffset(0.0),
    m_moveYOffset(0.0),
    m_currentTime(0.0)
{
    _flags |= SET_CENTER_ON_WHEEL_FORWARD_MOVEMENT;
    setAnimationTime(0.4);
}
NavigateManipulator::NavigateManipulator(const NavigateManipulator& uim, const CopyOp& copyOp)
    : osg::Object(uim, copyOp),
    inherited(uim, copyOp),
    m_mode(uim.m_mode),
    m_bLockHAxis(uim.m_bLockHAxis),
    m_bLockAxisCenter(uim.m_bLockAxisCenter),
    m_axisCenter(uim.m_axisCenter),
    m_moveXOffset(uim.m_moveXOffset),
    m_moveYOffset(uim.m_moveYOffset)
{
}

void NavigateManipulator::setByMatrix(const osg::Matrixd& matrix)
{
    inherited::setByMatrix(matrix);
    setAxisCenter(_center);
}

void NavigateManipulator::setTransformation(const osg::Vec3d& eye, const osg::Quat& rotation)
{
    inherited::setTransformation(eye, rotation);
    setAxisCenter(_center);
}
void NavigateManipulator::setTransformation(const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up)
{
    inherited::setTransformation(eye, center, up);
    setAxisCenter(_center);
}

void NavigateManipulator::setAxisCenter(const osg::Vec3d & axisCenter)
{
    if (!m_bLockAxisCenter) {
        m_axisCenter = axisCenter;
    }
}

void NavigateManipulator::lockAxisCenter(const osg::Vec3d & axisCenter)
{
    m_axisCenter = axisCenter;
    m_bLockAxisCenter = true;
}
void NavigateManipulator::releaseAxisCenter()
{
    m_bLockAxisCenter = false;
}

void NavigateManipulator::setNode(osg::Node* node)
{
    return inherited::setNode(node);
}

void NavigateManipulator::home(double currentTime)
{
    inherited::home(currentTime);
    m_moveXOffset = 0.0;
    m_moveYOffset = 0.0;
    releaseAxisCenter();
    setAxisCenter(_homeCenter);
}
void NavigateManipulator::home(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    inherited::home(ea, us);
    m_moveXOffset = 0.0;
    m_moveYOffset = 0.0;
    releaseAxisCenter();
    setAxisCenter(_homeCenter);
}

bool NavigateManipulator::handleFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    m_currentTime = ea.getTime();
    return inherited::handleFrame(ea, us);
}

bool NavigateManipulator::handleKeyDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    return inherited::handleKeyDown(ea, us);
}
bool NavigateManipulator::handleKeyUp(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    return inherited::handleKeyUp(ea, us);
}

bool NavigateManipulator::handleMouseWheel(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    osgGA::GUIEventAdapter::ScrollingMotion sm = ea.getScrollingMotion();

    // 选定轴心
    if (_flags & SET_CENTER_ON_WHEEL_FORWARD_MOVEMENT)
    {
        if (sm == GUIEventAdapter::SCROLL_DOWN || sm == GUIEventAdapter::SCROLL_UP)
        {
            // 选定轴心
            prepareRotate(ea, us);
        }
    }
    switch (sm)
    {
    case GUIEventAdapter::SCROLL_DOWN:
    {
        zoomModel(-_wheelZoomFactor, true);
        us.requestRedraw();
        us.requestContinuousUpdate(isAnimating() || _thrown);
        return true;
    }
    case GUIEventAdapter::SCROLL_UP:
    {
        zoomModel(_wheelZoomFactor, true);
        us.requestRedraw();
        us.requestContinuousUpdate(isAnimating() || _thrown);
        return true;
    }
    default: // unhandled mouse scrolling motion
        return false;
    }
    return false;
}

void NavigateManipulator::prepareRotate(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    if (!m_bLockAxisCenter) {
        osg::Vec3d newAxisCenter = getAxisCenter();
        pickPointByMousePointerIntersection(ea, us, newAxisCenter);
        setAxisCenter(newAxisCenter);
    }
    // 旋转矩阵坐标系是,x向左,y向上,z向屏幕外
    osg::Matrixd rotMatrix(_rotation);
    // 场景中,看向屏幕的方向为y,相当于旋转矩阵坐标系的-z方向
    osg::Vec3d front = osg::Vec3(0, 0, -1) * rotMatrix;
    osg::Vec3d v0 = getAxisCenter() - _center;
    double dist = v0 * front;
    _center += front * dist;
    _distance += dist;
}

bool NavigateManipulator::handleMousePush(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    // 选定平移中心(1.鼠标中键移动, 2.平移模式,左键点击)
    if (ea.getButton() == ea.MIDDLE_MOUSE_BUTTON ||
        (ea.getButton() == ea.LEFT_MOUSE_BUTTON && m_mode == PAN))
    {
        preparePan(ea, us);
    }

    return inherited::handleMousePush(ea, us);
}

void NavigateManipulator::preparePan(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    osg::Vec3d moveCenter;
    pickPointByMousePointerIntersection(ea, us, moveCenter);

    // 将视点拉到moveCenter
    osg::Matrixd rotMatrix(_rotation);
    osg::Vec3d front = osg::Vec3(0, 0, -1) * rotMatrix;
    osg::Vec3d v0 = moveCenter - _center;
    double dist = v0 * front;
    _center += front * dist;
    _distance += dist;

    // 获取相机参数,并且计算平移参数
    double left, right, top, bottom, zNear, zFar;
    if (us.asView()->getCamera()->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar))
    {
        double nearW = right - left;
        double nearH = top - bottom;
        m_moveXOffset = nearW * _distance / zNear / ea.getWindowWidth();
        m_moveYOffset = nearH * _distance / zNear / ea.getWindowHeight();
    }
    else // 获取不到的话,那么不是透视矩阵,计算方式完全不同了啊!
    {
        m_moveXOffset = 0.0;
        m_moveYOffset = 0.0;
    }
}

bool NavigateManipulator::performMovementLeftMouseButton(const double /*eventTimeDelta*/, const double dx, const double dy)
{
    switch (m_mode)
    {
    case NONE: return true; // 不处理左键信息
    case PAN:
    {
        float dWinX = _ga_t0->getX() - _ga_t1->getX();
        float dWinY = _ga_t0->getY() - _ga_t1->getY();
        panModel(-dWinX*m_moveXOffset, -dWinY*m_moveYOffset);
        break;
    }
    case ROTATE: rotateWithFixedVertical(dx, dy); break;
    case ZOOM: zoomModel(-dy, true); break;
    case SWEEP: sweepRotate(dx, dy); break;
    default: break;
    }

    return true;
}

bool NavigateManipulator::performMovementMiddleMouseButton(const double /*eventTimeDelta*/, const double /*dx*/, const double /*dy*/)
{
    // 获取平移的偏移
    float dWinX = _ga_t0->getX() - _ga_t1->getX();
    float dWinY = _ga_t0->getY() - _ga_t1->getY();
    panModel(-dWinX * m_moveXOffset, -dWinY * m_moveYOffset);
    return false;
}

bool NavigateManipulator::performMovementRightMouseButton(const double /*eventTimeDelta*/, const double dx, const double dy)
{
    rotateWithFixedVertical(dx, dy);
    return false;
}

void NavigateManipulator::applyAnimationStep(const double currentProgress, const double /*prevProgress*/)
{
    NavigateAnimationData * ad = dynamic_cast<NavigateAnimationData*>(_animationData.get());
    if (!ad) return;

    double newDist = ad->_preDist * (1 - currentProgress) + ad->_dstDist * currentProgress;
    osg::Vec3d newCtr = ad->_preCtr * (1 - currentProgress) + ad->_dstCtr * currentProgress;
    osg::Quat newRot;
    newRot.slerp(currentProgress, ad->_preRot, ad->_dstRot);

    if (getVerticalAxisFixed() && currentProgress >= 1.f)
    {
        CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
        Vec3d localUp = getUpVector(coordinateFrame);
        fixVerticalAxis(newRot, localUp, false);
    }
    _distance = newDist;
    _center = newCtr;
    _rotation = newRot;
}

void NavigateManipulator::NavigateAnimationData::start(
    const osg::Vec3d& preCtr, const osg::Quat& preRot, double preDist,
    const osg::Vec3d& dstCtr, const osg::Quat& dstRot, double dstDist,
    const double startTime)
{
    AnimationData::start(startTime);
    _dstCtr = dstCtr;
    _dstRot = dstRot;
    _dstDist = dstDist;
    _preCtr = preCtr;
    _preRot = preRot;
    _preDist = preDist;
}

void NavigateManipulator::transformAnimation(const osg::Vec3d& center, const osg::Quat& rot, double dist)
{
    NavigateAnimationData *ad = dynamic_cast<NavigateAnimationData*>(_animationData.get());
    if (ad)
    {
        double dstDist = dist;
        if (dstDist <= 0.0)
        {
            dstDist = _distance;
        }
        ad->start(_center, _rotation, _distance, center, rot, dstDist, m_currentTime);
        setAxisCenter(center);
    }
}
void NavigateManipulator::transformAnimation(const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up)
{
    NavigateAnimationData *ad = dynamic_cast<NavigateAnimationData*>(_animationData.get());
    if (ad)
    {
        osg::Vec3d lv(center - eye);
        osg::Vec3d f(lv);
        f.normalize();
        osg::Vec3d s(f^up);
        s.normalize();
        osg::Vec3d u(s^f);
        u.normalize();

        osg::Matrixd rotation_matrix(s[0], u[0], -f[0], 0.0f,
            s[1], u[1], -f[1], 0.0f,
            s[2], u[2], -f[2], 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f);

        osg::Vec3d dstCtr = center;
        double dstDist = lv.length();
        osg::Quat dstRot = rotation_matrix.getRotate().inverse();

        // fix current rotation
        if (getVerticalAxisFixed())
            fixVerticalAxis(dstCtr, dstRot, true);
        ad->start(_center, _rotation, _distance, dstCtr, dstRot, dstDist, m_currentTime);
        setAxisCenter(center);
    }
}

void NavigateManipulator::sweepRotate(const double dx, const double dy)
{
    CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    Vec3d localUp = getUpVector(coordinateFrame);
    osg::Vec3d camPos = getMatrix().getTrans();
    osg::Vec3d f1 = osg::Vec3d(0, 0, -1) * osg::Matrixd::rotate(_rotation);
    rotateYawPitch(_rotation, dx, dy, localUp);
    osg::Vec3d f2 = osg::Vec3d(0, 0, -1) * osg::Matrixd::rotate(_rotation);
    _center = _center * osg::Matrixd::translate(-camPos) * osg::Matrixd::rotate(f1, f2) * osg::Matrixd::translate(camPos);
    setAxisCenter(_center);
}
void NavigateManipulator::rotateWithFixedVertical(const float dx, const float dy)
{
    CoordinateFrame coordinateFrame = getCoordinateFrame(_center);
    Vec3d localUp = getUpVector(coordinateFrame);
    rotateWithFixedVertical(dx, dy, localUp);
}
void NavigateManipulator::rotateWithFixedVertical(const float dx, const float dy, const Vec3f& up)
{
    osg::Vec3d offset;
    preRotate(offset);
    rotateYawPitch(_rotation, dx, m_bLockHAxis ? 0 : dy, up);
    postRotate(offset);
}

void NavigateManipulator::preRotate(osg::Vec3d& offset)
{
    osg::Matrixd viewMat = getInverseMatrix();
    offset = getAxisCenter()*viewMat - _center*viewMat;
    _center = getAxisCenter();
}

void NavigateManipulator::postRotate(const osg::Vec3d& offset)
{
    osg::Matrixd rotMatrix(_rotation);
    osg::Matrixd viewmat = getInverseMatrix();
    osg::Vec3d v = _center * viewmat;
    v = v - offset;
    _center = v * osg::Matrixd::inverse(viewmat);
}

void NavigateManipulator::zoomModel(const float dy, bool pushForwardIfNeeded)
{
    float scale = 1.0f + dy;
    osg::Matrixd rotMatrix(_rotation);
    osg::Vec3d front = osg::Vec3d(0, 0, -1) * rotMatrix;
    // minimum distance
    float minDist = _minimumDistance;
    if (getRelativeFlag(_minimumDistanceFlagIndex))
        minDist *= _modelSize;

    // 相机前进(后退)的方向,向着旋转轴心方向前进(后退)
    osg::Vec3d camPos = getMatrix().getTrans();
    osg::Vec3d moveDir = getAxisCenter() - camPos;
    moveDir.normalize();

    // 求出相机前进(后退)的距离,相机前进(后退)的位置
    osg::Vec3d moveVec = moveDir * (_distance * scale - _distance); // 相机移动的向量
    camPos += moveVec; // 相机移动后的位置
    float frontDist = moveVec * front; // moveVec在front方向的投影
    float newDist = _distance - frontDist;
    // 如果前进距离大于最小距离,那么我们只移动相机,视点仅平移. 反之,我们同步移动相机和相机视点(在-z方向移动)
    if (newDist > minDist)
    {
        _distance = newDist; // 缩短距离
        _center = camPos + front * _distance; // 配合移动相机位置.其实是保证只移动相机,视点仅平移
    }
    else
    {
        if (pushForwardIfNeeded)
        {
            _center = camPos + front * _distance;
        }
    }
}

bool NavigateManipulator::pickPointByMousePointerIntersection(
    const osgGA::GUIEventAdapter& ea,
    osgGA::GUIActionAdapter& us,
    osg::Vec3d& pickPt)
{
    osg::View* view = us.asView();
    if (!view) return false;
    osg::Camera *camera = view->getCamera();
    if (!camera) return false;

    // prepare variables
    float x = 0.f, y = 0.f;
    pickScreenPoint(ea, x, y);
    x = (x - ea.getXmin()) / (ea.getXmax() - ea.getXmin());
    y = (y - ea.getYmin()) / (ea.getYmax() - ea.getYmin());
    osgUtil::LineSegmentIntersector::CoordinateFrame cf;
    osg::Viewport *vp = camera->getViewport();
    if (vp) {
        cf = osgUtil::Intersector::WINDOW;
        x *= vp->width();
        y *= vp->height();
    }
    else {
        cf = osgUtil::Intersector::PROJECTION;
    }

    // perform intersection computation
    ref_ptr< osgUtil::LineSegmentIntersector > picker = new osgUtil::LineSegmentIntersector(cf, x, y);
    osgUtil::IntersectionVisitor iv(picker.get());
    iv.setTraversalMask(getIntersectTraversalMask());
    camera->accept(iv);

    // 如果有相交点,那么返回相交点
    if (picker->containsIntersections())
    {
        osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
        pickPt = intersection.getWorldIntersectPoint();
        return true;
    }

    // 我们固定前进一个距离,从可进可退的,这样子是不是好一些
    return pickPointByMousePointerFromSpace(ea, us, pickPt);
}

bool NavigateManipulator::pickPointByMousePointerFromSpace(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us, osg::Vec3d& pickPt)
{
    osg::View* view = us.asView();
    if (!view) return false;
    osg::Camera *camera = view->getCamera();
    if (!camera) return false;
    osg::Viewport *vp = camera->getViewport();
    if (!vp) return false;

    float x = 0.f, y = 0.f;
    pickScreenPoint(ea, x, y);

    // 获取MVPW矩阵
    osg::Matrix winMat = vp->computeWindowMatrix();
    osg::Matrixd viewMat = getInverseMatrix();
    const osg::Matrixd& projMat = camera->getProjectionMatrix();
    osg::Matrixd invMVPW = osg::Matrixd::inverse(viewMat * projMat * winMat);
    osg::Vec3d ray = osg::Vec3d(x, y, 1.0) * invMVPW - osg::Vec3d(x, y, 0.0) * invMVPW;
    ray.normalize();
    osg::Vec3d camPos = getMatrix().getTrans();
    osg::Vec3d front = _center - camPos;
    front.normalize();
    double cosValue = front * ray;
    if (fabs(cosValue) < 1e-6) {
        cosValue = 1;
    }
    pickPt = camPos + ray * (_distance / cosValue);
    return true;
}

void NavigateManipulator::pickScreenPoint(const osgGA::GUIEventAdapter& ea, float& x, float& y)
{
    x = ea.getX();
    y = ea.getY();
}
