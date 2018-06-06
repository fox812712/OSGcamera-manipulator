#ifndef WISDOMCLOTH_D3_ASPACESC_NAVIGATEMANIPULATOR_H
#define WISDOMCLOTH_D3_ASPACESC_NAVIGATEMANIPULATOR_H 1

#include <osgGA/OrbitManipulator>

class NavigateManipulator : public osgGA::OrbitManipulator
{
    typedef osgGA::OrbitManipulator inherited;

public:
    NavigateManipulator();
    NavigateManipulator(const NavigateManipulator& m, const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY);

    META_Object(osgGA, NavigateManipulator);

    // 以下模式左键不同,其他为中键平移,滚轮缩放,shift+中键旋转.
    enum NavigateMode
    {
        NONE,   // 左键什么都没有
        PAN,	// 左键为平移
        ZOOM,   // 左键为缩放
        ROTATE,	// 左键旋转
        SWEEP	// 左键为环视
    };

    void setNavigateMode(NavigateMode mode) { m_mode = mode; }
    NavigateMode getNavigateMode() const { return m_mode; }

    void transformAnimation(const osg::Vec3d& center, const osg::Quat& rot, double dist); // dist如果<=0, 则使用当前dist
    void transformAnimation(const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up);

    virtual void setAxisCenter(const osg::Vec3d & axisCenter);
    const osg::Vec3d& getAxisCenter() const { return m_axisCenter; }

    virtual void lockAxisCenter(const osg::Vec3d & axisCenter);
    void releaseAxisCenter();

    // 锁定水平旋转轴
    void lockHAxis() { m_bLockHAxis = true; }
    void releaseHAxis() { m_bLockHAxis = false; }
    // 重载函数
public:

    virtual void setNode(osg::Node*) override;
    virtual void home(double) override;
    virtual void home(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;

    // 暴露接口,让外界调用旋转
    virtual void rotateWithFixedVertical(const float dx, const float dy) override;

    virtual void setByMatrix(const osg::Matrixd& matrix) override;
    virtual void setTransformation(const osg::Vec3d& eye, const osg::Quat& rotation) override;
    virtual void setTransformation(const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up) override;

    // 重载函数
protected:
    // 处理各种情况
    virtual bool handleFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    virtual bool handleKeyUp(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    virtual bool handleKeyDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    virtual bool handleMouseWheel(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;
    // push & release 
    virtual bool handleMousePush(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;

    virtual bool performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy) override;
    virtual bool performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy) override;
    virtual bool performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy) override;
    virtual void applyAnimationStep(const double currentProgress, const double prevProgress) override;

    // orbit
    virtual void rotateWithFixedVertical(const float dx, const float dy, const osg::Vec3f& up) override;
    virtual void zoomModel(const float dy, bool pushForwardIfNeeded = true) override;

    // 计算选取的屏幕点。为了让MultiTouch也能正确计算，毕竟没有鼠标，多触点下要稍微复杂一些
    // 输入为ea，包含有触摸点信息
    // 输出x，y，默认为ea.getX(), ea.getY()
    virtual void pickScreenPoint(const osgGA::GUIEventAdapter& ea, float& x, float& y);

    // 新定义的函数
protected:
    // 旋转前,首先需要重新设定distance和center.输出相机坐标系下的offset
    void preRotate(osg::Vec3d& offset);
    // 旋转后,distance是不用变了,但是center需要绕着axis旋转.传入offset
    void postRotate(const osg::Vec3d& offset);
    // 准备旋转的参数,决定旋转中心,调整视点位置
    void prepareRotate(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
    // 计算各种平移参数.在进行平移前调用,调整视点位置
    void preparePan(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
    // 环视旋转
    void sweepRotate(const double dx, const double dy);

    // 利用鼠标拾取旋转/缩放轴心, 第二个函数是选取平移中心
    bool pickPointByMousePointerIntersection(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us, osg::Vec3d& pickPt);
    // 从虚空之中选取一个点
    bool pickPointByMousePointerFromSpace(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us, osg::Vec3d& pickPt);

    // 锁定水平旋转轴
    bool m_bLockHAxis;
    // 锁定旋转中心
    bool m_bLockAxisCenter;

    // 模式选择
    NavigateMode m_mode; // default = NONE

    // 两个相机参数,临时选取用,在drag的时候用到
    double m_moveXOffset; // 平移参数, x方向的平移缩放
    double m_moveYOffset; // 平移参数, y方向的平移缩放
    osg::Vec3d m_axisCenter; // 旋转/缩放轴心.(世界坐标系下)

    double m_currentTime; // 记录下当前的时间

    class NavigateAnimationData : public AnimationData
    {
    public:
        osg::Vec3d _preCtr;
        osg::Quat _preRot;
        double _preDist;
        osg::Vec3d _dstCtr;
        osg::Quat _dstRot;
        double _dstDist;
        void start(
            const osg::Vec3d& preCtr, const osg::Quat& preRot, double preDist,
            const osg::Vec3d& dstCtr, const osg::Quat& dstRot, double dstDist,
            const double startTime);
    };
    virtual void allocAnimationData() override { _animationData = new NavigateAnimationData(); }
};


#endif
