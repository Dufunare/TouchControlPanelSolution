#include "GLCoordinateWidget.h"
#include "GlPoseMath.h"
#include "TeleopGeometry.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <QFont>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <QVector4D>

namespace {
using V=GLCoordinateWidget::Vertex;
constexpr float pi=3.14159265358979323846f;
const QVector3D kRed(0.98f,0.39f,0.42f),kGreen(0.34f,0.91f,0.69f),
                kBlue(0.37f,0.68f,1.0f),kCyan(0.34f,0.85f,0.99f);
void vertex(std::vector<V>& out,QVector3D p,QVector3D c) {
    out.push_back({p.x(),p.y(),p.z(),c.x(),c.y(),c.z()});
}
void line(std::vector<V>& out,QVector3D a,QVector3D b,QVector3D c) {
    vertex(out,a,c);vertex(out,b,c);
}
void triangle(std::vector<V>& out,QVector3D a,QVector3D b,QVector3D c,QVector3D color) {
    vertex(out,a,color);vertex(out,b,color);vertex(out,c,color);
}
void cylinder(std::vector<V>& out,QVector3D start,QVector3D end,
              float r1,float r2,QVector3D color,int sides=12) {
    const QVector3D direction=(end-start).normalized();
    const QVector3D reference=std::abs(direction.y())<0.8f?
        QVector3D(0,1,0):QVector3D(1,0,0);
    const QVector3D u=QVector3D::crossProduct(direction,reference).normalized();
    const QVector3D v=QVector3D::crossProduct(direction,u).normalized();
    for(int i=0;i<sides;++i) {
        const float a=2*pi*i/sides,b=2*pi*(i+1)/sides;
        const QVector3D x=u*std::cos(a)+v*std::sin(a);
        const QVector3D y=u*std::cos(b)+v*std::sin(b);
        const QVector3D p0=start+x*r1,p1=start+y*r1,p2=end+x*r2,p3=end+y*r2;
        triangle(out,p0,p1,p2,color);triangle(out,p1,p3,p2,color);
        triangle(out,start,p1,p0,color*0.65f);
        triangle(out,end,p2,p3,color*0.86f);
    }
}
QVector3D transformed(const QMatrix4x4& m,QVector3D p) {
    return (m*QVector4D(p,1)).toVector3D();
}
void localCylinder(std::vector<V>& out,const QMatrix4x4& model,
                   float startZ,float endZ,float startRadius,float endRadius,
                   QVector3D color,int sides=14) {
    const QVector3D s(0,0,startZ),e(0,0,endZ);
    const float da=2*pi/sides;
    for(int i=0;i<sides;++i) {
        const float a=da*i,b=da*(i+1);
        const QVector3D p0(std::cos(a)*startRadius,std::sin(a)*startRadius,startZ);
        const QVector3D p1(std::cos(b)*startRadius,std::sin(b)*startRadius,startZ);
        const QVector3D p2(std::cos(a)*endRadius,std::sin(a)*endRadius,endZ);
        const QVector3D p3(std::cos(b)*endRadius,std::sin(b)*endRadius,endZ);
        triangle(out,transformed(model,p0),transformed(model,p1),transformed(model,p2),color);
        triangle(out,transformed(model,p1),transformed(model,p3),transformed(model,p2),color);
    }
    Q_UNUSED(s);Q_UNUSED(e);
}
constexpr char vertShader[]=R"glsl(
#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 color;
uniform mat4 mvp;
out vec3 colorOut;
void main() {
    gl_Position=mvp*vec4(position,1.0);
    colorOut=color;
}
)glsl";
constexpr char fragShader[]=R"glsl(
#version 330 core
in vec3 colorOut;
out vec4 outputColor;
void main() { outputColor=vec4(colorOut,1.0); }
)glsl";
}

GLCoordinateWidget::GLCoordinateWidget(QWidget* parent):QOpenGLWidget(parent) {
    setMinimumSize(350,240);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
}
GLCoordinateWidget::~GLCoordinateWidget() {
    if(context()) {
        makeCurrent();
        releaseResources();
        doneCurrent();
    }
}
void GLCoordinateWidget::setDeviceState(const touchpanel::DeviceState& state) {
    m_state=state;
    if(state.valid) {
        const QVector3D p=renderPosition();
        if(!m_haveValidPose || (m_trail.empty() || (p-m_trail.back()).length()>1.5f)) {
            m_trail.push_back(p);
            if(m_trail.size()>250) m_trail.pop_front();
        }
        m_haveValidPose=true;
        if(m_follow)m_cameraTarget=p;
    } else {
        m_haveValidPose=false;
    }
    update();
}
bool GLCoordinateWidget::captureDisplayOrigin() {
    if(!m_origin.capture(m_state.positionMm,m_state.valid))return false;
    m_trail.clear();m_cameraTarget={};
    update();return true;
}
void GLCoordinateWidget::clearDisplayOrigin() {
    m_origin.clear();m_trail.clear();m_cameraTarget={};
    update();
}
void GLCoordinateWidget::clearTrail() {m_trail.clear();update();}
void GLCoordinateWidget::resetCamera() {
    m_follow=false;m_cameraTarget={};m_yaw=43;m_pitch=24;m_distance=430;update();
}
void GLCoordinateWidget::focusOnPen() {
    if(!m_state.valid)return;
    m_cameraTarget=renderPosition();m_follow=true;update();
}
QVector3D GLCoordinateWidget::renderPosition() const {
    const auto p=m_origin.render(m_state.positionMm);
    return QVector3D(float(p[0]),float(p[1]),float(p[2]));
}
void GLCoordinateWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.039f,0.054f,0.078f,1.0f);
    m_shader=std::make_unique<QOpenGLShaderProgram>();
    if(!m_shader->addShaderFromSourceCode(QOpenGLShader::Vertex,vertShader) ||
       !m_shader->addShaderFromSourceCode(QOpenGLShader::Fragment,fragShader) ||
       !m_shader->link()) {
        qWarning("GLCoordinateWidget shader failed: %s",qPrintable(m_shader->log()));
        m_shader.reset();return;
    }
    buildStaticScene();
}
void GLCoordinateWidget::resizeGL(int w,int h) {
    m_projection.setToIdentity();
    m_projection.perspective(43.0f,h>0?float(w)/float(h):1.0f,0.3f,4500.0f);
}
void GLCoordinateWidget::upload(QOpenGLVertexArrayObject& vao,QOpenGLBuffer& vbo,
                                const std::vector<Vertex>& verts) {
    if(!vao.isCreated())vao.create();
    if(!vbo.isCreated())vbo.create();
    m_shader->bind();
    vao.bind();vbo.bind();
    vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    if(!verts.empty())vbo.allocate(verts.data(),int(verts.size()*sizeof(Vertex)));
    else vbo.allocate(0);
    m_shader->enableAttributeArray(0);m_shader->enableAttributeArray(1);
    m_shader->setAttributeBuffer(0,GL_FLOAT,offsetof(Vertex,x),3,sizeof(Vertex));
    m_shader->setAttributeBuffer(1,GL_FLOAT,offsetof(Vertex,r),3,sizeof(Vertex));
    vbo.release();vao.release();m_shader->release();
}
void GLCoordinateWidget::draw(QOpenGLVertexArrayObject& vao,int count,
                              unsigned mode,float width) {
    if(count<=0 || !vao.isCreated())return;
    glLineWidth(width);
    vao.bind();glDrawArrays(mode,0,count);vao.release();
}
void GLCoordinateWidget::buildStaticScene() {
    std::vector<V> grid,axes;
    // Three reference planes with subtle multi-scale grid hierarchy.
    for(int i=-10;i<=10;++i) {
        const float a=float(i)*20;
        const QVector3D major=i%5==0?QVector3D(0.19f,0.28f,0.37f):
            QVector3D(0.095f,0.14f,0.20f);
        line(grid,{a,-120,-200},{a,-120,200},major);
        line(grid,{-200,-120,a},{200,-120,a},major);
    }
    for(int i=-8;i<=8;++i) {
        const float a=float(i)*25;
        const QVector3D c(0.095f,0.13f,0.17f);
        line(grid,{a,-180,0},{a,180,0},c);
        line(grid,{0,-180,a},{0,180,a},c);
    }
    const float extent=160;
    line(axes,{-extent,0,0},{extent,0,0},kRed);
    line(axes,{0,-extent,0},{0,extent,0},kGreen);
    line(axes,{0,0,-extent},{0,0,extent},kBlue);
    // Arrow heads, world origin marker.
    line(axes,{extent-9,-4,0},{extent,0,0},kRed);
    line(axes,{extent-9,4,0},{extent,0,0},kRed);
    line(axes,{0,extent-9,-4},{0,extent,0},kGreen);
    line(axes,{0,extent-9,4},{0,extent,0},kGreen);
    line(axes,{-4,0,extent-9},{0,0,extent},kBlue);
    line(axes,{4,0,extent-9},{0,0,extent},kBlue);
    upload(m_gridVao,m_gridVbo,grid);m_gridCount=int(grid.size());
    upload(m_axesVao,m_axesVbo,axes);m_axesCount=int(axes.size());
}
void GLCoordinateWidget::updateDynamicGeometry() {
    std::vector<V> trail,pen,solids;
    if(m_trail.size()>1)for(size_t i=1;i<m_trail.size();++i)
        line(trail,m_trail[i-1],m_trail[i],QVector3D(0.24f,0.52f,0.66f));
    m_trailCount=int(trail.size());
    upload(m_trailVao,m_trailVbo,trail);

    if(m_state.valid && m_state.orientationValid) {
        const auto rotation=m_state.rotationMatrix;
        const auto tip=renderPosition();
        QMatrix4x4 model;
        const auto& r=rotation;
        // Native OpenHaptics matrix is converted to row-major in TouchBackend;
        // QMatrix4x4 constructor accepts explicit row-major values.
        QMatrix4x4 local(
            float(r[0]),float(r[1]),float(r[2]),tip.x(),
            float(r[3]),float(r[4]),float(r[5]),tip.y(),
            float(r[6]),float(r[7]),float(r[8]),tip.z(),
            0,0,0,1);
        model=local;
        const QVector3D barrel(0.68f,0.77f,0.88f),grip(0.20f,0.27f,0.36f),
                        metal(0.82f,0.87f,0.94f);
        // Tip anchored exactly at HD_CURRENT_POSITION (local 0,0,0).
        // Shaft extends along local +Z. No force / robot command is generated.
        localCylinder(solids,model,0,6,0.12f,2.2f,kCyan,12);
        localCylinder(solids,model,6,13,2.2f,3.1f,metal,16);
        localCylinder(solids,model,13,52,3.1f,3.1f,barrel,16);
        localCylinder(solids,model,27,44,3.45f,3.45f,grip,16);
        localCylinder(solids,model,52,56,3.0f,3.0f,metal,16);
        localCylinder(solids,model,56,59,3.0f,2.4f,grip,16);
        // Fine rings around the grip, also good cues for orientation roll.
        for(int i=0;i<7;++i) {
            const float z=29.0f+i*2.0f;
            for(int j=0;j<16;++j) {
                const float a=2*pi*j/16,b=2*pi*(j+1)/16;
                line(pen,transformed(model,{3.53f*std::cos(a),3.53f*std::sin(a),z}),
                         transformed(model,{3.53f*std::cos(b),3.53f*std::sin(b),z}),
                         QVector3D(0.28f,0.45f,0.57f));
            }
        }
        // Orientation gizmo: local axes follow the actual 3x3 matrix.
        const QVector3D c=transformed(model,{0,0,16});
        line(pen,c,transformed(model,{24,0,16}),kRed);
        line(pen,c,transformed(model,{0,24,16}),kGreen);
        line(pen,c,transformed(model,{0,0,40}),kBlue);
        // Emphasized tip crosshair, useful when zooming out.
        for(int i=0;i<3;++i) {
            QVector3D e(0,0,0); e[i]=3;
            line(pen,tip-e,tip+e,kCyan);
        }
    }
    m_penCount=int(pen.size());m_solidCount=int(solids.size());
    upload(m_penVao,m_penVbo,pen);
    upload(m_solidVao,m_solidVbo,solids);
}
QMatrix4x4 GLCoordinateWidget::viewMatrix() const {
    const float a=m_yaw*pi/180,b=m_pitch*pi/180;
    QVector3D offset(m_distance*std::cos(b)*std::cos(a),
                    m_distance*std::sin(b),
                    m_distance*std::cos(b)*std::sin(a));
    QMatrix4x4 view;
    view.lookAt(m_cameraTarget+offset,m_cameraTarget,{0,1,0});
    return view;
}
void GLCoordinateWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    if(m_shader) {
        const QMatrix4x4 mvp=m_projection*viewMatrix();
        m_shader->bind();
        m_shader->setUniformValue("mvp",mvp);
        draw(m_gridVao,m_gridCount,GL_LINES,1);
        draw(m_axesVao,m_axesCount,GL_LINES,2);
        updateDynamicGeometry();
        draw(m_trailVao,m_trailCount,GL_LINES,2);
        glDisable(GL_CULL_FACE);
        draw(m_solidVao,m_solidCount,GL_TRIANGLES);
        glDisable(GL_DEPTH_TEST);
        draw(m_penVao,m_penCount,GL_LINES,2);
        glEnable(GL_DEPTH_TEST);
        m_shader->release();
    }
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const QColor fg(226,237,250),muted(132,157,185),accent(104,209,242);
    painter.setPen(fg);
    QFont font=painter.font();font.setPointSize(12);font.setBold(true);
    painter.setFont(font);
    painter.drawText(QRect(18,15,width()-36,28),Qt::AlignLeft|Qt::AlignVCenter,
                     QStringLiteral("6DoF  /  TOUCH SPACE"));
    font.setPointSize(9);font.setBold(false);painter.setFont(font);
    painter.setPen(muted);
    painter.drawText(QRect(18,42,width()-36,22),Qt::AlignLeft,
                     QStringLiteral("RAW FRAME  ·  POSITION + ORIENTATION"));
    const auto p=m_origin.render(m_state.positionMm);
    painter.setPen(fg);
    const auto raw=m_state.positionMm;
    painter.drawText(QRect(18,height()-119,width()-36,21),Qt::AlignLeft,
        QStringLiteral("RAW     X %1    Y %2    Z %3 mm")
            .arg(raw[0],0,'f',2).arg(raw[1],0,'f',2).arg(raw[2],0,'f',2));
    painter.setPen(muted);
    painter.drawText(QRect(18,height()-97,width()-36,20),Qt::AlignLeft,
        QStringLiteral("VIEW    X %1    Y %2    Z %3 mm")
            .arg(p[0],0,'f',2).arg(p[1],0,'f',2).arg(p[2],0,'f',2));
    if(m_state.orientationValid) {
        touchpanel::teleop::Vec3 angles{};
        const bool hasAngles=touchpanel::teleop::matrixToRpy(
            m_state.rotationMatrix,{0,0,0},angles);
        if(hasAngles) {
            painter.setPen(fg);
            painter.drawText(QRect(18,height()-75,width()-36,20),Qt::AlignLeft,
                QStringLiteral("RPY*   %1°   %2°   %3°")
                    .arg(angles[0],0,'f',1).arg(angles[1],0,'f',1)
                    .arg(angles[2],0,'f',1));
        } else {
            painter.setPen(muted);
            painter.drawText(QRect(18,height()-75,width()-36,20),Qt::AlignLeft,
                QStringLiteral("RPY* unavailable near Euler singularity"));
        }
    }
    painter.setPen(muted);
    painter.drawText(QRect(18,height()-53,width()-36,20),Qt::AlignLeft,
        QStringLiteral("RPY* ZYX view estimate, not verified robot Rx/Ry/Rz"));
    painter.drawText(QRect(18,height()-30,width()-36,20),Qt::AlignLeft,
        m_state.valid ? QStringLiteral("● 6DOF LIVE    ·    C clear trail    R reset") :
                        QStringLiteral("○ NO VALID FRAME"));
    painter.setPen(muted);
    painter.drawText(QRect(18,68,width()-36,18),Qt::AlignLeft,
        QStringLiteral("DRAG orbit  ·  SHIFT+DRAG pan  ·  WHEEL zoom  ·  DOUBLE CLICK focus"));
}
void GLCoordinateWidget::mousePressEvent(QMouseEvent* e) {
    m_lastMousePos=e->position().toPoint();setFocus();e->accept();
}
void GLCoordinateWidget::mouseMoveEvent(QMouseEvent* e) {
    QPoint current=e->position().toPoint(),delta=current-m_lastMousePos;
    m_lastMousePos=current;
    if(e->buttons()&Qt::LeftButton) {
        if(e->modifiers()&Qt::ShiftModifier) {
            const float d=m_distance/500.0f;
            const float a=m_yaw*pi/180;
            m_cameraTarget+=QVector3D(std::sin(a),0,-std::cos(a))*float(delta.x())*d;
            m_cameraTarget+=QVector3D(0,1,0)*float(delta.y())*d;
            m_follow=false;
        } else {
            m_yaw+=delta.x()*0.6f;
            m_pitch=std::clamp(m_pitch+delta.y()*0.5f,-85.0f,85.0f);
        }
        update();
    }
    e->accept();
}
void GLCoordinateWidget::wheelEvent(QWheelEvent* e) {
    m_distance=std::clamp(m_distance*std::pow(0.87f,e->angleDelta().y()/120.0f),55.0f,1300.0f);
    update();e->accept();
}
void GLCoordinateWidget::mouseDoubleClickEvent(QMouseEvent* e) {
    focusOnPen();e->accept();
}
void GLCoordinateWidget::keyPressEvent(QKeyEvent* e) {
    if(e->key()==Qt::Key_R)resetCamera();
    else if(e->key()==Qt::Key_F)focusOnPen();
    else if(e->key()==Qt::Key_C)clearTrail();
    else QOpenGLWidget::keyPressEvent(e);
}
void GLCoordinateWidget::releaseResources() {
    m_gridVbo.destroy();m_gridVao.destroy();
    m_axesVbo.destroy();m_axesVao.destroy();
    m_trailVbo.destroy();m_trailVao.destroy();
    m_penVbo.destroy();m_penVao.destroy();
    m_solidVbo.destroy();m_solidVao.destroy();
    m_shader.reset();
}
