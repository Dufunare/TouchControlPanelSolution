#pragma once
#include <deque>
#include <memory>
#include <vector>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>
#include "DeviceState.h"
#include "DisplayOrigin.h"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

class GLCoordinateWidget final : public QOpenGLWidget, protected QOpenGLFunctions {
public:
    struct Vertex { float x,y,z,r,g,b; };

    explicit GLCoordinateWidget(QWidget* parent=nullptr);
    ~GLCoordinateWidget() override;

    void setDeviceState(const touchpanel::DeviceState& state);
    bool captureDisplayOrigin();
    void clearDisplayOrigin();
    void resetCamera();
    void focusOnPen();
    void clearTrail();

protected:
    void initializeGL() override;
    void resizeGL(int w,int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void upload(QOpenGLVertexArrayObject& vao,QOpenGLBuffer& vbo,
                const std::vector<Vertex>& vertices);
    void draw(QOpenGLVertexArrayObject& vao,int count,unsigned mode,float width=1.0f);
    void buildStaticScene();
    void updateDynamicGeometry();
    QMatrix4x4 viewMatrix() const;
    QVector3D renderPosition() const;
    void releaseResources();

    touchpanel::DeviceState m_state{};
    touchpanel::visualization::DisplayOrigin m_origin;
    std::deque<QVector3D> m_trail;
    bool m_haveValidPose=false;
    bool m_follow=false;
    QVector3D m_cameraTarget{};
    QPoint m_lastMousePos;
    float m_yaw=43.0f, m_pitch=24.0f, m_distance=430.0f;

    QMatrix4x4 m_projection;
    std::unique_ptr<QOpenGLShaderProgram> m_shader;
    QOpenGLVertexArrayObject m_gridVao,m_axesVao,m_trailVao,m_penVao,m_solidVao;
    QOpenGLBuffer m_gridVbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_axesVbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_trailVbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_penVbo{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_solidVbo{QOpenGLBuffer::VertexBuffer};
    int m_gridCount=0,m_axesCount=0,m_trailCount=0,m_penCount=0,m_solidCount=0;
};
