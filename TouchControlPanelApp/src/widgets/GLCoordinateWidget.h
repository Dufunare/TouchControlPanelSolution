#pragma once

#include <memory>

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>

#include "DeviceState.h"

class QMouseEvent;
class QWheelEvent;

class GLCoordinateWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    struct Vertex
    {
        float x;
        float y;
        float z;
        float r;
        float g;
        float b;
    };

public:
    explicit GLCoordinateWidget(QWidget* parent = nullptr);
    ~GLCoordinateWidget() override;

    void setDeviceState(const touchpanel::DeviceState& state);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void initializeProgram();

    void initializeGridPlanes();
    void initializeBox();
    void initializeAxis();
    void initializeArrows();
    void initializeOrigin();
    void initializePen();

    void initializeStaticMesh(QOpenGLVertexArrayObject& vao,
        QOpenGLBuffer& vbo,
        const Vertex* data,
        int byteCount,
        int vertexCount,
        QOpenGLBuffer::UsagePattern usagePattern);

    void setupVertexLayout();
    void updatePenGeometry();
    void destroyGlResources();

    QMatrix4x4 buildViewMatrix() const;

private:
    touchpanel::DeviceState m_state{};

    QMatrix4x4 m_projection;
    std::unique_ptr<QOpenGLShaderProgram> m_program;

    QOpenGLVertexArrayObject m_gridXyVao;
    QOpenGLBuffer m_gridXyVbo{ QOpenGLBuffer::VertexBuffer };
    int m_gridXyVertexCount = 0;

    QOpenGLVertexArrayObject m_gridXzVao;
    QOpenGLBuffer m_gridXzVbo{ QOpenGLBuffer::VertexBuffer };
    int m_gridXzVertexCount = 0;

    QOpenGLVertexArrayObject m_gridYzVao;
    QOpenGLBuffer m_gridYzVbo{ QOpenGLBuffer::VertexBuffer };
    int m_gridYzVertexCount = 0;

    QOpenGLVertexArrayObject m_boxVao;
    QOpenGLBuffer m_boxVbo{ QOpenGLBuffer::VertexBuffer };
    int m_boxVertexCount = 0;

    QOpenGLVertexArrayObject m_axisVao;
    QOpenGLBuffer m_axisVbo{ QOpenGLBuffer::VertexBuffer };
    int m_axisVertexCount = 0;

    QOpenGLVertexArrayObject m_arrowXVao;
    QOpenGLBuffer m_arrowXVbo{ QOpenGLBuffer::VertexBuffer };
    int m_arrowXVertexCount = 0;

    QOpenGLVertexArrayObject m_arrowYVao;
    QOpenGLBuffer m_arrowYVbo{ QOpenGLBuffer::VertexBuffer };
    int m_arrowYVertexCount = 0;

    QOpenGLVertexArrayObject m_arrowZVao;
    QOpenGLBuffer m_arrowZVbo{ QOpenGLBuffer::VertexBuffer };
    int m_arrowZVertexCount = 0;

    QOpenGLVertexArrayObject m_originVao;
    QOpenGLBuffer m_originVbo{ QOpenGLBuffer::VertexBuffer };
    int m_originVertexCount = 0;

    QOpenGLVertexArrayObject m_penVao;
    QOpenGLBuffer m_penVbo{ QOpenGLBuffer::VertexBuffer };
    int m_penVertexCount = 0;

    QPoint m_lastMousePos;

    float m_sceneScale = 1.0f;
    float m_yawDeg = 38.0f;
    float m_pitchDeg = 24.0f;
    float m_cameraDistanceMm = 420.0f;

    float m_worldHalfRangeMm = 120.0f;
    float m_gridStepMm = 20.0f;
    float m_axisLengthMm = 140.0f;

    float m_arrowLengthMm = 24.0f;
    float m_arrowHeadLengthMm = 10.0f;
    float m_arrowHeadHalfWidthMm = 3.5f;

    float m_originPointSize = 16.0f;
};