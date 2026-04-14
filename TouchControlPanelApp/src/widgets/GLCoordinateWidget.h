#pragma once

#include <memory>

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>

#include "DeviceState.h"

class GLCoordinateWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    explicit GLCoordinateWidget(QWidget* parent = nullptr);
    ~GLCoordinateWidget() override;

    void setDeviceState(const touchpanel::DeviceState& state);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

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

    void initializeProgram();
    void initializeGrid();
    void initializeAxis();
    void initializePoint();
    void setupVertexLayout();
    void destroyGlResources();

private:
    touchpanel::DeviceState m_state;

    QMatrix4x4 m_projection;

    std::unique_ptr<QOpenGLShaderProgram> m_program;

    QOpenGLVertexArrayObject m_gridVao;
    QOpenGLBuffer m_gridVbo{ QOpenGLBuffer::VertexBuffer };
    int m_gridVertexCount = 0;

    QOpenGLVertexArrayObject m_axisVao;
    QOpenGLBuffer m_axisVbo{ QOpenGLBuffer::VertexBuffer };
    int m_axisVertexCount = 0;

    QOpenGLVertexArrayObject m_pointVao;
    QOpenGLBuffer m_pointVbo{ QOpenGLBuffer::VertexBuffer };

    float m_worldHalfRangeMm = 120.0f;
    float m_axisLengthMm = 120.0f;
};
