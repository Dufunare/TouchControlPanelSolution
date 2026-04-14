#include "GLCoordinateWidget.h"

#include <array>
#include <cstddef>
#include <vector>

#include <QDebug>
#include <QVector3D>

namespace
{
    constexpr char kVertexShaderSource[] = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;

        uniform mat4 uMvp;

        out vec3 vColor;

        void main()
        {
            gl_Position = uMvp * vec4(aPos, 1.0);
            vColor = aColor;
            gl_PointSize = 12.0;
        }
    )";

    constexpr char kFragmentShaderSource[] = R"(
        #version 330 core
        in vec3 vColor;
        out vec4 FragColor;

        void main()
        {
            FragColor = vec4(vColor, 1.0);
        }
    )";

    std::vector<GLCoordinateWidget::Vertex> buildGridVertices(float halfRange, float step)
    {
        std::vector<GLCoordinateWidget::Vertex> vertices;
        const float c = 0.30f;

        for (float x = -halfRange; x <= halfRange + 0.1f; x += step)
        {
            vertices.push_back({ x, -halfRange, 0.0f, c, c, c });
            vertices.push_back({ x,  halfRange, 0.0f, c, c, c });
        }

        for (float y = -halfRange; y <= halfRange + 0.1f; y += step)
        {
            vertices.push_back({ -halfRange, y, 0.0f, c, c, c });
            vertices.push_back({ halfRange, y, 0.0f, c, c, c });
        }

        return vertices;
    }

    std::array<GLCoordinateWidget::Vertex, 6> buildAxisVertices(float axisLength)
    {
        return {
            GLCoordinateWidget::Vertex{-axisLength, 0.0f,       0.0f,       0.8f, 0.2f, 0.2f},
            GLCoordinateWidget::Vertex{ axisLength, 0.0f,       0.0f,       1.0f, 0.2f, 0.2f},
            GLCoordinateWidget::Vertex{ 0.0f,      -axisLength, 0.0f,       0.2f, 0.8f, 0.2f},
            GLCoordinateWidget::Vertex{ 0.0f,       axisLength, 0.0f,       0.2f, 1.0f, 0.2f},
            GLCoordinateWidget::Vertex{ 0.0f,       0.0f,      -axisLength, 0.2f, 0.5f, 0.9f},
            GLCoordinateWidget::Vertex{ 0.0f,       0.0f,       axisLength, 0.2f, 0.7f, 1.0f}
        };
    }
} // namespace

GLCoordinateWidget::GLCoordinateWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(640, 480);
}

GLCoordinateWidget::~GLCoordinateWidget()
{
    if (context() != nullptr)
    {
        makeCurrent();
        destroyGlResources();
        doneCurrent();
    }
}

void GLCoordinateWidget::setDeviceState(const touchpanel::DeviceState& state)
{
    m_state = state;
    update();
}

void GLCoordinateWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glClearColor(0.08f, 0.10f, 0.12f, 1.0f);

    initializeProgram();
    initializeGrid();
    initializeAxis();
    initializePoint();
}

void GLCoordinateWidget::resizeGL(int w, int h)
{
    const float aspect = (h == 0) ? 1.0f : static_cast<float>(w) / static_cast<float>(h);

    m_projection.setToIdentity();
    m_projection.perspective(45.0f, aspect, 1.0f, 1500.0f);
}

void GLCoordinateWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_program)
    {
        return;
    }

    QMatrix4x4 view;
    view.lookAt(QVector3D(260.0f, 220.0f, 220.0f),
        QVector3D(0.0f, 0.0f, 0.0f),
        QVector3D(0.0f, 1.0f, 0.0f));

    const QMatrix4x4 mvp = m_projection * view;

    m_program->bind();
    m_program->setUniformValue("uMvp", mvp);

    m_gridVao.bind();
    glDrawArrays(GL_LINES, 0, m_gridVertexCount);
    m_gridVao.release();

    m_axisVao.bind();
    glDrawArrays(GL_LINES, 0, m_axisVertexCount);
    m_axisVao.release();

    Vertex pointVertex{};
    pointVertex.x = static_cast<float>(m_state.positionMm[0]);
    pointVertex.y = static_cast<float>(m_state.positionMm[1]);
    pointVertex.z = static_cast<float>(m_state.positionMm[2]);

    if (m_state.valid)
    {
        pointVertex.r = 1.0f;
        pointVertex.g = 0.9f;
        pointVertex.b = 0.2f;
    }
    else
    {
        pointVertex.r = 0.7f;
        pointVertex.g = 0.7f;
        pointVertex.b = 0.7f;
    }

    m_pointVao.bind();
    m_pointVbo.bind();
    m_pointVbo.write(0, &pointVertex, sizeof(Vertex));
    glDrawArrays(GL_POINTS, 0, 1);
    m_pointVbo.release();
    m_pointVao.release();

    m_program->release();
}

void GLCoordinateWidget::initializeProgram()
{
    m_program = std::make_unique<QOpenGLShaderProgram>();

    if (!m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShaderSource))
    {
        qFatal("Vertex shader compile failed: %s", qPrintable(m_program->log()));
    }

    if (!m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShaderSource))
    {
        qFatal("Fragment shader compile failed: %s", qPrintable(m_program->log()));
    }

    if (!m_program->link())
    {
        qFatal("OpenGL program link failed: %s", qPrintable(m_program->log()));
    }
}

void GLCoordinateWidget::initializeGrid()
{
    const auto vertices = buildGridVertices(m_worldHalfRangeMm, 20.0f);
    m_gridVertexCount = static_cast<int>(vertices.size());

    m_gridVao.create();
    m_gridVao.bind();

    m_gridVbo.create();
    m_gridVbo.bind();
    m_gridVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_gridVbo.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(Vertex)));

    m_program->bind();
    setupVertexLayout();
    m_program->release();

    m_gridVbo.release();
    m_gridVao.release();
}

void GLCoordinateWidget::initializeAxis()
{
    const auto vertices = buildAxisVertices(m_axisLengthMm);
    m_axisVertexCount = static_cast<int>(vertices.size());

    m_axisVao.create();
    m_axisVao.bind();

    m_axisVbo.create();
    m_axisVbo.bind();
    m_axisVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_axisVbo.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(Vertex)));

    m_program->bind();
    setupVertexLayout();
    m_program->release();

    m_axisVbo.release();
    m_axisVao.release();
}

void GLCoordinateWidget::initializePoint()
{
    const Vertex pointVertex{ 0.0f, 0.0f, 0.0f, 1.0f, 0.9f, 0.2f };

    m_pointVao.create();
    m_pointVao.bind();

    m_pointVbo.create();
    m_pointVbo.bind();
    m_pointVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_pointVbo.allocate(&pointVertex, sizeof(Vertex));

    m_program->bind();
    setupVertexLayout();
    m_program->release();

    m_pointVbo.release();
    m_pointVao.release();
}

void GLCoordinateWidget::setupVertexLayout()
{
    m_program->enableAttributeArray(0);
    m_program->enableAttributeArray(1);

    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));
}

void GLCoordinateWidget::destroyGlResources()
{
    m_gridVbo.destroy();
    m_gridVao.destroy();

    m_axisVbo.destroy();
    m_axisVao.destroy();

    m_pointVbo.destroy();
    m_pointVao.destroy();

    m_program.reset();
}
