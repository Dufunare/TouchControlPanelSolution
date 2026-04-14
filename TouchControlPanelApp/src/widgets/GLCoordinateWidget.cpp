#include "GLCoordinateWidget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <QDebug>
#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QSizePolicy>
#include <QString>
#include <QVector3D>
#include <QWheelEvent>

namespace
{
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

    constexpr char kVertexShaderSource[] = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;

        uniform mat4 uMvp;
        uniform float uPointSize;

        out vec3 vColor;

        void main()
        {
            gl_Position = uMvp * vec4(aPos, 1.0);
            vColor = aColor;
            gl_PointSize = uPointSize;
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

    enum class GridPlane
    {
        XY,
        XZ,
        YZ
    };

    enum class AxisTag
    {
        X,
        Y,
        Z
    };

    void pushLine(std::vector<GLCoordinateWidget::Vertex>& vertices,
        const QVector3D& a,
        const QVector3D& b,
        const QVector3D& color)
    {
        vertices.push_back({ a.x(), a.y(), a.z(), color.x(), color.y(), color.z() });
        vertices.push_back({ b.x(), b.y(), b.z(), color.x(), color.y(), color.z() });
    }

    std::vector<GLCoordinateWidget::Vertex> buildGridPlaneVertices(GridPlane plane, float halfRange, float step)
    {
        std::vector<GLCoordinateWidget::Vertex> vertices;
        const QVector3D gridColor(0.28f, 0.30f, 0.33f);

        for (float t = -halfRange; t <= halfRange + 0.01f; t += step)
        {
            switch (plane)
            {
            case GridPlane::XY:
                pushLine(vertices, QVector3D(t, -halfRange, 0.0f), QVector3D(t, halfRange, 0.0f), gridColor);
                pushLine(vertices, QVector3D(-halfRange, t, 0.0f), QVector3D(halfRange, t, 0.0f), gridColor);
                break;
            case GridPlane::XZ:
                pushLine(vertices, QVector3D(t, 0.0f, -halfRange), QVector3D(t, 0.0f, halfRange), gridColor);
                pushLine(vertices, QVector3D(-halfRange, 0.0f, t), QVector3D(halfRange, 0.0f, t), gridColor);
                break;
            case GridPlane::YZ:
                pushLine(vertices, QVector3D(0.0f, t, -halfRange), QVector3D(0.0f, t, halfRange), gridColor);
                pushLine(vertices, QVector3D(0.0f, -halfRange, t), QVector3D(0.0f, halfRange, t), gridColor);
                break;
            }
        }

        return vertices;
    }

    std::vector<GLCoordinateWidget::Vertex> buildBoxVertices(float halfRange)
    {
        std::vector<GLCoordinateWidget::Vertex> vertices;
        const QVector3D boxColor(0.72f, 0.76f, 0.80f);

        const QVector3D p000(-halfRange, -halfRange, -halfRange);
        const QVector3D p001(-halfRange, -halfRange, halfRange);
        const QVector3D p010(-halfRange, halfRange, -halfRange);
        const QVector3D p011(-halfRange, halfRange, halfRange);
        const QVector3D p100(halfRange, -halfRange, -halfRange);
        const QVector3D p101(halfRange, -halfRange, halfRange);
        const QVector3D p110(halfRange, halfRange, -halfRange);
        const QVector3D p111(halfRange, halfRange, halfRange);

        // Bottom
        pushLine(vertices, p000, p001, boxColor);
        pushLine(vertices, p001, p101, boxColor);
        pushLine(vertices, p101, p100, boxColor);
        pushLine(vertices, p100, p000, boxColor);

        // Top
        pushLine(vertices, p010, p011, boxColor);
        pushLine(vertices, p011, p111, boxColor);
        pushLine(vertices, p111, p110, boxColor);
        pushLine(vertices, p110, p010, boxColor);

        // Vertical edges
        pushLine(vertices, p000, p010, boxColor);
        pushLine(vertices, p001, p011, boxColor);
        pushLine(vertices, p100, p110, boxColor);
        pushLine(vertices, p101, p111, boxColor);

        return vertices;
    }

    std::vector<GLCoordinateWidget::Vertex> buildAxisVertices(float axisLength)
    {
        std::vector<GLCoordinateWidget::Vertex> vertices;

        const QVector3D white(0.93f, 0.95f, 0.97f);

        const QVector3D red(1.0f, 0.25f, 0.25f);
        const QVector3D green(0.25f, 1.0f, 0.25f);
        const QVector3D blue(0.3f, 0.7f, 1.0f);

        const float highlightLength = 24.0f;

        // =========================
        // X轴
        // =========================

        // 负方向（白）
        pushLine(vertices,
            QVector3D(-axisLength, 0, 0),
            QVector3D(0, 0, 0),
            white);

        // 正方向高亮段（红）
        pushLine(vertices,
            QVector3D(0, 0, 0),
            QVector3D(highlightLength, 0, 0),
            red);

        // 正方向剩余段（白）
        pushLine(vertices,
            QVector3D(highlightLength, 0, 0),
            QVector3D(axisLength, 0, 0),
            white);


        // =========================
        // Y轴
        // =========================

        pushLine(vertices,
            QVector3D(0, -axisLength, 0),
            QVector3D(0, 0, 0),
            white);

        pushLine(vertices,
            QVector3D(0, 0, 0),
            QVector3D(0, highlightLength, 0),
            green);

        pushLine(vertices,
            QVector3D(0, highlightLength, 0),
            QVector3D(0, axisLength, 0),
            white);


        // =========================
        // Z轴
        // =========================

        pushLine(vertices,
            QVector3D(0, 0, -axisLength),
            QVector3D(0, 0, 0),
            white);

        pushLine(vertices,
            QVector3D(0, 0, 0),
            QVector3D(0, 0, highlightLength),
            blue);

        pushLine(vertices,
            QVector3D(0, 0, highlightLength),
            QVector3D(0, 0, axisLength),
            white);

        return vertices;
    }

    std::vector<GLCoordinateWidget::Vertex> buildArrowVertices(AxisTag axis,
        float length,
        float headLength,
        float headHalfWidth)
    {
        std::vector<GLCoordinateWidget::Vertex> vertices;

        QVector3D color;
        switch (axis)
        {
        case AxisTag::X: color = QVector3D(1.0f, 0.20f, 0.20f); break;
        case AxisTag::Y: color = QVector3D(0.20f, 1.0f, 0.20f); break;
        case AxisTag::Z: color = QVector3D(0.25f, 0.72f, 1.0f); break;
        }

        auto pushTriangle =
            [&vertices, &color](const QVector3D& a, const QVector3D& b, const QVector3D& c)
            {
                vertices.push_back({ a.x(), a.y(), a.z(), color.x(), color.y(), color.z() });
                vertices.push_back({ b.x(), b.y(), b.z(), color.x(), color.y(), color.z() });
                vertices.push_back({ c.x(), c.y(), c.z(), color.x(), color.y(), color.z() });
            };

        QVector3D tip;
        QVector3D baseCenter;
        QVector3D p1, p2, p3, p4;

        switch (axis)
        {
        case AxisTag::X:
            tip = QVector3D(length, 0.0f, 0.0f);
            baseCenter = QVector3D(length - headLength, 0.0f, 0.0f);

            p1 = baseCenter + QVector3D(0.0f, headHalfWidth, 0.0f);
            p2 = baseCenter + QVector3D(0.0f, -headHalfWidth, 0.0f);
            p3 = baseCenter + QVector3D(0.0f, 0.0f, headHalfWidth);
            p4 = baseCenter + QVector3D(0.0f, 0.0f, -headHalfWidth);
            break;

        case AxisTag::Y:
            tip = QVector3D(0.0f, length, 0.0f);
            baseCenter = QVector3D(0.0f, length - headLength, 0.0f);

            p1 = baseCenter + QVector3D(headHalfWidth, 0.0f, 0.0f);
            p2 = baseCenter + QVector3D(-headHalfWidth, 0.0f, 0.0f);
            p3 = baseCenter + QVector3D(0.0f, 0.0f, headHalfWidth);
            p4 = baseCenter + QVector3D(0.0f, 0.0f, -headHalfWidth);
            break;

        case AxisTag::Z:
            tip = QVector3D(0.0f, 0.0f, length);
            baseCenter = QVector3D(0.0f, 0.0f, length - headLength);

            p1 = baseCenter + QVector3D(headHalfWidth, 0.0f, 0.0f);
            p2 = baseCenter + QVector3D(-headHalfWidth, 0.0f, 0.0f);
            p3 = baseCenter + QVector3D(0.0f, headHalfWidth, 0.0f);
            p4 = baseCenter + QVector3D(0.0f, -headHalfWidth, 0.0f);
            break;
        }

        // 四个侧面三角形
        pushTriangle(tip, p1, p3);
        pushTriangle(tip, p3, p2);
        pushTriangle(tip, p2, p4);
        pushTriangle(tip, p4, p1);

        // 底面做成两个三角形，形成一个菱形底
        pushTriangle(p1, p3, p2);
        pushTriangle(p1, p2, p4);

        return vertices;
    }

    std::array<GLCoordinateWidget::Vertex, 1> buildOriginVertex()
    {
        return { GLCoordinateWidget::Vertex{ 0.0f, 0.0f, 0.0f, 1.0f, 0.95f, 0.55f } };
    }

    std::vector<GLCoordinateWidget::Vertex> buildPenVertices(const QVector3D& tip, bool valid)
    {
        std::vector<GLCoordinateWidget::Vertex> vertices;

        const QVector3D color = valid
            ? QVector3D(1.0f, 0.88f, 0.35f)
            : QVector3D(0.70f, 0.70f, 0.70f);

        const QVector3D dir = QVector3D(1.0f, 0.20f, -0.10f).normalized();

        QVector3D side = QVector3D::crossProduct(dir, QVector3D(0.0f, 1.0f, 0.0f));
        if (side.lengthSquared() < 1e-6f)
        {
            side = QVector3D(0.0f, 0.0f, 1.0f);
        }
        side.normalize();

        QVector3D up = QVector3D::crossProduct(side, dir).normalized();

        const float penLength = 18.0f;
        const float penHalfWidth = 2.4f;
        const float nibBackOffset = 5.0f;

        const QVector3D tailCenter = tip - dir * penLength;
        const QVector3D bodyFrontCenter = tip - dir * nibBackOffset;

        const QVector3D tailA = tailCenter + up * penHalfWidth + side * (penHalfWidth * 0.55f);
        const QVector3D tailB = tailCenter - up * penHalfWidth + side * (penHalfWidth * 0.55f);
        const QVector3D tailC = tailCenter - up * penHalfWidth - side * (penHalfWidth * 0.55f);
        const QVector3D tailD = tailCenter + up * penHalfWidth - side * (penHalfWidth * 0.55f);

        const QVector3D nibUp = bodyFrontCenter + up * (penHalfWidth * 0.55f);
        const QVector3D nibDown = bodyFrontCenter - up * (penHalfWidth * 0.55f);
        const QVector3D nibLeft = bodyFrontCenter + side * (penHalfWidth * 0.35f);
        const QVector3D nibRight = bodyFrontCenter - side * (penHalfWidth * 0.35f);

        pushLine(vertices, tailA, tailB, color);
        pushLine(vertices, tailB, tailC, color);
        pushLine(vertices, tailC, tailD, color);
        pushLine(vertices, tailD, tailA, color);

        pushLine(vertices, tailA, bodyFrontCenter, color);
        pushLine(vertices, tailB, bodyFrontCenter, color);
        pushLine(vertices, tailC, bodyFrontCenter, color);
        pushLine(vertices, tailD, bodyFrontCenter, color);

        pushLine(vertices, tailCenter, bodyFrontCenter, color);

        pushLine(vertices, nibUp, tip, color);
        pushLine(vertices, nibDown, tip, color);
        pushLine(vertices, nibLeft, tip, color);
        pushLine(vertices, nibRight, tip, color);

        pushLine(vertices, nibUp, nibDown, color);

        return vertices;
    }
} // namespace

GLCoordinateWidget::GLCoordinateWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(320, 220);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
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
    glClearColor(0.07f, 0.09f, 0.11f, 1.0f);

    initializeProgram();
    //initializeGridPlanes(); 减少初始化开销
    initializeBox();
    initializeAxis();
    initializeArrows();
    initializeOrigin();
    initializePen();
}

void GLCoordinateWidget::resizeGL(int w, int h)
{
    const float aspect = (h == 0) ? 1.0f : static_cast<float>(w) / static_cast<float>(h);

    m_projection.setToIdentity();
    m_projection.perspective(45.0f, aspect, 1.0f, 4000.0f);
}

void GLCoordinateWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_program)
    {
        return;
    }

    QMatrix4x4 model;
    model.scale(m_sceneScale);

    const QMatrix4x4 mvp = m_projection * buildViewMatrix() * model;

    m_program->bind();
    m_program->setUniformValue("uMvp", mvp);
    m_program->setUniformValue("uPointSize", 1.0f);

    glLineWidth(1.0f);

    /*m_gridXyVao.bind();
    glDrawArrays(GL_LINES, 0, m_gridXyVertexCount);
    m_gridXyVao.release();

    m_gridXzVao.bind();
    glDrawArrays(GL_LINES, 0, m_gridXzVertexCount);
    m_gridXzVao.release();

    m_gridYzVao.bind();
    glDrawArrays(GL_LINES, 0, m_gridYzVertexCount);
    m_gridYzVao.release();
    */ //用来画网格线的，现在暂时注释掉
    glLineWidth(1.5f);
    m_boxVao.bind();
    glDrawArrays(GL_LINES, 0, m_boxVertexCount);
    m_boxVao.release();

    // 坐标轴分三段画：白色负轴 -> RGB高亮短段 -> 白色正轴剩余
    m_axisVao.bind();

    // 白（负轴）
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, 6);

    // RGB短段
    glLineWidth(5.0f);
    glDrawArrays(GL_LINES, 6, 6);

    // 白（正轴剩余）
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 12, 6);

    m_axisVao.release();

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    m_arrowXVao.bind();
    glDrawArrays(GL_TRIANGLES, 0, m_arrowXVertexCount);
    m_arrowXVao.release();

    m_arrowYVao.bind();
    glDrawArrays(GL_TRIANGLES, 0, m_arrowYVertexCount);
    m_arrowYVao.release();

    m_arrowZVao.bind();
    glDrawArrays(GL_TRIANGLES, 0, m_arrowZVertexCount);
    m_arrowZVao.release();

    glDisable(GL_CULL_FACE);

    m_program->setUniformValue("uPointSize", m_originPointSize * std::max(0.55f, m_sceneScale));
    m_originVao.bind();
    glDrawArrays(GL_POINTS, 0, m_originVertexCount);
    m_originVao.release();
    m_program->setUniformValue("uPointSize", 1.0f);

    updatePenGeometry();
    glLineWidth(3.0f);
    m_penVao.bind();
    glDrawArrays(GL_LINES, 0, m_penVertexCount);
    m_penVao.release();

    m_program->release();

    QPainter painter(this);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QFont font("Consolas");
    font.setPointSize(11);
    painter.setFont(font);

    const QString text = QString("X: %1   Y: %2   Z: %3 mm")
        .arg(m_state.positionMm[0], 0, 'f', 3)
        .arg(m_state.positionMm[1], 0, 'f', 3)
        .arg(m_state.positionMm[2], 0, 'f', 3);

    const QRect textRect(14, 12, width() - 28, 28);
    const QColor textColor = m_state.valid ? QColor(230, 240, 255) : QColor(170, 170, 170);

    painter.setPen(QColor(0, 0, 0, 180));
    painter.drawText(textRect.translated(1, 1), Qt::AlignLeft | Qt::AlignVCenter, text);

    painter.setPen(textColor);
    painter.drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, text);
}

void GLCoordinateWidget::mousePressEvent(QMouseEvent* event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    m_lastMousePos = event->position().toPoint();
#else
    m_lastMousePos = event->pos();
#endif
    event->accept();
}

void GLCoordinateWidget::mouseMoveEvent(QMouseEvent* event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QPoint currentPos = event->position().toPoint();
#else
    const QPoint currentPos = event->pos();
#endif

    const QPoint delta = currentPos - m_lastMousePos;
    m_lastMousePos = currentPos;

    if (event->buttons() & Qt::LeftButton)
    {
        m_yawDeg += static_cast<float>(delta.x()) * 0.6f;
        m_pitchDeg += static_cast<float>(delta.y()) * 0.5f;
        m_pitchDeg = std::clamp(m_pitchDeg, -89.0f, 89.0f);
        update();
    }

    event->accept();
}

void GLCoordinateWidget::wheelEvent(QWheelEvent* event)
{
    const int delta = event->angleDelta().y();
    if (delta != 0)
    {
        const float factor = std::pow(1.10f, static_cast<float>(delta) / 120.0f);
        m_sceneScale = std::clamp(m_sceneScale * factor, 0.25f, 4.0f);
        update();
    }

    event->accept();
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

void GLCoordinateWidget::initializeGridPlanes()
{
    const auto xyVertices = buildGridPlaneVertices(GridPlane::XY, m_worldHalfRangeMm, m_gridStepMm);
    m_gridXyVertexCount = static_cast<int>(xyVertices.size());
    initializeStaticMesh(m_gridXyVao,
        m_gridXyVbo,
        xyVertices.data(),
        static_cast<int>(xyVertices.size() * sizeof(Vertex)),
        m_gridXyVertexCount,
        QOpenGLBuffer::StaticDraw);

    const auto xzVertices = buildGridPlaneVertices(GridPlane::XZ, m_worldHalfRangeMm, m_gridStepMm);
    m_gridXzVertexCount = static_cast<int>(xzVertices.size());
    initializeStaticMesh(m_gridXzVao,
        m_gridXzVbo,
        xzVertices.data(),
        static_cast<int>(xzVertices.size() * sizeof(Vertex)),
        m_gridXzVertexCount,
        QOpenGLBuffer::StaticDraw);

    const auto yzVertices = buildGridPlaneVertices(GridPlane::YZ, m_worldHalfRangeMm, m_gridStepMm);
    m_gridYzVertexCount = static_cast<int>(yzVertices.size());
    initializeStaticMesh(m_gridYzVao,
        m_gridYzVbo,
        yzVertices.data(),
        static_cast<int>(yzVertices.size() * sizeof(Vertex)),
        m_gridYzVertexCount,
        QOpenGLBuffer::StaticDraw);
}

void GLCoordinateWidget::initializeBox()
{
    const auto boxVertices = buildBoxVertices(m_worldHalfRangeMm);
    m_boxVertexCount = static_cast<int>(boxVertices.size());
    initializeStaticMesh(m_boxVao,
        m_boxVbo,
        boxVertices.data(),
        static_cast<int>(boxVertices.size() * sizeof(Vertex)),
        m_boxVertexCount,
        QOpenGLBuffer::StaticDraw);
}

void GLCoordinateWidget::initializeAxis()
{
    const auto axisVertices = buildAxisVertices(m_axisLengthMm);
    m_axisVertexCount = static_cast<int>(axisVertices.size());
    initializeStaticMesh(m_axisVao,
        m_axisVbo,
        axisVertices.data(),
        static_cast<int>(axisVertices.size() * sizeof(Vertex)),
        m_axisVertexCount,
        QOpenGLBuffer::StaticDraw);
}

void GLCoordinateWidget::initializeArrows()
{
    const auto arrowXVertices = buildArrowVertices(AxisTag::X,
        m_arrowLengthMm,
        m_arrowHeadLengthMm,
        m_arrowHeadHalfWidthMm);
    m_arrowXVertexCount = static_cast<int>(arrowXVertices.size());
    initializeStaticMesh(m_arrowXVao,
        m_arrowXVbo,
        arrowXVertices.data(),
        static_cast<int>(arrowXVertices.size() * sizeof(Vertex)),
        m_arrowXVertexCount,
        QOpenGLBuffer::StaticDraw);

    const auto arrowYVertices = buildArrowVertices(AxisTag::Y,
        m_arrowLengthMm,
        m_arrowHeadLengthMm,
        m_arrowHeadHalfWidthMm);
    m_arrowYVertexCount = static_cast<int>(arrowYVertices.size());
    initializeStaticMesh(m_arrowYVao,
        m_arrowYVbo,
        arrowYVertices.data(),
        static_cast<int>(arrowYVertices.size() * sizeof(Vertex)),
        m_arrowYVertexCount,
        QOpenGLBuffer::StaticDraw);

    const auto arrowZVertices = buildArrowVertices(AxisTag::Z,
        m_arrowLengthMm,
        m_arrowHeadLengthMm,
        m_arrowHeadHalfWidthMm);
    m_arrowZVertexCount = static_cast<int>(arrowZVertices.size());
    initializeStaticMesh(m_arrowZVao,
        m_arrowZVbo,
        arrowZVertices.data(),
        static_cast<int>(arrowZVertices.size() * sizeof(Vertex)),
        m_arrowZVertexCount,
        QOpenGLBuffer::StaticDraw);
}

void GLCoordinateWidget::initializeOrigin()
{
    const auto originVertex = buildOriginVertex();
    m_originVertexCount = static_cast<int>(originVertex.size());
    initializeStaticMesh(m_originVao,
        m_originVbo,
        originVertex.data(),
        static_cast<int>(originVertex.size() * sizeof(Vertex)),
        m_originVertexCount,
        QOpenGLBuffer::StaticDraw);
}

void GLCoordinateWidget::initializePen()
{
    const auto penVertices = buildPenVertices(QVector3D(0.0f, 0.0f, 0.0f), false);
    m_penVertexCount = static_cast<int>(penVertices.size());

    m_penVao.create();
    m_penVao.bind();

    m_penVbo.create();
    m_penVbo.bind();
    m_penVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_penVbo.allocate(penVertices.data(), static_cast<int>(penVertices.size() * sizeof(Vertex)));

    m_program->bind();
    setupVertexLayout();
    m_program->release();

    m_penVbo.release();
    m_penVao.release();
}

void GLCoordinateWidget::initializeStaticMesh(QOpenGLVertexArrayObject& vao,
    QOpenGLBuffer& vbo,
    const Vertex* data,
    int byteCount,
    int vertexCount,
    QOpenGLBuffer::UsagePattern usagePattern)
{
    Q_UNUSED(vertexCount);

    vao.create();
    vao.bind();

    vbo.create();
    vbo.bind();
    vbo.setUsagePattern(usagePattern);
    vbo.allocate(data, byteCount);

    m_program->bind();
    setupVertexLayout();
    m_program->release();

    vbo.release();
    vao.release();
}

void GLCoordinateWidget::setupVertexLayout()
{
    m_program->enableAttributeArray(0);
    m_program->enableAttributeArray(1);

    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));
}

void GLCoordinateWidget::updatePenGeometry()
{
    const QVector3D tip(
        static_cast<float>(m_state.positionMm[0]),
        static_cast<float>(m_state.positionMm[1]),
        static_cast<float>(m_state.positionMm[2]));

    const auto penVertices = buildPenVertices(tip, m_state.valid);

    m_penVertexCount = static_cast<int>(penVertices.size());
    m_penVbo.bind();
    m_penVbo.write(0, penVertices.data(), static_cast<int>(penVertices.size() * sizeof(Vertex)));
    m_penVbo.release();
}

QMatrix4x4 GLCoordinateWidget::buildViewMatrix() const
{
    const float yawRad = m_yawDeg * kDegToRad;
    const float pitchRad = m_pitchDeg * kDegToRad;

    const float cosPitch = std::cos(pitchRad);
    const float sinPitch = std::sin(pitchRad);
    const float cosYaw = std::cos(yawRad);
    const float sinYaw = std::sin(yawRad);

    const QVector3D eye(
        m_cameraDistanceMm * cosPitch * cosYaw,
        m_cameraDistanceMm * sinPitch,
        m_cameraDistanceMm * cosPitch * sinYaw);

    QMatrix4x4 view;
    view.lookAt(eye, QVector3D(0.0f, 0.0f, 0.0f), QVector3D(0.0f, 1.0f, 0.0f));
    return view;
}

void GLCoordinateWidget::destroyGlResources()
{
    m_gridXyVbo.destroy();
    m_gridXyVao.destroy();

    m_gridXzVbo.destroy();
    m_gridXzVao.destroy();

    m_gridYzVbo.destroy();
    m_gridYzVao.destroy();

    m_boxVbo.destroy();
    m_boxVao.destroy();

    m_axisVbo.destroy();
    m_axisVao.destroy();

    m_arrowXVbo.destroy();
    m_arrowXVao.destroy();

    m_arrowYVbo.destroy();
    m_arrowYVao.destroy();

    m_arrowZVbo.destroy();
    m_arrowZVao.destroy();

    m_originVbo.destroy();
    m_originVao.destroy();

    m_penVbo.destroy();
    m_penVao.destroy();

    m_program.reset();
}