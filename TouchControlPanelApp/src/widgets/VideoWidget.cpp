#include "VideoWidget.h"

#include <QPainter>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QTimer>

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent)
{
    // A JPEG's native pixel dimensions must NEVER become a layout minimum.
    // Paint within our own rect instead of allowing a pixmap QLabel to grow.
    setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
    setMinimumSize(180,120);
    setAttribute(Qt::WA_OpaquePaintEvent);
    auto* refresh=new QTimer(this);
    refresh->setInterval(40); // GUI displays latest available frame, <=25 FPS.
    connect(refresh,&QTimer::timeout,this,&VideoWidget::updateFrame);
    refresh->start();
}

void VideoWidget::setFrame(const QImage& image)
{
    // No QWidget calls, queued repaint events or deep copies in this path.
    std::lock_guard<std::mutex> lock(m_frameMutex);
    m_latestFrame=image;
    ++m_revision;
}

void VideoWidget::setFrameFromMat(const cv::Mat& frame)
{
    Q_UNUSED(frame);
}

void VideoWidget::showNoSignal()
{
    setFrame(QImage());
}

void VideoWidget::clearFrame()
{
    showNoSignal();
}

bool VideoWidget::hasSignal() const
{
    std::lock_guard<std::mutex> lock(m_frameMutex);
    return !m_latestFrame.isNull();
}

void VideoWidget::updateFrame()
{
    refreshWarning(); // Age and hold timers advance even when video is frozen.
    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        if(m_displayRevision==m_revision)return;
        m_displayFrame=m_latestFrame;
        m_displayRevision=m_revision;
    }
    rebuildDisplayCache();
    update();
}

void VideoWidget::setWarningThreshold(double mm)
{
    m_cameraWarning.setThreshold(mm,CameraWarningState::nowMs());
    refreshWarning();
}

void VideoWidget::setCameraObservation(const QJsonObject& observation,qint64 receivedAtMs)
{
    m_cameraWarning.receive(observation,receivedAtMs,CameraWarningState::nowMs());
    refreshWarning();
}

void VideoWidget::refreshWarning()
{
    const auto now=CameraWarningState::nowMs();
    const auto text=m_cameraWarning.text(now);
    const bool active=m_cameraWarning.warning(now);
    if(text==m_warningText && active==m_warningActive)return;
    m_warningText=text;m_warningActive=active;update();
}

void VideoWidget::paintWarning(QPainter& painter)
{
    painter.save();
    QFont font=painter.font();font.setPixelSize(12);painter.setFont(font);
    const QRect available=rect().adjusted(8,6,-8,-6);
    const QRect bounds=painter.boundingRect(available,Qt::TextWordWrap,m_warningText);
    painter.fillRect(QRect(0,0,width(),qMin(height(),bounds.height()+12)),
        m_warningActive?QColor(145,25,25,235):QColor(20,25,30,220));
    painter.setPen(Qt::white);
    painter.drawText(available,Qt::TextWordWrap,m_warningText);
    painter.restore();
}

void VideoWidget::rebuildDisplayCache()
{
    m_cachedSize=size();
    m_cachedDpr=devicePixelRatioF();
    m_scaledFrame=QPixmap();
    if(m_displayFrame.isNull() || size().isEmpty())return;
    // Like PR #2, scale once per new frame or geometry change. Repaints
    // caused by the OpenGL sibling only blit the already-sized pixmap.
    const QSize deviceSize(qRound(m_cachedSize.width()*m_cachedDpr),
                           qRound(m_cachedSize.height()*m_cachedDpr));
    const QImage image=m_mirrored?m_displayFrame.mirrored(true,false):m_displayFrame;
    m_scaledFrame=QPixmap::fromImage(image.scaled(deviceSize,Qt::KeepAspectRatio,
                                                 Qt::FastTransformation));
    m_scaledFrame.setDevicePixelRatio(m_cachedDpr);
}

void VideoWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    rebuildDisplayCache();
}

void VideoWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setClipRect(rect());
    painter.fillRect(rect(),QColor(11,13,16));
    if(m_displayFrame.isNull()) {
        painter.setPen(QColor(170,181,195));
        painter.drawText(rect(),Qt::AlignCenter,QStringLiteral("NO SIGNAL"));
        paintWarning(painter);
        return;
    }
    if(m_cachedSize!=size() || m_cachedDpr!=devicePixelRatioF())rebuildDisplayCache();
    if(m_scaledFrame.isNull()){paintWarning(painter);return;}
    const QSizeF fitted=m_scaledFrame.deviceIndependentSize();
    painter.drawPixmap(QPointF((width()-fitted.width())/2.0,
                               (height()-fitted.height())/2.0),m_scaledFrame);
    paintWarning(painter);
}

bool VideoWidget::saveFrame(const QString& path) const
{
    QImage snapshot;
    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        snapshot=m_latestFrame;
    }
    if(snapshot.isNull())return false;
    // Keep native image dimensions while including the current local warning.
    QPainter painter(&snapshot);
    QFont font=painter.font();font.setPixelSize(12);painter.setFont(font);
    const auto now=CameraWarningState::nowMs();
    const auto text=m_cameraWarning.text(now);
    const QRect available=snapshot.rect().adjusted(8,6,-8,-6);
    const auto bounds=painter.boundingRect(available,Qt::TextWordWrap,text);
    painter.fillRect(QRect(0,0,snapshot.width(),qMin(snapshot.height(),bounds.height()+12)),
        m_cameraWarning.warning(now)?QColor(145,25,25,235):QColor(20,25,30,220));
    painter.setPen(Qt::white);painter.drawText(available,Qt::TextWordWrap,text);painter.end();
    return snapshot.save(path,"PNG");
}

void VideoWidget::setMirrored(bool mirrored)
{
    if(m_mirrored==mirrored)return;
    m_mirrored=mirrored;
    rebuildDisplayCache();
    update();
}
