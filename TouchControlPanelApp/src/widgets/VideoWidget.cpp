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
    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        if(m_displayRevision==m_revision)return;
        m_displayFrame=m_latestFrame;
        m_displayRevision=m_revision;
    }
    rebuildDisplayCache();
    update();
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
        return;
    }
    if(m_cachedSize!=size() || m_cachedDpr!=devicePixelRatioF())rebuildDisplayCache();
    if(m_scaledFrame.isNull())return;
    const QSizeF fitted=m_scaledFrame.deviceIndependentSize();
    painter.drawPixmap(QPointF((width()-fitted.width())/2.0,
                               (height()-fitted.height())/2.0),m_scaledFrame);
}

bool VideoWidget::saveFrame(const QString& path) const
{
    QImage snapshot;
    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        snapshot=m_latestFrame;
    }
    return !snapshot.isNull() && snapshot.save(path,"PNG");
}

void VideoWidget::setMirrored(bool mirrored)
{
    if(m_mirrored==mirrored)return;
    m_mirrored=mirrored;
    rebuildDisplayCache();
    update();
}
