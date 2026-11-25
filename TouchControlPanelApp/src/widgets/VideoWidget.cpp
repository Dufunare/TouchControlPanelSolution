#include "VideoWidget.h"

#include <QPainter>
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
    update();
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
    const QSize fitted=m_displayFrame.size().scaled(size(),Qt::KeepAspectRatio);
    const QRect target((width()-fitted.width())/2,(height()-fitted.height())/2,
                       fitted.width(),fitted.height());
    painter.setRenderHint(QPainter::SmoothPixmapTransform,true);
    if(m_mirrored) {
        painter.translate(width(),0);
        painter.scale(-1,1);
    }
    painter.drawImage(target,m_displayFrame);
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
    update();
}
