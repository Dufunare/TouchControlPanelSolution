#include "VideoWidget.h"

#include <QLabel>
#include <QMetaObject>
#include <QPixmap>
#include <QResizeEvent>
#include <QStackedLayout>
#include <QThread>

VideoWidget::VideoWidget(QWidget* parent)
    : QWidget(parent)
{
    setStyleSheet("background-color: #0B0D10;");

    m_videoLabel = new QLabel(this);
    m_videoLabel->setAlignment(Qt::AlignCenter);
    m_videoLabel->setScaledContents(false);
    m_videoLabel->setStyleSheet("background-color: transparent;");

    m_noSignalLabel = new QLabel("NO SIGNAL", this);
    m_noSignalLabel->setAlignment(Qt::AlignCenter);
    m_noSignalLabel->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    m_noSignalLabel->setStyleSheet(
        "background-color: rgba(0, 0, 0, 80);"
        "color: rgba(220, 220, 220, 210);"
        "font-size: 30px;"
        "font-weight: 600;");

    auto* stacked = new QStackedLayout(this);
    stacked->setContentsMargins(0, 0, 0, 0);
    stacked->setStackingMode(QStackedLayout::StackAll);
    stacked->addWidget(m_videoLabel);
    stacked->addWidget(m_noSignalLabel);

    showNoSignal();
}

void VideoWidget::setFrame(const QImage& image)
{
    if (QThread::currentThread() != thread())
    {
        const QImage copied = image.copy();
        QMetaObject::invokeMethod(this,
            [this, copied]()
            {
                setFrame(copied);
            },
            Qt::QueuedConnection);
        return;
    }

    if (image.isNull())
    {
        showNoSignal();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        m_currentFrame = image.copy();
    }

    m_hasSignal.store(true, std::memory_order_relaxed);
    m_noSignalLabel->hide();
    updateFrame();
}

void VideoWidget::setFrameFromMat(const cv::Mat& frame)
{
    Q_UNUSED(frame);
}

void VideoWidget::showNoSignal()
{
    if (QThread::currentThread() != thread())
    {
        QMetaObject::invokeMethod(this,
            [this]()
            {
                showNoSignal();
            },
            Qt::QueuedConnection);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        m_currentFrame = QImage();
    }

    m_hasSignal.store(false, std::memory_order_relaxed);
    m_videoLabel->clear();
    m_noSignalLabel->show();
}

void VideoWidget::clearFrame()
{
    showNoSignal();
}

bool VideoWidget::hasSignal() const
{
    return m_hasSignal.load(std::memory_order_relaxed);
}

void VideoWidget::updateFrame()
{
    if (QThread::currentThread() != thread())
    {
        QMetaObject::invokeMethod(this,
            [this]()
            {
                updateFrame();
            },
            Qt::QueuedConnection);
        return;
    }

    QImage frameCopy;
    {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        frameCopy = m_currentFrame;
    }

    if (!m_hasSignal.load(std::memory_order_relaxed) || frameCopy.isNull())
    {
        m_videoLabel->clear();
        m_noSignalLabel->show();
        return;
    }

    const QPixmap pix = QPixmap::fromImage(frameCopy);
    const QSize targetSize = m_videoLabel->size();
    m_videoLabel->setPixmap(pix.scaled(targetSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    m_noSignalLabel->hide();
}

void VideoWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    updateFrame();
}
