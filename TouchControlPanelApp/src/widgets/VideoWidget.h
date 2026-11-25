#pragma once

#include <cstdint>
#include <mutex>
#include <QImage>
#include <QPixmap>
#include <QWidget>

class QPaintEvent;
class QResizeEvent;
namespace cv { class Mat; }

// setFrame()/showNoSignal() are mailbox-only and safe on the video worker.
// All widget painting and mirror controls stay on the GUI thread.
class VideoWidget : public QWidget
{
public:
    explicit VideoWidget(QWidget* parent = nullptr);
    ~VideoWidget() override = default;

    void setFrame(const QImage& image);
    void setFrameFromMat(const cv::Mat& frame);
    void showNoSignal();
    void clearFrame();
    bool hasSignal() const;
    bool saveFrame(const QString& path) const;
    void setMirrored(bool mirrored);
    void updateFrame();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    mutable std::mutex m_frameMutex;
    QImage m_latestFrame;
    std::uint64_t m_revision=0;
    QImage m_displayFrame;
    std::uint64_t m_displayRevision=0;
    bool m_mirrored=false;
    QPixmap m_scaledFrame;
    QSize m_cachedSize;
    qreal m_cachedDpr=0;
    void rebuildDisplayCache();
};
