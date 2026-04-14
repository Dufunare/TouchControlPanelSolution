#pragma once

#include <atomic>
#include <mutex>

#include <QImage>
#include <QWidget>

class QLabel;
class QResizeEvent;

namespace cv
{
    class Mat;
}

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
    void updateFrame();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    QLabel* m_videoLabel = nullptr;
    QLabel* m_noSignalLabel = nullptr;

    mutable std::mutex m_frameMutex;
    QImage m_currentFrame;
    std::atomic<bool> m_hasSignal{ false };
};
