#include <QApplication>
#include <QSurfaceFormat>

#include "DeviceStateQt.h"
#include "MainWindow.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    qRegisterMetaType<touchpanel::DeviceState>("touchpanel::DeviceState");

    // 第一版使用 QOpenGLWidget 画 3D 坐标系与点位，因此这里预先设置一个默认 OpenGL 格式。
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    QSurfaceFormat::setDefaultFormat(format);
	// 上面的代码都是在为了确保 QOpenGLWidget 能够正确创建 OpenGL 3.3 的上下文，且具有 24 位深度缓冲区和双缓冲交换行为。
    MainWindow window;
    window.show();

    return app.exec();
}
