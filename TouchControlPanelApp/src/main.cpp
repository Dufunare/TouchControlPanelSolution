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

    MainWindow window;
    window.show();

    return app.exec();
}
