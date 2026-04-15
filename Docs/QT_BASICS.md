# 这个项目里需要知道的 Qt 最小概念（当前实现）

## 1. QApplication

Qt Widgets 程序入口。负责事件循环。

在本项目中：

- `main.cpp` 创建 `QApplication`
- 设置 OpenGL 默认格式
- 创建并显示 `MainWindow`

## 2. QMainWindow

主窗口壳层，用于管理 central widget、状态栏等。

在本项目中：

- `MainWindow` 只做装配与连线
- 不直接执行 OpenHaptics 设备逻辑

## 3. QWidget

界面模块的基类。可以把每个区域拆成独立 Widget。

本项目里的例子：

- `StatusPanelWidget`
- `VideoWidget`

## 4. QOpenGLWidget

用于在 Qt Widgets 中嵌入 OpenGL 绘图。

关键函数：

- `initializeGL()`
- `resizeGL()`
- `paintGL()`

本项目里 `GLCoordinateWidget` 用它渲染 3D 坐标场景。

## 5. QSplitter 与 Layout

布局是当前界面结构核心。

本项目现在是双层 `QSplitter`：

- 外层水平分割：左侧可视化，右侧状态面板
- 左侧再垂直分割：上视频、下 3D

## 6. QObject / signal / slot

Qt 的事件连接机制。

本项目中：

- `StatusPanelWidget` 发出初始化/启动/停止请求
- `DeviceController` 接收请求并调用后端
- `DeviceController` 再发出状态更新与消息更新

## 7. QTimer

Qt 主线程定时器。

本项目中：

- `DeviceController` 用 `QTimer` 每 `8ms` 轮询一次后端状态
- 轮询频率高于传统 16ms，保持更流畅的坐标更新体验

## 8. 为什么不能在按钮槽里写 while(true)

Qt 界面响应依赖主线程事件循环。若在槽函数里阻塞：

- 窗口会卡死
- 重绘会停止
- 按钮无法继续交互

正确做法：

- 设备实时循环留在 OpenHaptics scheduler（后端）
- Qt 主线程只负责显示与交互

## 9. 线程边界要点

- OpenHaptics 回调线程：采集设备状态
- Qt 主线程：更新 QWidget
- 像 `VideoWidget` 这类控件若跨线程送帧，要通过 queued invoke 回主线程刷新
