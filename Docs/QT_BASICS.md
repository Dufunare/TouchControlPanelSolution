# 这个项目里需要知道的 Qt 最小概念

这一页不是完整 Qt 教程，而是只解释你现在这套架构里一定会碰到的类。

## 1. QApplication

它是 Qt Widgets 程序的入口对象。  
没有它，窗口系统、事件循环、按钮点击、界面刷新都不会工作。

在本项目里：

- `main.cpp` 先创建 `QApplication`
- 再创建 `MainWindow`
- 最后调用 `app.exec()` 进入 Qt 事件循环

---

## 2. QMainWindow

它是主窗口类，适合放：

- 菜单栏
- 工具栏
- 状态栏
- 中央区域（central widget）

在本项目里：

- `MainWindow` 是整个程序的“总装配工”
- 它不直接操作 OpenHaptics
- 它只负责把 3D 区域和右侧状态区摆好，并把信号槽连起来

---

## 3. QWidget

可以把它理解成“任何一个界面小块”的基础类。

比如：

- 一个按钮区
- 一个视频显示区
- 一个坐标显示区
- 一个参数设置区

都可以是 `QWidget` 子类。

在本项目里：

- `StatusPanelWidget` 是一个普通 `QWidget`
- 以后你的 `VideoViewerWidget`、`ForceControlWidget` 也都可以是 `QWidget`

---

## 4. QOpenGLWidget

这是一个专门给你在 Qt Widgets 界面里嵌入 OpenGL 绘图区域的类。

它最重要的三个函数是：

- `initializeGL()`：初始化 OpenGL 资源
- `resizeGL()`：窗口尺寸变化时更新投影
- `paintGL()`：真正画图

在本项目里：

- `GLCoordinateWidget` 继承自 `QOpenGLWidget`
- 负责画网格、XYZ 轴、移动点

---

## 5. Layout（布局）

Qt 里一般不推荐手写坐标去摆控件。  
应该让布局管理器自动帮你摆。

常见布局：

- `QVBoxLayout`：垂直排
- `QHBoxLayout`：水平排
- `QGridLayout`：网格排
- `QFormLayout`：左标签右内容
- `QSplitter`：可以拖动分隔线调整区域大小

在本项目里：

- 主窗口用 `QSplitter`
- 右侧状态面板内部用 `QVBoxLayout + QFormLayout`

---

## 6. QObject / signals / slots

这是 Qt 最有代表性的机制。

你可以把它理解成“事件连线系统”。

例如：

- 点击按钮 -> 发出一个 signal
- Controller 收到 signal -> 执行 slot
- Controller 再发出新 signal -> 通知界面刷新

在本项目里：

- `StatusPanelWidget` 发出：
  - `initializeRequested()`
  - `startRequested()`
  - `stopRequested()`
- `DeviceController` 接收这些信号，并控制后端
- `DeviceController` 再发出：
  - `deviceStateUpdated(...)`
  - `backendMessageChanged(...)`

---

## 7. QTimer

它是 Qt 事件循环里的定时器。

在本项目里：

- OpenHaptics 后端在自己的 scheduler 里高频采样
- `DeviceController` 用 `QTimer` 每 16ms 取一次“最新状态”
- 这样前端大约以 60 FPS 刷新，不会把界面线程拖死

---

## 8. 为什么不要在按钮槽函数里写 while(true)

因为 Qt 的界面刷新、按钮点击、窗口移动、重绘，都依赖主线程的事件循环。

如果在槽函数里写死循环：

- 主线程就不再处理事件
- 窗口不能刷新
- 按钮不能再点击
- 程序看起来就“卡死了”

所以正确做法是：

- OpenHaptics 的实时设备逻辑在后端
- Qt 主线程只做界面与交互
- 前端定时读取后端状态并显示

---

## 9. 可以怎么理解这套项目

你可以把它想成三层：

1. **设备层**  
   OpenHaptics、Touch 设备、servo loop

2. **控制层**  
   `TouchBackend`、`DeviceController`

3. **界面层**  
   `MainWindow`、`GLCoordinateWidget`、`StatusPanelWidget`

以后你加任何功能，都先问自己：

- 这是设备逻辑？
- 这是控制逻辑？
- 这是界面显示？

然后把它塞进对应那层，而不是一股脑全写进主窗口。
