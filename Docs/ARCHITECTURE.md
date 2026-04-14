# OpenHaptics + Qt 控制面板架构文档

## 阶段 1：实时坐标采集 + 3D 点显示

OpenHaptics + Qt 控制面板  
架构文档与阶段 1 框架说明  
面向 Visual Studio / Qt VS Tools / C++ 初学者

- 文档定位：解释整体架构、项目结构、工作机制、VS/Qt 落地步骤，以及第一阶段代码骨架。
- 目标功能：实时读取 Touch 设备坐标，并用 OpenGL 在 Qt 前端显示 3D 空间中的移动点。
- 建议工程形态：后端做成静态库 TouchBackendLib；前端做成 Qt Widgets 可执行程序 TouchControlPanelApp。
- 交付物：1 份架构文档 + 1 套可落地代码骨架 + 扩展指南。

## 先记住这条底线

不要把 OpenHaptics 的 while(true) 控制循环直接写进 Qt 的按钮槽函数。正确做法是：设备实时循环留在后端或 OpenHaptics scheduler，Qt 主线程只负责界面、布局、按钮和显示。

## 1. 你现在要搭建的到底是什么

你要做的不是一个“把设备代码写进窗口”的小程序，而是一套可以不断长大的控制面板框架。它要同时满足三件事：第一，第一阶段功能能尽快跑起来；第二，后端以后加功能时不要把界面拖垮；第三，前端以后拆区块、加视频、加参数、加状态监控时仍然能保持清晰。

因此，这份方案把系统拆成三层：设备后端层、Qt 控制层、Qt 界面层。其中后端静态库负责 OpenHaptics 和设备状态；Qt 控制层负责把界面动作翻译成后端调用，并周期性读取后端状态；Qt 界面层由多个独立 Widget 组成，主窗口只负责把这些积木拼起来。

## 2. 为什么这套架构适合你现在的项目

- 你当前最怕的问题是：随着功能增长，主窗口文件越来越臃肿，设备逻辑和界面逻辑缠在一起。
- 把后端做成静态库以后，前端不需要直接 include OpenHaptics 的一堆头文件，耦合会明显下降。
- 把每个显示区做成 Widget 以后，你的主窗口就能保持很轻，只做布局和连接。
- 以后新增功能时，可以只改后端、只改前端，或者同时各加一个模块，而不是全改。

### 对你最重要的结果

这不是为了“看起来高级”，而是为了让你以后加功能时还能改得动。尤其是图像回传、日志、参数区、任务状态区这类模块，都会因为这套分层而变得自然。

## 3. 这个项目里你只需要理解的 Qt 最小概念

| 类 / 概念                 | 在这个项目里的作用                                                                |
| ------------------------- | --------------------------------------------------------------------------------- |
| QApplication              | Qt Widgets 程序入口，负责事件循环。没有它，按钮点击、窗口刷新、定时器都不会工作。 |
| QMainWindow               | 整个主窗口的外壳，适合放中央区域、状态栏、菜单栏。这里它只做总装配。              |
| QWidget                   | 所有界面小块的基类。状态面板、视频面板、日志面板都可以做成 QWidget 子类。         |
| QOpenGLWidget             | Qt Widgets 里嵌入 OpenGL 绘制区域的标准控件。当前用它画 3D 坐标系和移动点。       |
| QSplitter                 | 把主界面拆成可拖动宽度的左右区域。当前左侧是 3D，右侧是状态/控制。                |
| QLayout                   | 自动摆控件的布局系统。当前右侧状态区内部主要用 QVBoxLayout 和 QFormLayout。       |
| QObject / signals / slots | Qt 的事件连线机制。按钮发信号，控制器收信号，控制器再通知界面刷新。               |
| QTimer                    | Qt 主线程中的定时器。当前用它每 16ms 读取一次后端最新状态并刷新界面。             |

## 4. 总体架构图

```text
Touch 设备
 ↓
OpenHaptics scheduler / servo loop
 ↓
TouchBackendLib
 ├─ 初始化设备
 ├─ 启动/停止 scheduler
 ├─ 在回调中读取最新位置
 └─ 对外提供 latestState()
 ↓
DeviceController (Qt 侧桥接层)
 ├─ 响应按钮
 ├─ 启动/停止后端
 └─ 用 QTimer 读取最新状态
 ↓
MainWindow
 ├─ GLCoordinateWidget：画 3D 坐标系与点
 └─ StatusPanelWidget：显示状态、坐标、按钮
```

当前这条链路非常简单，但它已经具备扩展能力。以后你加 VideoViewerWidget、LogWidget、TrajectoryWidget 的时候，MainWindow 仍然只是“加一个控件 + 连一个控制器”；后端也只是在静态库里增加新的服务或数据结构。

## 5. 线程与数据流是如何工作的

你现在这个项目最关键的机制不是某个按钮，而是“谁在什么线程里干什么”。如果这个关系理顺了，界面就不会因为设备循环而卡死。

| 层     | 谁在跑                                  | 做什么                                             | 你应该遵守的规则                                          |
| ------ | --------------------------------------- | -------------------------------------------------- | --------------------------------------------------------- |
| 设备层 | OpenHaptics scheduler / servo loop      | 高频读取设备状态，未来也可以在这里做力反馈控制。   | 不要把重 UI、磁盘 IO、网络请求塞进这个回调。              |
| 控制层 | Qt 主线程中的 DeviceController + QTimer | 每隔一小段时间读取 latestState()，把状态发给界面。 | 控制器负责“调度”，不要在主窗口里直接写后端逻辑。          |
| 界面层 | Qt 主线程中的 QWidget / QOpenGLWidget   | 显示文本、按钮、3D 画面。                          | 所有 QWidget 都只留在主线程里，不要从后台线程直接碰它们。 |

### 阶段 1 的推荐节奏

设备侧高频采样，界面侧低频显示。也就是说，后端负责每个 servo tick 更新“最新状态”，前端只需要每 16ms 左右读一次最新值并刷新。

## 6. 第一阶段功能是怎样跑起来的

1. 程序启动时，`main.cpp` 创建 `QApplication`，设置 `QOpenGLWidget` 默认格式，然后创建 `MainWindow`。
2. `MainWindow` 创建三个核心对象：`TouchBackend`、`DeviceController`、`GLCoordinateWidget / StatusPanelWidget`。
3. 当你点击“初始化设备”时，`StatusPanelWidget` 发出信号，`DeviceController` 调用 `TouchBackend::initialize()`。
4. 当你点击“启动实时采集”时，`DeviceController` 调用 `TouchBackend::start()`，后者注册 OpenHaptics 回调并启动 scheduler。
5. OpenHaptics callback 在每个 servo tick 内读取 `HD_CURRENT_POSITION`，并把最新 XYZ 保存到后端内部状态。
6. `DeviceController` 内部的 `QTimer` 每 16ms 触发一次，调用 `latestState()` 读取最新快照。
7. `MainWindow` 收到状态后，一份送给 `GLCoordinateWidget` 去画点，一份送给 `StatusPanelWidget` 去更新文字。
8. 当你点击“停止采集”时，`DeviceController` 停止 QTimer 并调用 `TouchBackend::stop()`，回调被取消，scheduler 停止。

## 7. 当前推荐的项目结构

```text
TouchControlPanelSolution/
├─ shared/
│ └─ DeviceState.h
├─ TouchBackendLib/
│ ├─ include/
│ │ └─ TouchBackend.h
│ └─ src/
│   └─ TouchBackend.cpp
└─ TouchControlPanelApp/
	└─ src/
		├─ main.cpp
		├─ MainWindow.h
		├─ MainWindow.cpp
		├─ DeviceStateQt.h
		├─ DeviceController.h
		├─ DeviceController.cpp
		└─ widgets/
			├─ GLCoordinateWidget.h
			├─ GLCoordinateWidget.cpp
			├─ StatusPanelWidget.h
			└─ StatusPanelWidget.cpp
```

这个目录结构的好处是非常明确：共享数据放 `shared/`，OpenHaptics 相关逻辑只在静态库里，Qt 前端自己的代码只在应用项目里。你以后如果再加一个视频流服务或轨迹服务，也能继续沿用这个规律。

## 8. 每个文件具体负责什么

| 文件                                                       | 职责                                                              | 以后一般什么时候会改它                                 |
| ---------------------------------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------ |
| shared/DeviceState.h                                       | 前后端共享的最新状态快照。当前有坐标、初始化状态、采样循环状态。  | 增加按钮状态、速度、关节角、时间戳等字段时先改这里。   |
| TouchBackendLib/include/TouchBackend.h                     | 后端静态库对外暴露的最小公共接口。                                | 后端接口形态改变时再改。                               |
| TouchBackendLib/src/TouchBackend.cpp                       | 真正接 OpenHaptics 的地方：初始化、启动 scheduler、读坐标、清理。 | 加按钮、速度、力反馈、校准、多设备时主要改这里。       |
| TouchControlPanelApp/src/main.cpp                          | Qt 程序入口，创建应用和主窗口，设置 OpenGL 格式。                 | 换 OpenGL 版本或全局初始化逻辑时会改。                 |
| TouchControlPanelApp/src/DeviceStateQt.h                   | 把共享状态注册为 Qt 类型，供信号槽系统使用。                      | 共享状态类型变化时一般要一起看一眼。                   |
| TouchControlPanelApp/src/DeviceController.h/.cpp           | Qt 侧控制桥。响应按钮、启动后端、定时读状态、向界面发更新信号。   | 引入新数据流、新控制按钮、新控制策略时会改。           |
| TouchControlPanelApp/src/MainWindow.h/.cpp                 | 主窗口装配中心，只负责创建控件、布局、连接信号。                  | 加新区域、新 Widget、新控制器时会改。                  |
| TouchControlPanelApp/src/widgets/GLCoordinateWidget.h/.cpp | 用 OpenGL 画网格、XYZ 轴和移动点。                                | 加轨迹线、相机控制、参考点、目标点、多设备点位时会改。 |
| TouchControlPanelApp/src/widgets/StatusPanelWidget.h/.cpp  | 右侧状态与控制面板。                                              | 加参数输入、日志区、开关、模式按钮时会改。             |

## 9. 为什么后端必须做成静态库

把后端做成静态库，本质上是在做“职责边界”。前端只认识 `TouchBackend.h` 这一层接口，而不需要直接碰 OpenHaptics SDK。这样做以后，你可以单独编译和测试后端，也可以让不同前端去共用同一个后端库。

- 前端不再直接依赖 OpenHaptics 细节。
- 主窗口不会因为设备代码越来越多而失控。
- 以后你可以把单元测试、命令行测试、小工具程序都链接到同一个后端库。
- 对你这种功能会不断增长的项目，这种拆法比“全都堆在 Qt 工程里”更耐用。

## 10. Visual Studio + Qt VS Tools 的落地步骤

9. 先安装 Visual Studio 2022，并勾选“使用 C++ 的桌面开发”。
10. 安装 Qt 6.x 的 MSVC 2022 64-bit 版本。
11. 在 Visual Studio 里通过 Extensions > Manage Extensions > Online 安装 Qt Visual Studio Tools。
12. 重启 Visual Studio 后，在 Extensions > Qt VS Tools > Qt Versions 中添加你的 Qt 版本，把 `qmake.exe` 路径填进去。
13. 创建一个解决方案，例如 `TouchControlPanelSolution`。
14. 先新建纯 C++ 静态库项目 `TouchBackendLib`。
15. 再新建 Qt Widgets Application 项目 `TouchControlPanelApp`。
16. 把文档交付的源码文件按对应目录加入两个项目。
17. 给后端项目配置 OpenHaptics 的 include 和 lib 路径，并把你实际需要的 OpenHaptics `.lib` 加到 Linker Input。
18. 给前端项目配置 `..\shared` 和 `..\TouchBackendLib\include` 的头文件路径。
19. 给前端项目把 `TouchBackendLib.lib` 加到 Additional Dependencies，并把后端输出目录加到 Additional Library Directories。
20. 在 Solution 的 Project Dependencies 里勾选：`TouchControlPanelApp` 依赖 `TouchBackendLib`。
21. 第一次调试时，先确保后端能单独编译，再编译前端。
22. 前端能打开窗口后，再连接设备，点击初始化，再点击启动实时采集。

### Qt Designer 在哪里用

这版代码为了便于你理解，主窗口先使用纯 C++ 手写。以后如果你想用 Qt Designer，只需要给某些子区域单独创建 `.ui` 文件，例如状态面板、视频面板、设置面板；而 OpenGL 绘图区通常仍然推荐手写自定义控件。

## 11. 主界面为什么当前推荐用 QSplitter

你原本提到过 QGridLayout 和 QSplitter，这两个都对。当前这一版代码选择最外层用 QSplitter，是因为它最适合做“左侧 3D 大区域 + 右侧状态/控制区域”这种二分结构，并且用户后续可以自己拖动分界线调整比例。

当你以后真的要把右侧继续拆成‘右上视频区 + 右下控制区’时，可以保留最外层 QSplitter 不变，然后在右侧内部再放一个 QWidget，给它套上 QVBoxLayout 或 QGridLayout。

## 12. 第一阶段代码骨架里最关键的 3 个类

| 类                 | 一句话理解        | 为什么重要                                                                   |
| ------------------ | ----------------- | ---------------------------------------------------------------------------- |
| TouchBackend       | 设备后端封装器    | 它把 OpenHaptics 设备初始化、scheduler、状态采样全部包起来，是整个后端入口。 |
| DeviceController   | Qt 与后端之间的桥 | 它把按钮动作变成后端调用，把后端状态变成前端刷新信号。                       |
| GLCoordinateWidget | 3D 坐标显示控件   | 它证明你的 Qt 前端可以独立地把设备坐标可视化出来。                           |

## 13. 以后新增功能时，新增/修改哪些文件

| 场景                     | 新增文件                                                                     | 通常要修改的文件                                   |
| ------------------------ | ---------------------------------------------------------------------------- | -------------------------------------------------- |
| 只加前端区域             | 新 Widget 类，例如 `LogWidget.h/.cpp`                                        | `MainWindow.cpp`；必要时改 `DeviceController.cpp`  |
| 只加后端数据             | 一般不一定新增文件，先扩展 `DeviceState`                                     | `shared/DeviceState.h`、`TouchBackend.cpp`         |
| 新业务模块（前后端联动） | 新的 Service / Controller / Widget / Shared 数据结构                         | `MainWindow.cpp` + 该模块相关文件                  |
| 视频回传 / 图像通道      | `VideoFrame.h`、`VideoStreamService`、`VideoViewerWidget`、`VideoController` | `MainWindow.cpp`，但不要把大图像塞进 `DeviceState` |

你可以把以后加功能这件事分成四类：只加前端、只加后端、前后端联动的新业务、单独的数据通道（比如图像）。只要你每次都先判断它属于哪一类，再决定新文件放哪里，你的工程就不会变成一锅粥。

## 14. 如果以后要加图像回传，应该怎么长

图像帧和位置状态不是同一种数据。位置状态适合放进一个小而快的 `DeviceState` 快照里；图像帧数据量大、更新节奏也可能不同，所以更推荐走单独的数据通道。

```text
shared/
└─ VideoFrame.h
TouchBackendLib/
├─ include/
│ └─ VideoStreamService.h
└─ src/
	└─ VideoStreamService.cpp
TouchControlPanelApp/
└─ src/
	├─ VideoController.h
	├─ VideoController.cpp
	└─ widgets/
		├─ VideoViewerWidget.h
		└─ VideoViewerWidget.cpp
```

然后在 `MainWindow.cpp` 里把 `VideoViewerWidget` 加入布局。这样图像流和设备状态流彼此独立，后面你调试时也会舒服很多。

## 15. 常见坑和应对方式

- 窗口一点击按钮就卡死：通常是你把 while(true) 或阻塞逻辑塞进了槽函数。
- OpenGL 区域黑屏：先检查 Qt 项目有没有正确启用 OpenGLWidgets，显卡上下文能不能创建出来。
- 后端编译不过：优先检查 OpenHaptics include/lib 路径、x64 配置、Debug/Release 配置是否一致。
- 设备初始化失败：先检查驱动、USB/LAN 连接、官方控制面板是否能识别设备。
- 坐标方向感觉不对：先确认你的坐标系约定，再决定是否在显示层做轴向映射。
- 以后功能一多，主窗口变乱：说明你该把新业务拆成新的 Widget / Controller / Service 了。

## 16. 你现在应该怎样使用这套交付物

23. 先通读本文件，理解为什么前端和后端要拆开。
24. 再看代码包里的 `README.md`、`docs/VS_QT_SETUP.md` 和 `docs/FILE_PURPOSES.md`。
25. 按文档顺序在 VS 里创建解决方案、后端静态库、Qt 前端工程。
26. 先把窗口跑起来，再接设备。
27. 第一阶段跑通以后，再决定你下一个要加的是参数区、日志区、轨迹区还是视频区。

## 附录 A. 你继续查资料时，优先看的官方文档标题

- Qt：Threads and QObjects
- Qt：QOpenGLWidget Class
- Qt VS Tools：Install Qt VS Tools
- Qt VS Tools：Add Qt versions
- Qt VS Tools：Building
- Qt VS Tools：Create UI form files
- Qt VS Tools：Start Qt Widgets Designer
- 3D Systems：OpenHaptics Developer Software
- 3D Systems：OpenHaptics API Reference Guide（重点看 scheduler、hdGet、hdStartScheduler）
