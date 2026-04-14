# OpenHaptics + Qt 控制面板框架（阶段 1）

这是一个给 **Visual Studio + Qt VS Tools + OpenHaptics** 使用的“分层骨架”。

这一版框架解决的核心问题是：

1. **不要把 OpenHaptics 的实时循环写进 Qt 按钮槽函数**  
   设备采样和 servo loop 放在后端，由 OpenHaptics scheduler 驱动；Qt 只负责显示和交互。
2. **把后端和前端拆成两个项目**  
   `TouchBackendLib` 是静态库，`TouchControlPanelApp` 是 Qt 可执行程序。
3. **把界面拆成独立 Widget**  
   `GLCoordinateWidget` 负责 3D 显示，`StatusPanelWidget` 负责状态和控制，主窗口只做拼装与连线。

---

## 当前实现了什么

当前代码实现的是的**第一阶段功能**：

- 通过 OpenHaptics 后端实时获取 Touch 设备当前位置
- 通过 Qt 前端定时读取“最新一帧状态”
- 使用 `QOpenGLWidget` 在 3D 坐标系中显示一个移动点
- 在右侧状态面板中显示 X / Y / Z 数值、运行状态、采样计数
- 用 `QSplitter` 把“3D 区域”和“状态/控制区域”拆成两个可调宽度区域

---

## 项目结构

```text
TouchControlPanelSolution/
├─ shared/
│  └─ DeviceState.h
├─ TouchBackendLib/
│  ├─ include/
│  │  └─ TouchBackend.h
│  └─ src/
│     └─ TouchBackend.cpp
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

---

## 线程 / 数据流

```text
Touch 设备
   ↓
OpenHaptics scheduler callback (后端 servo loop)
   ↓
TouchBackend 保存“最新状态快照”
   ↓
DeviceController 用 QTimer 每 16ms 读取一次状态
   ↓
MainWindow 收到状态
   ├─ 喂给 GLCoordinateWidget 画点
   └─ 喂给 StatusPanelWidget 更新数值
```

---

## 文档分类

- `docs/VS_QT_SETUP.md`  
  关于怎么在 VS 里装 Qt 插件、建工程、配路径、链接静态库。
- `docs/FILE_PURPOSES.md`  
  每个文件是干什么的。
- `docs/HOW_TO_EXTEND.md`  
  加新功能时，需要新增/修改哪些文件。
- `docs/QT_BASICS.md`  
  针对 Qt 的最小概念说明。

---

## 使用顺序

1. 先在 Visual Studio 里创建 **解决方案**。
2. 先创建 **后端静态库项目 `TouchBackendLib`**。
3. 再创建 **Qt Widgets 前端项目 `TouchControlPanelApp`**。
4. 把这里的 `.h/.cpp` 文件按对应目录加入项目。
5. 给后端配置 OpenHaptics 的 include / lib 路径。
6. 给前端配置 `TouchBackendLib.lib` 的依赖。
7. 先编译后端，再编译前端。
8. 连接好设备、驱动正常后运行程序。

---

## 需要自己检查 / 微调的地方

这份骨架已经按照官方文档的思路搭好了，但你仍然需要根据自己的 OpenHaptics 安装情况微调以下内容：

- OpenHaptics SDK 的头文件目录
- OpenHaptics SDK 的库文件目录
- 你的 SDK 实际使用的 `.lib` 名称
- 设备是否需要额外的校准流程
- 设备工作空间是否需要调整显示比例
- 如果显卡 / OpenGL 上下文不支持 3.3 Core Profile，需要下调 OpenGL 版本

---

## 后续扩展

### 1) 只加前端功能

例如：新增日志区、参数区、图表区

- 新增一个 Widget 类
- 在 `MainWindow.cpp` 里把它加进布局
- 在 `DeviceController` 或新的 Controller 中给它喂数据

### 2) 只加后端功能

如：读取按钮状态、速度、关节角

- 扩展 `DeviceState.h`
- 在 `TouchBackend.cpp` 的 scheduler 回调里采集更多状态
- 在前端决定是否显示这些数据

### 3) 前后端联动的新功能

例如：力反馈控制、视频回传、任务状态机

- `shared/` 里新增共享数据结构
- `TouchBackendLib/` 里新增服务类
- `TouchControlPanelApp/` 里新增 Widget / Controller
- `MainWindow.cpp` 负责把新模块拼接进界面

---

## 重点

**主窗口只负责“摆积木”和“连线”；真正的业务逻辑，放后端或独立 Controller；真正的界面显示，放独立 Widget。**
