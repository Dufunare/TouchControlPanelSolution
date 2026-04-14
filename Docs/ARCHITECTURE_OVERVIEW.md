# 架构总览

## 1. 目标

构建一套适合长期演进的 OpenHaptics + Qt 控制面板框架，满足：

- 实时获取 Touch 设备坐标
- 在 Qt 前端显示 3D 坐标系中的移动点
- 后续能容易扩展新后端功能
- 后续能容易扩展新前端区域
- 前后端可以“一起扩展”，也可以“各自独立扩展”

---

## 2. 分层

### 层 1：设备后端层

`TouchBackendLib`

负责：

- 初始化 OpenHaptics
- 启动 / 停止 scheduler
- 读取设备状态
- 保存最新状态快照
- 向外提供统一接口

### 层 2：Qt 控制层

`DeviceController`

负责：

- 把界面按钮动作翻译成后端调用
- 定时从后端取状态
- 把状态广播给界面

### 层 3：Qt 界面层

`MainWindow + Widgets`

负责：

- 布局
- 交互
- 显示
- 局部界面模块化

---

## 3. 分层理由

### 这样做以后：

- OpenHaptics 后端代码不会和 UI 缠在一起
- 新增 Widget 时，不必大改后端
- 新增设备功能时，不必把逻辑都塞进主窗口
- 以后做视频区 / 参数区 / 日志区 / 轨迹区都很自然

### 如果不这样做，最容易出现的坏结果：

- `MainWindow.cpp` 过于冗长不好调试
- OpenHaptics 调用和 Qt 代码互相耦合
- 改一个按钮，后端也需要一起改动
- 加新区域时越来越难维护

---

## 4. 当前第一阶段的数据流

```text
[Touch 设备]
     ↓
[OpenHaptics scheduler callback]
     ↓
[TouchBackend 内部保存最新 DeviceState]
     ↓
[DeviceController 每 16ms 读取一次 latestState()]
     ↓
[MainWindow 分发给各个 Widget]
     ├─ GLCoordinateWidget 画 3D 点
     └─ StatusPanelWidget 显示文字状态
```

---

## 5. 当前界面结构

```text
MainWindow
└─ QSplitter (水平)
   ├─ 左侧：GLCoordinateWidget
   └─ 右侧：StatusPanelWidget
```

以后可能会扩展为：

```text
MainWindow
└─ QSplitter (水平)
   ├─ 左侧：GLCoordinateWidget
   └─ 右侧：QWidget + QGridLayout
      ├─ 右上：VideoViewerWidget
      ├─ 右中：StatusPanelWidget
      └─ 右下：ControlWidget / LogWidget
```

---

## 6. 扩展规则

### 规则 1

**共享数据先放 `shared/`。**

### 规则 2

**设备逻辑先放后端库。**

### 规则 3

**界面逻辑先放独立 Widget。**

### 规则 4

**主窗口只负责拼装，不负责重业务。**

### 规则 5

**大块新功能优先拆成新 Controller / Service / Widget。**

---

## 7. 当前最重要的 3 个文件

### `TouchBackend.cpp`

这是设备数据入口。

### `DeviceController.cpp`

这是 Qt 和后端的桥。

### `MainWindow.cpp`

这是整个界面装配中心。
