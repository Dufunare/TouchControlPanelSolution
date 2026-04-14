# 后续怎么加新功能

---

## 场景 A：只加前端功能

例如：

- 右侧加一个日志区域
- 加一个参数设置区域
- 加一个任务状态灯区域
- 加一个曲线图区域

### 要新增的文件

例如要加日志区：

- `TouchControlPanelApp/src/widgets/LogWidget.h`
- `TouchControlPanelApp/src/widgets/LogWidget.cpp`

### 要改的文件

- `MainWindow.cpp`  
  把 `LogWidget` 加进布局
- 如果日志数据来自后端，还要改 `DeviceController.cpp`

### 原则

- 日志怎么显示 -> Widget 管
- 日志什么时候刷新 -> Controller 管
- 主窗口只负责把它摆进去

---

## 场景 B：只加后端功能

例如：

- 读取按钮状态
- 读取速度
- 读取关节角
- 读取安全开关状态

### 你要改的文件

1. `shared/DeviceState.h`  
   增加字段
2. `TouchBackendLib/src/TouchBackend.cpp`  
   在 OpenHaptics 回调里采集并填充新字段

### 可能还要改的文件

- `StatusPanelWidget.cpp`  
  如果你想把这些值显示出来
- `GLCoordinateWidget.cpp`  
  如果你想把这些值画出来

### 举例

如果加按钮状态，可以在 `DeviceState` 里加：

```cpp
int buttons = 0;
```

然后在 `TouchBackend.cpp` 的 haptics frame 里加：

```cpp
HDint buttons = 0;
hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
```

然后把这个值写进共享状态。

---

## 场景 C：前后端联动的新业务模块

例如：

- 力反馈控制
- 自动回零
- 任务脚本执行
- 轨迹录制 / 回放

这是最典型的“前后端都要动”的扩展。

### 推荐做法

新建一组“业务模块文件”。

例如加“轨迹录制”：

**后端：**

- `TouchBackendLib/include/TrajectoryService.h`
- `TouchBackendLib/src/TrajectoryService.cpp`

**前端：**

- `TouchControlPanelApp/src/widgets/TrajectoryWidget.h`
- `TouchControlPanelApp/src/widgets/TrajectoryWidget.cpp`
- `TouchControlPanelApp/src/TrajectoryController.h`
- `TouchControlPanelApp/src/TrajectoryController.cpp`

**共享：**

- `shared/TrajectoryFrame.h`

### 主窗口怎么改

- 在 `MainWindow.cpp` 里创建 `TrajectoryWidget`
- 把它加到布局里
- 把它和 `TrajectoryController` 连起来

### 原则

- 新业务尽量有自己独立的 Widget / Controller / Service
- 不要把所有新代码都堆进 `MainWindow.cpp`
- 不要把所有后端逻辑都堆进 `TouchBackend.cpp`

---

## 场景 D：以后加图像回传 / 视频区

这类功能和“设备位置”不是同一种数据流，所以**不要硬塞进 `DeviceState`**。

### 为什么

`DeviceState` 适合放“小而高频的状态快照”，比如：

- 坐标
- 速度
- 按钮
- 力

但图像帧通常：

- 数据量大
- 更新频率和坐标不同
- 生命周期不同
- 更适合单独通道传输

### 推荐新增的文件

**共享：**

- `shared/VideoFrame.h`

**后端：**

- `TouchBackendLib/include/VideoStreamService.h`
- `TouchBackendLib/src/VideoStreamService.cpp`

**前端：**

- `TouchControlPanelApp/src/widgets/VideoViewerWidget.h`
- `TouchControlPanelApp/src/widgets/VideoViewerWidget.cpp`
- `TouchControlPanelApp/src/VideoController.h`
- `TouchControlPanelApp/src/VideoController.cpp`

**主窗口：**

- 修改 `MainWindow.cpp`
- 把 `VideoViewerWidget` 放到合适区域

### 布局建议

现在的布局很适合这样扩展：

- 左侧大区域：3D 触觉 / 坐标显示
- 右上：视频回传
- 右下：按钮 + 参数 + 状态

这时可以这样改：

- 主窗口仍然保留一个最外层 `QSplitter`
- 右侧区域内部换成 `QVBoxLayout` 或 `QGridLayout`
- 在右上放 `VideoViewerWidget`
- 在右下放 `StatusPanelWidget` 和其他控制块

---

## 以后每次加功能时，先问自己这 4 个问题

1. 这个功能是“设备逻辑”还是“界面显示”？
2. 这个功能的数据应该放到共享状态里，还是应该单独建数据通道？
3. 这个功能要不要单独 Widget？
4. 这个功能是不是应该单独 Controller / Service？

只要每次都先分层，再写代码，项目就不会越来越乱。
