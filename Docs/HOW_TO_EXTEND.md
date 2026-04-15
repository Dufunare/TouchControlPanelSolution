# 后续怎么加新功能（按当前项目）

## 先记一个原则

每次扩展前，先判断功能属于哪一层：

- 设备采集/控制：后端层（ControlBackend）
- 调度与转发：控制层（DeviceController 或新 Controller）
- 展示与交互：界面层（Widget）

主窗口只负责拼装，不承载复杂业务。

---

## 场景 A：只加前端显示

例如：新增参数区、日志图表区、状态灯区。

推荐做法：

1. 新建 Widget 文件
2. 在 `MainWindow.cpp` 拼接到布局
3. 通过现有或新增 Controller 提供数据

典型改动文件：

- `TouchControlPanelApp/src/widgets/NewWidget.h`
- `TouchControlPanelApp/src/widgets/NewWidget.cpp`
- `TouchControlPanelApp/src/MainWindow.cpp`

---

## 场景 B：只加设备状态

例如：按钮状态、速度、力、关节角。

推荐做法：

1. 扩展 `shared/DeviceState.h`
2. 在 `ControlBackend/src/TouchBackend.cpp` 回调中填充数据
3. 按需要在界面展示

典型改动文件：

- `shared/DeviceState.h`
- `ControlBackend/src/TouchBackend.cpp`
- `TouchControlPanelApp/src/widgets/StatusPanelWidget.cpp`（可选）
- `TouchControlPanelApp/src/widgets/GLCoordinateWidget.cpp`（可选）

---

## 场景 C：新增独立业务模块（前后端联动）

例如：轨迹录制回放、任务流程控制、力反馈策略。

推荐做法：

- 新增后端服务类
- 新增前端 Controller
- 新增对应 Widget
- 主窗口只做实例化与连接

建议目录示例：

```text
ControlBackend/include/TrajectoryService.h
ControlBackend/src/TrajectoryService.cpp
TouchControlPanelApp/src/TrajectoryController.h
TouchControlPanelApp/src/TrajectoryController.cpp
TouchControlPanelApp/src/widgets/TrajectoryWidget.h
TouchControlPanelApp/src/widgets/TrajectoryWidget.cpp
shared/TrajectoryFrame.h
```

---

## 场景 D：接入真实视频流

当前项目已有 `VideoWidget`，可直接作为显示层。

推荐接入路线：

1. 新增采集线程或视频服务模块
2. 把原始帧统一转换为 `QImage`
3. 在 UI 线程调用 `VideoWidget::setFrame()`
4. 如需 OpenCV，补齐 `setFrameFromMat()`

不要把大图像帧塞进 `DeviceState`。`DeviceState` 只放“小而高频”的状态快照。

---

## 当前项目里的扩展检查清单

1. 新字段是否先定义在 `shared/DeviceState.h`
2. OpenHaptics 调用是否留在后端而非 UI
3. QWidget 更新是否仅发生在 Qt 主线程
4. 新功能是否拆成独立 Widget/Controller
5. `MainWindow.cpp` 是否仍保持“装配角色”
