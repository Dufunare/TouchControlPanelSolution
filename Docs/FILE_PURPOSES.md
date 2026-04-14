# 每个文件是做什么的

## shared/

### `shared/DeviceState.h`

这是前后端共享的数据结构。

当前保存的是：

- `positionMm[3]`：XYZ 坐标
- `initialized`：后端是否初始化
- `valid`：当前采样是否有效
- `schedulerRunning`：OpenHaptics scheduler 是否运行
- `sampleCounter`：采样计数

以后如果你想增加：

- 按钮状态
- 速度
- 力
- 关节角
- 时间戳
- 设备 ID

最先改的往往就是这个文件。

---

## TouchBackendLib/

### `TouchBackendLib/include/TouchBackend.h`

后端静态库对外暴露的“公共接口”。

前端只应该知道这些函数：

- `initialize()`
- `start()`
- `stop()`
- `latestState()`
- `lastError()`

这样前端就不需要直接 include OpenHaptics 的头文件。

### `TouchBackendLib/src/TouchBackend.cpp`

真正接 OpenHaptics 的地方。

它负责：

- 初始化设备
- 注册 scheduler 回调
- 启动 scheduler
- 在回调里读取当前坐标
- 保存“最新状态快照”
- 在停止时清理 scheduler / 设备

以后如果要加：

- 按钮读取
- 速度读取
- 力反馈输出
- 校准
- 多设备支持

基本都会改这个文件，或者从这个文件继续拆出更多后端类。

---

## TouchControlPanelApp/src/

### `TouchControlPanelApp/src/main.cpp`

程序入口。

它负责：

- 创建 `QApplication`
- 注册 Qt 元类型
- 设置 OpenGL 默认格式
- 创建主窗口并运行事件循环

### `TouchControlPanelApp/src/DeviceStateQt.h`

把 `touchpanel::DeviceState` 注册为 Qt 可识别的类型。  
这样它就能出现在 signal / slot 的参数里。

### `TouchControlPanelApp/src/DeviceController.h`

Qt 侧的控制器头文件。

它对上接按钮事件，对下调用 `TouchBackend`，再把状态发给界面。

### `TouchControlPanelApp/src/DeviceController.cpp`

控制器实现。

它负责：

- 响应初始化按钮
- 响应启动按钮
- 响应停止按钮
- 用 `QTimer` 定时读取后端状态
- 发出“状态更新”和“消息文本”信号

### `TouchControlPanelApp/src/MainWindow.h`

主窗口类声明。

### `TouchControlPanelApp/src/MainWindow.cpp`

主窗口装配文件。

它负责：

- 创建 `DeviceController`
- 创建左侧 `GLCoordinateWidget`
- 创建右侧 `StatusPanelWidget`
- 用 `QSplitter` 把它们拼起来
- 把 signal / slot 连接起来

以后如果你新增一个界面区域，例如：

- 视频区
- 参数区
- 日志区
- 任务状态区

一般都要改这个文件，把新 Widget 拼进来。

---

## TouchControlPanelApp/src/widgets/

### `widgets/GLCoordinateWidget.h`

OpenGL 坐标显示控件的类声明。

### `widgets/GLCoordinateWidget.cpp`

OpenGL 坐标显示控件的具体实现。

当前负责：

- 初始化 shader
- 创建网格 / 坐标轴 / 点 的 VBO / VAO
- 在 `paintGL()` 里绘制 3D 场景
- 接收 `DeviceState` 后调用 `update()` 刷新

以后如果想加：

- 鼠标旋转视角
- 缩放
- 轨迹线
- 多个点
- 目标点 / 参考坐标系
- 力向量箭头

主要改这个文件。

### `widgets/StatusPanelWidget.h`

右侧状态控件声明。

### `widgets/StatusPanelWidget.cpp`

右侧状态控件实现。

当前负责：

- 显示后端状态
- 显示 X / Y / Z 数值
- 显示采样计数
- 提供初始化 / 启动 / 停止按钮
- 显示后端消息文本

以后如果你想加：

- 参数输入框
- 复选框
- 模式切换按钮
- 日志输出
- 状态灯

主要改这个文件，或者从它继续拆更多 Widget。
