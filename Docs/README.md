# OpenHaptics + Qt 控制面板文档索引（当前仓库）

本目录文档已按当前工程结构更新：

- 后端静态库项目：`ControlBackend`
- 前端 Qt 项目：`TouchControlPanelApp`
- 共享状态：`shared/DeviceState.h`

## 当前已实现能力

- OpenHaptics 设备初始化与采样循环
- Qt 控制层轮询后端状态（8ms）
- 3D 坐标可视化（OpenGL）
- 状态/日志面板
- 视频显示占位区（VideoWidget）

## 当前界面布局

```text
MainWindow
└─ QSplitter(水平)
   ├─ 左：QSplitter(垂直)
   │  ├─ VideoWidget
   │  └─ GLCoordinateWidget
   └─ 右：StatusPanelWidget
```

## 先看哪些文档

1. `ARCHITECTURE.md`：完整架构与线程/数据流
2. `ARCHITECTURE_OVERVIEW.md`：快速总览
3. `FILE_PURPOSES.md`：文件职责地图
4. `HOW_TO_EXTEND.md`：如何扩展
5. `VS_QT_SETUP.md`：VS + Qt + OpenHaptics 配置
6. `QT_BASICS.md`：项目中会用到的 Qt 核心概念

## 快速启动建议

1. 先确保 `ControlBackend` 可单独编译。
2. 再编译 `TouchControlPanelApp`，确认可链接 `ControlBackend.lib`。
3. 运行后先点击“初始化设备”，再点击“启动实时采集”。
4. 观察右侧日志与左侧 3D 画面是否同步更新。
