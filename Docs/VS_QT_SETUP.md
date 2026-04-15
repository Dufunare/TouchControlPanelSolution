# 从零开始：Visual Studio + Qt VS Tools + OpenHaptics（按当前工程）

本文档按当前仓库结构编写：

- 静态库项目：`ControlBackend`
- Qt 前端项目：`TouchControlPanelApp`
- 共享头：`shared/DeviceState.h`

## 1. 环境准备

1. 安装 Visual Studio 2022（勾选 C++ 桌面开发）
2. 安装 Qt（MSVC 2022 64-bit 版本）
3. 安装扩展 Qt Visual Studio Tools
4. 在 Qt VS Tools 中添加 Qt 版本（qmake 路径）

## 2. 解决方案结构

当前推荐结构：

```text
TouchControlPanelSolution/
├─ ControlBackend/
├─ TouchControlPanelApp/
└─ shared/
```

## 3. 配置 ControlBackend（后端静态库）

### 3.1 项目类型

- Visual C++ Static Library

### 3.2 关键文件

- `ControlBackend/include/TouchBackend.h`
- `ControlBackend/src/TouchBackend.cpp`
- `shared/DeviceState.h`

### 3.3 Include 路径（Debug|x64 示例）

`C/C++ > Additional Include Directories`：

```text
$(MSBuildThisFileDirectory);$(ProjectDir)include;C:\OpenHaptics\Developer\3.5.0\include;C:\OpenHaptics\Developer\3.5.0\utilities\include
```

### 3.4 库依赖（Debug|x64 示例）

- `Lib > Additional Dependencies`：`hd.lib;hl.lib;hdu.lib`
- `LibraryPath` 包含：

```text
C:\OpenHaptics\Developer\3.5.0\lib\x64\Debug
C:\OpenHaptics\Developer\3.5.0\utilities\lib\x64\Debug
```

说明：请根据本机安装目录调整路径和版本。

## 4. 配置 TouchControlPanelApp（Qt 前端）

### 4.1 Qt 模块

当前项目使用：

- core
- gui
- widgets
- openglwidgets

### 4.2 Include 路径

`C/C++ > Additional Include Directories`：

```text
$(MSBuildThisFileDirectory);..\shared;..\ControlBackend\include;.\src;.\src\widgets
```

### 4.3 链接 ControlBackend.lib

`Linker > Additional Library Directories`：

```text
$(SolutionDir)ControlBackend\$(Platform)\$(Configuration)\
```

`Linker > Additional Dependencies`：

```text
ControlBackend.lib
```

### 4.4 项目依赖

确保 `TouchControlPanelApp` 依赖 `ControlBackend`。

## 5. Qt 构建相关项

当前工程已启用 QtMsBuild，包含：

- `QtMoc`（例如 `DeviceController.h`、`StatusPanelWidget.h`）
- `QtUic`（`src/TouchControlPanelApp.ui`）
- `QtRcc`（`TouchControlPanelApp.qrc`）

## 6. 首次运行前检查

1. 设备驱动和 OpenHaptics 安装正常
2. 工程配置使用 x64
3. 后端能成功输出 `ControlBackend.lib`
4. 前端链接目录和依赖名匹配
5. Qt 编译器与 VS 工具链兼容
6. OpenGL 上下文可正常创建

## 7. 推荐调试顺序

1. 先单独编译 `ControlBackend`
2. 再编译 `TouchControlPanelApp`
3. 运行程序，先点“初始化设备”
4. 再点“启动实时采集”
5. 检查右侧日志、状态值与左侧 3D 是否联动

## 8. 常见问题

### 编译找不到 OpenHaptics 头文件

- 检查后端 Include 路径
- 检查路径是否按当前配置（Debug/Release、x64）生效

### 前端链接不到 ControlBackend.lib

- 检查 `Additional Library Directories`
- 检查库名是否为 `ControlBackend.lib`
- 检查项目依赖是否已设置

### 运行后设备初始化失败

- 检查设备连接与驱动
- 检查 OpenHaptics 控制面板是否识别设备
- 查看右侧消息日志里的错误信息
