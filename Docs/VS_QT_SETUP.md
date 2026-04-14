# 从零开始：Visual Studio + Qt VS Tools + 静态库 + 这套框架怎么落地

**仅供参考，许多配置方法不是很详细和很适用**

下面假设是第一次搭这套工程

---

## 一、先准备软件环境

### 1. 安装 Visual Studio

推荐：

- Visual Studio 2022
- 勾选“使用 C++ 的桌面开发”

### 2. 安装 Qt

推荐安装：

- Qt 6.x
- **MSVC 2022 64-bit** 版本

### 3. 安装 Qt VS Tools

在 Visual Studio 里：

1. 打开 **Extensions > Manage Extensions > Online**
2. 搜索 **Qt Visual Studio Tools**
3. 点击下载
4. 重启 Visual Studio

### 4. 在 Qt VS Tools 里添加 Qt 版本

在 Visual Studio 里：

1. 打开 **Extensions > Qt VS Tools > Qt Versions**
2. 点击 **Add**
3. 把 Qt 的 `qmake.exe` 路径填进去  
   例如：`C:\Qt\6.8.0\msvc2022_64\bin\qmake.exe`
4. 保存

> 如果 Qt 是用官方在线安装器装的，也可以先试试 Autodetect。

---

## 二、创建解决方案

推荐解决方案名：

```text
TouchControlPanelSolution
```

---

## 三、先创建后端静态库项目

这一步很重要。  
你的后端不要做成 Qt 项目，而是做成**纯 C++ 静态库**。

### 1. 新建项目

在 VS 里：

1. **File > New > Project**
2. 选择 **Static Library**（静态库）
3. 项目名填：`TouchBackendLib`

### 2. 给后端项目加文件

把下面文件加进去：

- `shared/DeviceState.h`
- `TouchBackendLib/include/TouchBackend.h`
- `TouchBackendLib/src/TouchBackend.cpp`

### 3. 配置后端项目的包含目录

右键 `TouchBackendLib` 项目 -> **Properties**

#### C/C++ > General > Additional Include Directories

加入：

- OpenHaptics SDK `include` 目录
- `..\shared`
- `.\include`

示例写法（需要改成自己的真实路径）：

```text
C:\YourOpenHapticsPath\include;..\shared;.\include;%(AdditionalIncludeDirectories)
```

### 4. 配置后端项目的库目录

#### Linker > General > Additional Library Directories

加入 OpenHaptics 的 `lib` 目录，例如：

```text
C:\YourOpenHapticsPath\lib;%(AdditionalLibraryDirectories)
```

### 5. 配置后端项目要链接的 OpenHaptics 库

#### Linker > Input > Additional Dependencies

加入实际安装版本所需的 OpenHaptics `.lib`。

> 这一步请以本机安装目录和官方示例工程为准。  
> 因为不同安装包、不同版本、是否使用 HLAPI / HDAPI，都可能影响具体库名。  
> 本框架第一阶段只依赖 **HDAPI** 的坐标读取能力。

### 6. 编译后端

成功后，你会得到：

```text
TouchBackendLib.lib
```

这个 `.lib` 就是给前端 Qt 项目调用的。

---

## 四、创建前端 Qt Widgets 项目

### 1. 新建项目

在 VS 里：

1. **File > New > Project**
2. 选择 **Qt Widgets Application**
3. 项目名填：`TouchControlPanelApp`

### 2. 选 Qt 版本

创建向导里选择刚才配置的 Qt 版本，比如：

- Qt 6.x / msvc2022_64

### 3. 给前端项目加文件

把下面文件加进去：

- `shared/DeviceState.h`
- `TouchControlPanelApp/src/main.cpp`
- `TouchControlPanelApp/src/DeviceStateQt.h`
- `TouchControlPanelApp/src/DeviceController.h`
- `TouchControlPanelApp/src/DeviceController.cpp`
- `TouchControlPanelApp/src/MainWindow.h`
- `TouchControlPanelApp/src/MainWindow.cpp`
- `TouchControlPanelApp/src/widgets/GLCoordinateWidget.h`
- `TouchControlPanelApp/src/widgets/GLCoordinateWidget.cpp`
- `TouchControlPanelApp/src/widgets/StatusPanelWidget.h`
- `TouchControlPanelApp/src/widgets/StatusPanelWidget.cpp`

### 4. 配置前端项目的包含目录

右键 `TouchControlPanelApp` -> **Properties**

#### C/C++ > General > Additional Include Directories

加入：

- `..\shared`
- `..\TouchBackendLib\include`
- `.\src`
- `.\src\widgets`

例如：

```text
..\shared;..\TouchBackendLib\include;.\src;.\src\widgets;%(AdditionalIncludeDirectories)
```

### 5. 配置前端项目链接后端静态库

#### Linker > Input > Additional Dependencies

加入：

```text
TouchBackendLib.lib;%(AdditionalDependencies)
```

#### Linker > General > Additional Library Directories

指向 `TouchBackendLib.lib` 生成目录。  
例如（Debug x64 只是例子）：

```text
..\x64\Debug;%(AdditionalLibraryDirectories)
```

也可以直接填后端项目的输出目录。

### 6. 设置项目依赖

右键 **Solution** -> **Project Dependencies**

勾选：

- `TouchControlPanelApp` 依赖 `TouchBackendLib`

这样每次编译前端时，VS 会先编译后端。

---

## 五、Qt 项目里要确认的几个点

### 1. Qt/MSBuild 已启用

Qt VS Tools 会自动接管：

- `moc`
- `uic`
- `rcc`

一般不需要手动写这些构建步骤。

### 2. Qt 模块要够用

这套代码至少会用到：

- Core
- Gui
- Widgets
- OpenGLWidgets

如果模板默认没有把 `OpenGLWidgets` 模块配好，记得在项目设置里补上。

### 3. 如果以后想用 Qt Designer

可以：

- **Project > Add New Item > Installed > Visual C++ > Qt > Qt Widgets Form File**
- 或者直接启动 **Extensions > Qt VS Tools > Launch Qt Widgets Designer**

当前这版代码为了便于理解，**主窗口先用纯 C++ 手写**。  
等熟悉后，再把一部分区域换成 `.ui` 文件也完全可以。

---

## 六、第一次运行前的检查

1. 设备驱动是否安装正常
2. Touch 设备是否被系统识别
3. OpenHaptics SDK 路径是否配对
4. Debug/Release、x64 配置是否一致
5. Qt 使用的编译器是否与 VS 一致
6. 前端是否已经正确链接 `TouchBackendLib.lib`
7. 显卡是否能创建 OpenGL 上下文

---

## 七、推荐的调试顺序

### 第一步：先只编译后端

确认 `TouchBackendLib` 不报头文件 / 库路径错误。

### 第二步：再编译前端

确认 Qt 项目能找到 `TouchBackend.h` 和 `DeviceState.h`。

### 第三步：先让界面跑起来

即使设备还没配好，也先确认窗口能打开、OpenGL 区域能显示。

### 第四步：点击“初始化设备”

看右侧消息区是否显示成功 / 失败信息。

### 第五步：点击“启动实时采集”

看右侧 XYZ 数值是否变化，左侧点是否移动。

---

## 八、静态库到底是什么，为什么这里必须这么做

可以这样理解：

- `TouchBackendLib` 是“后端积木包”
- `TouchControlPanelApp` 是“把积木摆到界面上的程序”

静态库的好处：

1. 前端不直接碰 OpenHaptics 细节
2. 后端改了，前端界面代码不一定要跟着大改
3. 以后你可以单独测试后端
4. 以后你可以把不同前端（Qt / 控制台 / 测试程序）都连到同一个后端库

这就是“解耦”。

---

## 九、以后真的要用 Qt Designer 时怎么做

当前版本的主窗口是手写的，原因是：

- 你更容易看懂代码流向
- 你更容易理解布局和信号槽
- 你不需要一上来就理解 `.ui` 自动生成代码

但以后完全可以这样做：

### 方法 1：给某个子区域单独做 `.ui`

例如做一个 `VideoViewerWidget.ui`：

- 右键项目 -> Add New Item
- 选择 `Qt Widget Form File`
- Qt VS Tools 会帮你生成 `.ui + .h + .cpp`

这很适合做：

- 状态面板
- 参数面板
- 对话框
- 日志面板

### 方法 2：主窗口继续手写，子控件用 Designer

也就是：

- `MainWindow.cpp` 继续负责拼装和布局
- 某些复杂的右侧面板，用 `.ui` 设计
- 自定义 OpenGL 区域依然手写

这样既直观，又不容易失控。
