cmake_minimum_required(VERSION 3.20)
project(QtVideoSmoke LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Network OpenGL OpenGLWidgets)
set(APP "${CMAKE_CURRENT_LIST_DIR}/../../TouchControlPanelApp/src")
add_executable(qt_video_smoke
    video_smoke.cpp
    "${APP}/RelayVideoClient.cpp" "${APP}/RelayVideoClient.h"
    "${APP}/widgets/VideoWidget.cpp"
    "${APP}/widgets/GLCoordinateWidget.cpp"
    "${APP}/widgets/ControlPanelWidget.cpp" "${APP}/widgets/ControlPanelWidget.h"
    "${APP}/widgets/StatusPanelWidget.cpp" "${APP}/widgets/StatusPanelWidget.h")
target_include_directories(qt_video_smoke PRIVATE "${APP}" "${APP}/widgets"
    "${CMAKE_CURRENT_LIST_DIR}/../../shared")
target_link_libraries(qt_video_smoke PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets
    Qt6::Network Qt6::OpenGL Qt6::OpenGLWidgets)

# Compile the actual control/window translation units with the real Qt API,
# without linking Windows Winsock/OpenHaptics backends or commanding hardware.
add_library(qt_control_compile OBJECT
    "${APP}/MainWindow.cpp" "${APP}/DeviceController.cpp"
    "${APP}/DeviceController.h" "${APP}/RelayStateClient.cpp"
    "${APP}/RelayStateClient.h" "${APP}/RelativeCalibrationDialog.cpp"
    "${APP}/RelativeCalibrationDialog.h" "${APP}/TeleopCalibrationDialog.cpp"
    "${APP}/TeleopCalibrationDialog.h")
target_include_directories(qt_control_compile PRIVATE "${APP}" "${APP}/widgets"
    "${CMAKE_CURRENT_LIST_DIR}/../../ControlBackend/include"
    "${CMAKE_CURRENT_LIST_DIR}/../../shared")
target_link_libraries(qt_control_compile PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets
    Qt6::Network Qt6::OpenGL Qt6::OpenGLWidgets)
add_executable(qt_control_thread_smoke control_thread_smoke.cpp
    "${APP}/RelayStateClient.cpp" "${APP}/RelayStateClient.h")
target_include_directories(qt_control_thread_smoke PRIVATE "${APP}"
    "${CMAKE_CURRENT_LIST_DIR}/../../shared")
target_link_libraries(qt_control_thread_smoke PRIVATE Qt6::Core Qt6::Network)
