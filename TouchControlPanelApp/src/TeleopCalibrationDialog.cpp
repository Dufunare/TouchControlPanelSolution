#include "TeleopCalibrationDialog.h"
#include "DeviceController.h"
#include "RelayStateClient.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QFile>
#include <QStandardPaths>
#include <QDir>

namespace {
QString settingsPath() {
    const QString dir=QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    QDir().mkpath(dir);
    return dir+QStringLiteral("/touch_robot_calibration_v1.json");
}
QJsonArray numbers(const touchpanel::teleop::Mat3& m) {
    QJsonArray a;for(double d:m)a.append(d);return a;
}
}

TeleopCalibrationDialog::TeleopCalibrationDialog(DeviceController* touch,RelayStateClient* robot,QWidget* parent)
    :QDialog(parent),m_touch(touch),m_robot(robot) {
    setWindowTitle(QStringLiteral("Touch/机械臂空间标定（仅采样）"));
    auto* layout=new QVBoxLayout(this);
    m_status=new QLabel(this);m_status->setWordWrap(true);
    layout->addWidget(new QLabel(QStringLiteral(
        "需要在安全条件下分别采集 3 组 Touch/机器人对应点。\n"
        "本界面只读取当前位置，不会控制机械臂移动。"
        "机器人用户/工具坐标系不可在标定过程中改变。"),this));
    layout->addWidget(m_status);
    m_capture=new QPushButton(QStringLiteral("捕获当前对应点"),this);
    m_solve=new QPushButton(QStringLiteral("计算并保存（只读预览）"),this);
    m_solve->setEnabled(false);
    layout->addWidget(m_capture);layout->addWidget(m_solve);
    auto* reset=new QPushButton(QStringLiteral("重新采样"),this);layout->addWidget(reset);
    connect(m_capture,&QPushButton::clicked,this,&TeleopCalibrationDialog::capture);
    connect(m_solve,&QPushButton::clicked,this,&TeleopCalibrationDialog::solveAndSave);
    connect(reset,&QPushButton::clicked,this,&TeleopCalibrationDialog::resetSamples);
    resetSamples();
    loadSaved();
}
void TeleopCalibrationDialog::showStatus(const QString& info) {m_status->setText(info);}
void TeleopCalibrationDialog::resetSamples() {
    m_count=0;m_user=-1;m_tool=-1;m_epoch=0;
    m_solve->setEnabled(false);
    showStatus(QStringLiteral("尚未采样。需采集三对不共线的对应点。"));
}
void TeleopCalibrationDialog::capture() {
    if(!m_touch||!m_robot)return;
    touchpanel::DeviceState touch;
    if(!m_touch->getTouchSnapshot(touch)||!touch.valid||!touch.orientationValid||
       !touch.schedulerRunning||!touchpanel::teleop::finite(touch.positionMm)) {
        showStatus(QStringLiteral("Touch 数据不可用，无法采样"));return;
    }
    const auto robot=m_robot->snapshot();
    const auto gate=touchpanel::evaluateRobotGate(robot);
    if(!gate.canCaptureCalibration) {
        showStatus(QStringLiteral("机器人反馈过期或不可用，无法采样"));return;
    }
    if(m_count>0&&(robot.sessionEpoch!=m_epoch||robot.userIndex!=m_user||
                   robot.toolIndex!=m_tool)) {
        resetSamples();
        showStatus(QStringLiteral("会话或用户/工具坐标已变更；请重新采样"));return;
    }
    if(m_count>=3)return;
    if(m_count==0) {m_epoch=robot.sessionEpoch;m_user=robot.userIndex;m_tool=robot.toolIndex;}
    m_samples[m_count++]={touch.positionMm,
        {robot.tcpPose[0],robot.tcpPose[1],robot.tcpPose[2]}};
    m_solve->setEnabled(m_count==3);
    showStatus(QStringLiteral("已采集 %1/3 对，User=%2 Tool=%3；请调整到下一个明显不同的位置。")
        .arg(m_count).arg(m_user).arg(m_tool));
}
void TeleopCalibrationDialog::solveAndSave() {
    if(m_count!=3)return;
    const auto robot=m_robot->snapshot();
    if(!touchpanel::evaluateRobotGate(robot).canCaptureCalibration ||
       robot.sessionEpoch!=m_epoch || robot.userIndex!=m_user ||robot.toolIndex!=m_tool) {
        showStatus(QStringLiteral("机器人状态/坐标系已变化，拒绝保存"));return;
    }
    touchpanel::teleop::Calibration calibration;
    if(!touchpanel::teleop::calibrateThreePairs(m_samples,calibration)) {
        showStatus(QStringLiteral("标定失败：三点共线、位移过小或尺度不一致"));return;
    }
    QJsonObject object{{"schema",1},{"user",m_user},{"tool",m_tool},
        {"verified_for_motion",false},{"scale",calibration.scale},
        {"rotation",numbers(calibration.touchToBase)}};
    QSaveFile file(settingsPath());
    if(!file.open(QIODevice::WriteOnly)||
       file.write(QJsonDocument(object).toJson(QJsonDocument::Indented))<0||
       !file.commit()) {
        showStatus(QStringLiteral("写入标定配置失败"));return;
    }
    emit calibrationSaved(calibration,m_user,m_tool);
    showStatus(QStringLiteral("已保存三点映射（仅预览），scale=%1。此标定不是机器人运动安全认证。")
        .arg(calibration.scale,0,'f',4));
}
void TeleopCalibrationDialog::loadSaved() {
    QFile file(settingsPath());
    if(!file.open(QIODevice::ReadOnly))return;
    const auto doc=QJsonDocument::fromJson(file.readAll());
    const auto o=doc.object();
    if(o.value("schema").toInt()!=1||o.value("verified_for_motion").toBool(true))return;
    const auto matrix=o.value("rotation").toArray();
    if(matrix.size()!=9)return;
    touchpanel::teleop::Calibration c;
    for(int i=0;i<9;++i) {
        if(!matrix[i].isDouble())return;
        c.touchToBase[i]=matrix[i].toDouble();
    }
    c.scale=o.value("scale").toDouble(0);
    c.measured=true;
    if(!touchpanel::teleop::properRotation(c.touchToBase)||
       c.scale<0.05||c.scale>20.0)return;
    showStatus(QStringLiteral("发现历史标定配置（仅预览）。重新采样后可更新；不会自动授权运动。"));
}
