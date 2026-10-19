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
        "需要在安全条件下采集至少 8 组、覆盖三维空间的 Touch/机器人对应点。\n"
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
    m_count=0;m_user=-1;m_tool=-1;m_epoch=0;m_touchCalibrationEpoch=0;
    m_samples.clear();
    m_solve->setEnabled(false);
    showStatus(QStringLiteral("尚未采样。至少 8 对，建议采集 12–20 对分散的空间点。"));
}
void TeleopCalibrationDialog::capture() {
    if(!m_touch||!m_robot)return;
    touchpanel::DeviceState touch;
    if(!m_touch->getTouchSnapshot(touch)||!touch.valid||!touch.orientationValid||
       !touch.schedulerRunning||!touch.deviceCalibrated||
       !touchpanel::teleop::finite(touch.positionMm)) {
        showStatus(QStringLiteral("Touch 数据不可用，无法采样"));return;
    }
    const auto robot=m_robot->snapshot();
    const auto gate=touchpanel::evaluateRobotGate(robot);
    if(!gate.canCaptureCalibration) {
        showStatus(QStringLiteral("机器人反馈过期或不可用，无法采样"));return;
    }
    if(m_count>0&&(robot.sessionEpoch!=m_epoch||robot.userIndex!=m_user||
                   robot.toolIndex!=m_tool||touch.calibrationEpoch!=m_touchCalibrationEpoch)) {
        resetSamples();
        showStatus(QStringLiteral("会话或用户/工具坐标已变更；请重新采样"));return;
    }
    if(m_count>=32)return;
    if(m_count==0) {m_epoch=robot.sessionEpoch;m_user=robot.userIndex;m_tool=robot.toolIndex;
        m_touchCalibrationEpoch=touch.calibrationEpoch;}
    m_samples.push_back({touch.positionMm,
        {robot.tcpPose[0],robot.tcpPose[1],robot.tcpPose[2]}});
    ++m_count;
    m_solve->setEnabled(m_count>=8);
    showStatus(QStringLiteral("已采集 %1/32 对（至少 8 对），User=%2 Tool=%3；请调整到下一个明显不同的位置。")
        .arg(m_count).arg(m_user).arg(m_tool));
}
void TeleopCalibrationDialog::solveAndSave() {
    if(m_count<8)return;
    const auto robot=m_robot->snapshot();
    if(!touchpanel::evaluateRobotGate(robot).canCaptureCalibration ||
       robot.sessionEpoch!=m_epoch || robot.userIndex!=m_user ||robot.toolIndex!=m_tool) {
        showStatus(QStringLiteral("机器人状态/坐标系已变化，拒绝保存"));return;
    }
    touchpanel::DeviceState touch;
    if(!m_touch->getTouchSnapshot(touch)||!touch.deviceCalibrated||
       touch.calibrationEpoch!=m_touchCalibrationEpoch) {
        showStatus(QStringLiteral("Touch 校准状态已改变，拒绝保存"));return;
    }
    const auto fit=touchpanel::teleop::fitSimilarity(m_samples);
    if(!fit.accepted()) {
        showStatus(QStringLiteral("多点配准拒绝：错误类型 %1，RMS=%2 mm，最大误差=%3 mm")
            .arg(static_cast<int>(fit.issue)).arg(fit.rmsMm,0,'f',2)
            .arg(fit.maxResidualMm,0,'f',2));return;
    }
    const auto calibration=fit.calibration;
    QJsonObject object{{"schema",2},{"user",m_user},{"tool",m_tool},
        {"verified_for_motion",false},{"scale",calibration.scale},
        {"rotation",numbers(calibration.touchToBase)},
        {"rms_mm",fit.rmsMm},{"max_residual_mm",fit.maxResidualMm},
        {"sample_count",m_count}};
    QSaveFile file(settingsPath());
    if(!file.open(QIODevice::WriteOnly)||
       file.write(QJsonDocument(object).toJson(QJsonDocument::Indented))<0||
       !file.commit()) {
        showStatus(QStringLiteral("写入标定配置失败"));return;
    }
    emit calibrationSaved(calibration,m_user,m_tool);
    showStatus(QStringLiteral("已保存多点映射（仅预览），scale=%1，RMS=%2 mm，最大误差=%3 mm。不是运动安全认证。")
        .arg(calibration.scale,0,'f',4).arg(fit.rmsMm,0,'f',2)
        .arg(fit.maxResidualMm,0,'f',2));
}
void TeleopCalibrationDialog::loadSaved() {
    QFile file(settingsPath());
    if(!file.open(QIODevice::ReadOnly))return;
    const auto doc=QJsonDocument::fromJson(file.readAll());
    const auto o=doc.object();
    if(o.value("schema").toInt()!=2||o.value("verified_for_motion").toBool(true))return;
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
