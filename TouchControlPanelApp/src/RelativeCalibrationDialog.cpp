#include "RelativeCalibrationDialog.h"
#include "DeviceController.h"
#include "RelayStateClient.h"
#include <QCheckBox>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QFileInfo>
#include <QPushButton>
#include <QSaveFile>
#include <QStandardPaths>
#include <QVBoxLayout>
#include <cmath>

namespace {
using namespace touchpanel;
using namespace touchpanel::teleop;
QJsonArray array3(const Vec3& v) {
    QJsonArray a;for(double x:v)a.append(x);return a;
}
QJsonArray array9(const Mat3& v) {
    QJsonArray a;for(double x:v)a.append(x);return a;
}
QString outputPath() {
    const QString dir=QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    return dir+QStringLiteral("/touch_robot_relative_calibration_v1.json");
}
QString qualityMessage(const RelativeMotionFit& f) {
    return QStringLiteral("训练 RMS (无平移) %1 mm，最大 %2 mm；独立验证 RMS %3 mm，最大 %4 mm；比例 %5")
        .arg(f.zeroOffsetRmsMm,0,'f',2).arg(f.zeroOffsetMaxMm,0,'f',2)
        .arg(f.holdoutRmsMm,0,'f',2).arg(f.holdoutMaxMm,0,'f',2)
        .arg(f.calibration.scale,0,'f',4);
}
}
RelativeCalibrationDialog::RelativeCalibrationDialog(DeviceController* touch,
                                                    RelayStateClient* robot,
                                                    QWidget* parent)
    :QDialog(parent),m_touch(touch),m_robot(robot) {
    setWindowTitle(QStringLiteral("相对位移与方向标定 · 只读向导"));
    setMinimumWidth(520);
    auto* root=new QVBoxLayout(this);
    m_instructions=new QLabel(QStringLiteral(
        "用于互不重叠的工作空间：每组先记录两台设备的起点，再通过外部安全示教移动机器人、"
        "手动移动 Touch，使两段位移代表同一预先约定的空间方向与比例，最后记录终点。\n"
        "建议至少 10 组，包含 X/Y/Z 正反方向和空间斜向，避免共面；最后两组自动作为独立验证。\n"
        "本窗口不调用任何机器人运动命令。位置变化必须依据共同方向基准事先对应，"
        "不能把任意位移凑在一起。姿态标定另行进行。"),this);
    m_instructions->setWordWrap(true);
    root->addWidget(m_instructions);
    m_progress=new QLabel(this);
    m_progress->setWordWrap(true);root->addWidget(m_progress);
    m_status=new QLabel(this);
    m_status->setWordWrap(true);
    m_status->setMinimumHeight(80);
    root->addWidget(m_status);
    m_confirm=new QCheckBox(QStringLiteral("确认本组两个设备的位移按预定方向与比例对应，并已确认机器人静止"),this);
    root->addWidget(m_confirm);
    m_start=new QPushButton(QStringLiteral("① 捕获两台设备起点（静止采样）"),this);
    m_end=new QPushButton(QStringLiteral("② 捕获两台设备终点并记录位移"),this);
    m_fit=new QPushButton(QStringLiteral("③ 拟合 + 独立验证 + 保存（仅预览）"),this);
    m_reset=new QPushButton(QStringLiteral("清空本次全部样本"),this);
    root->addWidget(m_start);root->addWidget(m_end);root->addWidget(m_fit);root->addWidget(m_reset);
    m_timer.setInterval(45);
    m_timer.setTimerType(Qt::PreciseTimer);
    connect(&m_timer,&QTimer::timeout,this,&RelativeCalibrationDialog::captureTick);
    connect(m_start,&QPushButton::clicked,this,&RelativeCalibrationDialog::captureStart);
    connect(m_end,&QPushButton::clicked,this,&RelativeCalibrationDialog::captureEnd);
    connect(m_fit,&QPushButton::clicked,this,&RelativeCalibrationDialog::fitAndSave);
    connect(m_reset,&QPushButton::clicked,this,&RelativeCalibrationDialog::resetSamples);
    connect(m_confirm,&QCheckBox::toggled,this,[this]{updateControls();});
    resetSamples();
}
void RelativeCalibrationDialog::describe(const QString& message) {
    m_status->setText(message);
}
double RelativeCalibrationDialog::angularDifference(double a,double b) {
    return std::abs(std::remainder(a-b,360.0));
}
void RelativeCalibrationDialog::updateControls() {
    const int n=static_cast<int>(m_gestures.size());
    m_progress->setText(QStringLiteral("有效位移：%1 / 32 组（至少 10 组，最后 2 组留作验证） · %2")
        .arg(n).arg(m_haveStart?QStringLiteral("起点已锁定，等待终点"):
                                      QStringLiteral("等待新起点")));
    m_start->setEnabled(!m_capturing&&!m_haveStart&&n<32);
    m_end->setEnabled(!m_capturing&&m_haveStart&&m_confirm->isChecked());
    m_fit->setEnabled(!m_capturing&&!m_haveStart&&n>=10);
    m_reset->setEnabled(!m_capturing);
}
void RelativeCalibrationDialog::resetSamples() {
    if(m_capturing)return;
    m_timer.stop();
    m_window.clear();m_gestures.clear();
    m_haveStart=false;m_sessionEpoch=0;m_touchEpoch=0;
    m_user=-1;m_tool=-1;
    m_previousStateSeq=0;m_previousTouchCounter=0;
    m_confirm->setChecked(false);
    describe(QStringLiteral("尚无样本。请先确定两个工作空间的共同方向基准，在两端分别摆放可重复的起点。"));
    updateControls();
}
bool RelativeCalibrationDialog::matchingSession(const RobotSnapshot& robot,
                                                 const DeviceState& touch) const {
    return m_sessionEpoch==0 || (
        robot.sessionEpoch==m_sessionEpoch && robot.userIndex==m_user &&
        robot.toolIndex==m_tool && touch.calibrationEpoch==m_touchEpoch
    );
}
bool RelativeCalibrationDialog::readValidPair(Pair& pair,RobotSnapshot& robot,
                                               DeviceState& touch) const {
    if(!m_touch||!m_robot||!m_touch->getTouchSnapshot(touch)||
       !touch.valid||!touch.schedulerRunning||!touch.deviceCalibrated||
       !touch.orientationValid||!finite(touch.positionMm))return false;
    robot=m_robot->snapshot();
    if(!evaluateRobotGate(robot).canCaptureCalibration||
       !robotStationaryForCapture(robot.jointVelocity,robot.tcpVelocity)||
       robot.running||robot.jogging||robot.paused||robot.error||!matchingSession(robot,touch))
        return false;
    pair={touch.positionMm,{robot.tcpPose[0],robot.tcpPose[1],robot.tcpPose[2]}};
    return true;
}
void RelativeCalibrationDialog::captureStart(){beginCapture(Phase::Start);}
void RelativeCalibrationDialog::captureEnd(){beginCapture(Phase::End);}
void RelativeCalibrationDialog::beginCapture(Phase phase) {
    if(m_capturing||(phase==Phase::End&&!m_haveStart)||
       (phase==Phase::Start&&m_haveStart))return;
    if(phase==Phase::End&&!m_confirm->isChecked()) {
        describe(QStringLiteral("需要先确认本组对应关系与静止状态"));return;
    }
    m_phase=phase;m_capturing=true;
    m_window.clear();m_previousStateSeq=0;m_previousTouchCounter=0;
    describe(QStringLiteral("采集静止窗口中：请勿移动 Touch 或机器人…"));
    updateControls();m_timer.start();
}
void RelativeCalibrationDialog::captureTick() {
    Pair pair;RobotSnapshot robot;DeviceState touch;
    if(!readValidPair(pair,robot,touch) ||
       (m_previousStateSeq!=0 && robot.stateSeq<=m_previousStateSeq) ||
       (m_previousTouchCounter!=0 && touch.sampleCounter<=m_previousTouchCounter)) {
        m_timer.stop();m_capturing=false;m_window.clear();
        describe(QStringLiteral("采样中断：设备未校准、机器人未静止、遥测不新鲜、会话变化或采样未更新。请检查状态后重试。"));
        updateControls();return;
    }
    m_previousStateSeq=robot.stateSeq;
    m_previousTouchCounter=touch.sampleCounter;
    m_window.push_back(pair);
    if(m_window.size()<8)return;
    m_timer.stop();m_capturing=false;
    if(!captureWindowStable(m_window)) {
        describe(QStringLiteral("采样窗口抖动超限（Touch >1.5 mm 或 TCP >2 mm），未保存该端点。请保持静止重试。"));
        m_window.clear();updateControls();return;
    }
    const Pair average=meanCapture(m_window);
    m_window.clear();
    if(m_phase==Phase::Start) {
        if(m_sessionEpoch==0) {
            m_sessionEpoch=robot.sessionEpoch;m_touchEpoch=touch.calibrationEpoch;
            m_user=robot.userIndex;m_tool=robot.toolIndex;
        }
        m_origin=average;
        m_robotOrientation={robot.tcpPose[3],robot.tcpPose[4],robot.tcpPose[5]};
        m_haveStart=true;
        describe(QStringLiteral("起点已记录。请用独立安全示教方式调整机器人至终点，再按预定对应方向移动 Touch，均静止后勾选确认并捕获终点。"));
    } else {
        for(int i=0;i<3;++i) {
            if(angularDifference(robot.tcpPose[i+3],m_robotOrientation[i])>8.0) {
                describe(QStringLiteral("终点拒绝：机器人 TCP 姿态变化超过 8°，本向导只拟合平移映射。可调整回原姿态后重试。"));
                updateControls();return;
            }
        }
        const Pair delta={sub(average.touch,m_origin.touch),
                          sub(average.robot,m_origin.robot)};
        if(norm(delta.touch)<15||norm(delta.robot)<15) {
            describe(QStringLiteral("终点拒绝：任一设备位移小于 15 mm，增加移动距离后重试。"));
            updateControls();return;
        }
        if(!independentPoint(m_gestures,delta,8.0,8.0)) {
            describe(QStringLiteral("终点拒绝：位移方向与长度与已有样本过于接近，改用反向或斜向手势。"));
            updateControls();return;
        }
        m_gestures.push_back(delta);
        m_haveStart=false;
        m_confirm->setChecked(false);
        describe(QStringLiteral("已记录第 %1 组：Touch Δ(%2,%3,%4)，Robot Δ(%5,%6,%7) mm。重新摆放起点继续。")
            .arg(m_gestures.size())
            .arg(delta.touch[0],0,'f',1).arg(delta.touch[1],0,'f',1).arg(delta.touch[2],0,'f',1)
            .arg(delta.robot[0],0,'f',1).arg(delta.robot[1],0,'f',1).arg(delta.robot[2],0,'f',1));
    }
    updateControls();
}
void RelativeCalibrationDialog::fitAndSave() {
    if(m_capturing||m_haveStart||m_gestures.size()<10||!m_touch||!m_robot)return;
    Pair pair;RobotSnapshot robot;DeviceState touch;
    if(!readValidPair(pair,robot,touch)) {
        describe(QStringLiteral("拒绝保存：Touch 校准、机器人会话或实时状态不再有效。"));return;
    }
    const RelativeMotionFit fit=fitRelativeDisplacements(m_gestures);
    if(!fit.accepted) {
        describe(QStringLiteral("相对配准未通过，禁止应用。%1。请核查每组是否真的有物理对应关系，或清空重采。")
            .arg(qualityMessage(fit)));
        return;
    }
    QJsonArray gestures;
    for(const auto& p:m_gestures)
        gestures.append(QJsonObject{{"touch_delta_mm",array3(p.touch)},
                                    {"robot_delta_mm",array3(p.robot)}});
    const QJsonObject document{
        {"schema",1},{"mode",QStringLiteral("relative_displacement")},
        {"verified_for_motion",false},{"orientation_verified",false},
        {"user",m_user},{"tool",m_tool},
        {"scale",fit.calibration.scale},
        {"rotation",array9(fit.calibration.touchToBase)},
        {"training_zero_offset_rms_mm",fit.zeroOffsetRmsMm},
        {"training_zero_offset_max_mm",fit.zeroOffsetMaxMm},
        {"holdout_rms_mm",fit.holdoutRmsMm},
        {"holdout_max_mm",fit.holdoutMaxMm},
        {"gesture_count",static_cast<int>(m_gestures.size())},{"gestures",gestures}
    };
    const QString path=outputPath();
    const QFileInfo fileInfo(path);
    if(!QDir().mkpath(fileInfo.absolutePath())) {
        describe(QStringLiteral("无法创建私有标定配置目录"));return;
    }
    QSaveFile file(path);
    if(!file.open(QIODevice::WriteOnly) ||
       file.write(QJsonDocument(document).toJson(QJsonDocument::Indented))<0 ||
       !file.commit()) {
        describe(QStringLiteral("保存相对标定失败；结果未应用"));return;
    }
    emit calibrationSaved(fit.calibration,m_user,m_tool);
    describe(QStringLiteral("平移相对标定已保存并应用至本次只读预览。%1。姿态尚未验证、实体运动始终禁用。")
        .arg(qualityMessage(fit)));
}
