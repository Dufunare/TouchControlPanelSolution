#pragma once
#include <QJsonObject>
#include <QString>
#include <chrono>
#include <cmath>

// GUI-only observation policy; never authorizes motion. All timing is local
// steady time plus the relay-reported age, including cross-thread queue delay.
class CameraWarningState {
public:
    static qint64 nowMs() {return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();}
    static double checkedThreshold(double mm) {
        return std::isfinite(mm) && mm>=1 && mm<=500 ? mm : 30;
    }
    double threshold() const {return m_threshold;}
    void setThreshold(double mm,qint64 now) {
        m_threshold=checkedThreshold(mm);
        m_warningUntil=0; // Manual correction must take effect immediately.
        if(fresh(now)&&m_gap<=m_threshold)m_warningUntil=now+2000;
    }
    void receive(const QJsonObject& observation,qint64 receivedAt,qint64 now) {
        m_valid=false;
        const auto gap=observation.value("gap_mm"),age=observation.value("age_ms");
        if(observation.value("valid")!=true || !gap.isDouble() || !age.isDouble() ||
           !std::isfinite(gap.toDouble()) || !std::isfinite(age.toDouble()) ||
           age.toDouble()<0 || age.toDouble()>250 || receivedAt>now ||
           now-receivedAt>250)return;
        // Age is rounded UP so conversion cannot make old data younger.
        m_sampleAt=receivedAt-static_cast<qint64>(std::ceil(age.toDouble()));
        m_gap=gap.toDouble();m_valid=true;
        if(fresh(now)&&m_gap<=m_threshold)m_warningUntil=now+2000;
    }
    bool fresh(qint64 now) const {return m_valid && now>=m_sampleAt && now-m_sampleAt<=250;}
    bool warning(qint64 now) const {return (fresh(now)&&m_gap<=m_threshold)||now<m_warningUntil;}
    QString text(qint64 now) const {
        const QString distance=fresh(now)?QStringLiteral("估计间隙 %1 mm").arg(m_gap,0,'f',1):QStringLiteral("距离未知 / 观测已失效");
        const QString state=warning(now)?(fresh(now)&&m_gap<=m_threshold?QStringLiteral("距离过近"):
            QStringLiteral("近期过近提示")):QStringLiteral("仅观察");
        return QStringLiteral("%1 · %2\n警示阈值 %3 mm · 不触发自动运动")
            .arg(state,distance).arg(m_threshold,0,'f',1);
    }
private:
    double m_threshold=30,m_gap=0;
    bool m_valid=false;
    qint64 m_sampleAt=0,m_warningUntil=0;
};
