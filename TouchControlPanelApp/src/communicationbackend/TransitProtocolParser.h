#pragma once

#include <QString>
#include <QtGlobal>

struct TransitParsedFrame
{
	bool hasPort = false;
	quint16 port = 0;
	QString payload;
	QString raw;
};

class TransitProtocolParser
{
public:
	QString pack(quint16 robotPort, const QString& command) const;
	TransitParsedFrame parse(const QString& rawLine) const;
};
