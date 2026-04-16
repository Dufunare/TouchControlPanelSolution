#include "TransitProtocolParser.h"

QString TransitProtocolParser::pack(quint16 robotPort, const QString& command) const
{
	return QStringLiteral("%1|%2").arg(robotPort).arg(command);
}

TransitParsedFrame TransitProtocolParser::parse(const QString& rawLine) const
{
	TransitParsedFrame frame;
	frame.raw = rawLine;

	const int separatorIndex = rawLine.indexOf('|');
	if (separatorIndex <= 0)
	{
		frame.payload = rawLine;
		return frame;
	}

	bool ok = false;
	const quint16 parsedPort = rawLine.left(separatorIndex).toUShort(&ok);
	if (ok)
	{
		frame.hasPort = true;
		frame.port = parsedPort;
	}

	frame.payload = rawLine.mid(separatorIndex + 1);
	return frame;
}
