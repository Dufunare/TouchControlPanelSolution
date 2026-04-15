// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.
#include "pch.h"
#include "dobot/BitConverter.h"

#include <cstring>

namespace Dobot
{
    short CBitConverter::ToShort(char* pBuffer)
    {
        short value = pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    unsigned short CBitConverter::ToUShort(char* pBuffer)
    {
        unsigned short value = pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    int CBitConverter::ToInt(char* pBuffer)
    {
        int value = pBuffer[3] & 0xFF;
        value <<= 8;
        value |= pBuffer[2] & 0xFF;
        value <<= 8;
        value |= pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    unsigned int CBitConverter::ToUInt(char* pBuffer)
    {
        unsigned int value = pBuffer[3] & 0xFF;
        value <<= 8;
        value |= pBuffer[2] & 0xFF;
        value <<= 8;
        value |= pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    long CBitConverter::ToLong(char* pBuffer)
    {
        long value = pBuffer[3] & 0xFF;
        value <<= 8;
        value |= pBuffer[2] & 0xFF;
        value <<= 8;
        value |= pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    unsigned long CBitConverter::ToULong(char* pBuffer)
    {
        unsigned long value = pBuffer[3] & 0xFF;
        value <<= 8;
        value |= pBuffer[2] & 0xFF;
        value <<= 8;
        value |= pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    int64_t CBitConverter::ToInt64(char* pBuffer)
    {
        int64_t value = pBuffer[7] & 0xFF;
        value <<= 8;
        value |= pBuffer[6] & 0xFF;
        value <<= 8;
        value |= pBuffer[5] & 0xFF;
        value <<= 8;
        value |= pBuffer[4] & 0xFF;
        value <<= 8;
        value |= pBuffer[3] & 0xFF;
        value <<= 8;
        value |= pBuffer[2] & 0xFF;
        value <<= 8;
        value |= pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    uint64_t CBitConverter::ToUInt64(char* pBuffer)
    {
        uint64_t value = pBuffer[7] & 0xFF;
        value <<= 8;
        value |= pBuffer[6] & 0xFF;
        value <<= 8;
        value |= pBuffer[5] & 0xFF;
        value <<= 8;
        value |= pBuffer[4] & 0xFF;
        value <<= 8;
        value |= pBuffer[3] & 0xFF;
        value <<= 8;
        value |= pBuffer[2] & 0xFF;
        value <<= 8;
        value |= pBuffer[1] & 0xFF;
        value <<= 8;
        value |= pBuffer[0] & 0xFF;
        return value;
    }

    float CBitConverter::ToFloat(char* pBuffer)
    {
        float value = 0.0f;
        std::memcpy(&value, pBuffer, sizeof(value));
        return value;
    }

    double CBitConverter::ToDouble(char* pBuffer)
    {
        double value = 0.0;
        std::memcpy(&value, pBuffer, sizeof(value));
        return value;
    }
} // namespace Dobot
