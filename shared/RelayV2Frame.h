#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace touchpanel::protocol {

// Compatible with relay_station TCP 8891: uint32 big-endian length followed
// by exactly that many UTF-8 JSON bytes. No newline or NUL delimiter.
class V2FrameCodec final {
public:
    static constexpr std::size_t maxBodyBytes = 1024 * 1024;
    static constexpr std::size_t maxBufferedBytes = 2 * maxBodyBytes + 8;

    bool feed(const char* data, std::size_t size, std::vector<std::string>& frames) {
        if (failed_ || (size != 0 && data == nullptr) ||
            size > maxBufferedBytes - buffer_.size()) {
            return fail(frames);
        }
        if (size) buffer_.append(data, size);
        while (buffer_.size() >= 4) {
            const auto* p = reinterpret_cast<const unsigned char*>(buffer_.data());
            const auto n = (std::uint32_t(p[0]) << 24) |
                           (std::uint32_t(p[1]) << 16) |
                           (std::uint32_t(p[2]) << 8) | std::uint32_t(p[3]);
            if (n == 0 || n > maxBodyBytes) return fail(frames);
            if (buffer_.size() < std::size_t(n) + 4) break;
            frames.emplace_back(buffer_.substr(4, n));
            buffer_.erase(0, std::size_t(n) + 4);
        }
        return true;
    }

    static std::string encode(std::string_view body) {
        if (body.empty() || body.size() > maxBodyBytes) return {};
        const auto n = static_cast<std::uint32_t>(body.size());
        std::string out;
        out.reserve(body.size() + 4);
        out.push_back(char((n >> 24) & 0xff));
        out.push_back(char((n >> 16) & 0xff));
        out.push_back(char((n >> 8) & 0xff));
        out.push_back(char(n & 0xff));
        out.append(body.data(), body.size());
        return out;
    }

    void reset() { buffer_.clear(); failed_ = false; }
    bool failed() const { return failed_; }
    std::size_t bufferedBytes() const { return buffer_.size(); }

private:
    bool fail(std::vector<std::string>& frames) {
        failed_ = true;
        buffer_.clear();
        frames.clear(); // discard a batch containing malformed frames
        return false;
    }
    std::string buffer_;
    bool failed_ = false;
};

class LegacyReplyFramer final {
public:
    static constexpr std::size_t maxBufferedBytes = 64 * 1024;

    // Dobot replies and relay errors end with ';', unlike client commands
    // (which arrive as "29999|Command(...)\n" at the relay).
    bool feed(const char* data, std::size_t size, std::vector<std::string>& replies) {
        if (failed_ || (size != 0 && data == nullptr) ||
            size > maxBufferedBytes - buffer_.size()) return fail(replies);
        if (size) buffer_.append(data, size);
        for (;;) {
            const std::size_t end = buffer_.find_first_of(";\r\n");
            if (end == std::string::npos) break;
            const char terminator = buffer_[end];
            std::string one = buffer_.substr(0, end + (terminator == ';' ? 1 : 0));
            buffer_.erase(0, end + 1);
            if (!one.empty()) replies.push_back(std::move(one));
        }
        return true;
    }

    void reset() { buffer_.clear(); failed_ = false; }
    bool failed() const { return failed_; }
private:
    bool fail(std::vector<std::string>& replies) {
        failed_ = true; buffer_.clear(); replies.clear(); return false;
    }
    std::string buffer_;
    bool failed_ = false;
};
} // namespace touchpanel::protocol
