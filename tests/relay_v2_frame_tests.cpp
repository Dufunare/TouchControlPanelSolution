#include "../shared/RelayV2Frame.h"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using touchpanel::protocol::V2FrameCodec;
using touchpanel::protocol::LegacyReplyFramer;

int main() {
    V2FrameCodec parser;
    std::vector<std::string> frames;
    const auto a = V2FrameCodec::encode(R"({"v":2,"type":"hello_ack"})");
    const auto b = V2FrameCodec::encode(R"({"v":2,"type":"robot_state"})");
    assert(!a.empty() && !b.empty());
    for (char c: a) {
        assert(parser.feed(&c, 1, frames));
    }
    assert(frames.size() == 1 && frames.front() == R"({"v":2,"type":"hello_ack"})");
    frames.clear();
    const auto glued = b + a;
    assert(parser.feed(glued.data(), glued.size(), frames));
    assert(frames.size() == 2 &&
           frames[0] == R"({"v":2,"type":"robot_state"})" &&
           frames[1] == R"({"v":2,"type":"hello_ack"})");
    frames.clear();
    const std::string zero(4,'\0');
    assert(!parser.feed(zero.data(), zero.size(), frames));
    assert(parser.failed());
    assert(!parser.feed(a.data(),a.size(),frames));
    parser.reset();
    const std::string tooLarge("\x00\x10\x00\x01",4);
    assert(!parser.feed(tooLarge.data(),tooLarge.size(),frames));
    parser.reset();
    assert(V2FrameCodec::encode("").empty());

    LegacyReplyFramer legacy;
    std::vector<std::string> replies;
    assert(legacy.feed("0,{5},Robot",11,replies) && replies.empty());
    const std::string tail="Mode();0,{1,2,3,4,5,6},GetPose();";
    assert(legacy.feed(tail.data(),tail.size(),replies));
    assert(replies.size()==2 && replies[0]=="0,{5},RobotMode();" &&
           replies[1]=="0,{1,2,3,4,5,6},GetPose();");
    replies.clear();
    assert(legacy.feed("ERROR|robot_reply_timeout|x;",sizeof("ERROR|robot_reply_timeout|x;")-1,replies));
    assert(replies.size()==1 && replies[0]=="ERROR|robot_reply_timeout|x;");
    replies.clear();
    legacy.reset();
    assert(legacy.feed("0,{5},RobotMode();\r\n",sizeof("0,{5},RobotMode();\r\n")-1,replies));
    assert(replies.size()==1 && replies[0]=="0,{5},RobotMode();");
    std::cout<<"V2/Legacy frame codec checks passed\n";
}
