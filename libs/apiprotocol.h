#pragma once

#include <string>
#include <vector>

namespace pulse::api {

struct Request {
    std::string method;
    std::vector<std::string> arguments;
    std::string buffers;
};

bool Active();
void Begin(const char* tool = nullptr);
Request ReadRequest(bool compilerMethods = true);
std::string String(const std::string& value);
void Record(const char* tag, const std::string& json);
void Line(const std::string& value);
void Finish(int exitCode);

}
