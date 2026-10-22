#pragma once

#include <initializer_list>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace pulse::api {

constexpr int kVersion = 2;

bool Active();
void Begin(const char* tool = nullptr);

// One JSON object read from stdin to EOF; its "version" must be kVersion and
// "method" a string. Each member a tool reads must be listed in CheckMembers.
nlohmann::json ReadRequest();
void CheckMembers(const nlohmann::json& request, std::initializer_list<const char*> allowed);
std::string GetString(const nlohmann::json& request, const char* name);
std::vector<std::string> GetStrings(const nlohmann::json& request, const char* name);

std::string String(const std::string& value);
void Record(const char* tag, const std::string& json);
void Line(const std::string& value);
void Finish(int exitCode);

}
