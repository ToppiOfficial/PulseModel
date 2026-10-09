#ifndef PULSE_KEYVALUES3_KV3_H
#define PULSE_KEYVALUES3_KV3_H
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <utility>
#include <vector>

namespace pulse::keyvalues3 {
struct Value {
    using Array = std::vector<Value>;
    using Object = std::map<std::string, Value>;
    using Blob = std::vector<uint8_t>;
    std::variant<std::nullptr_t, bool, int64_t, uint64_t, double, std::string, Array, Object, Blob> data{nullptr};
    std::string flag;
    Value() = default;
    Value(std::nullptr_t) : data(nullptr) {}
    Value(bool v) : data(v) {}
    Value(int v) : data(static_cast<int64_t>(v)) {}
    Value(int64_t v) : data(v) {}
    Value(uint64_t v) : data(v) {}
    Value(double v) : data(v) {}
    Value(const char* v) : data(std::string(v)) {}
    Value(std::string v) : data(std::move(v)) {}
    Value(Array v) : data(std::move(v)) {}
    Value(Object v) : data(std::move(v)) {}
    Value(Blob v) : data(std::move(v)) {}
    bool IsObject() const;
    bool IsArray() const;
    bool Has(const std::string& key) const;
    Object& Members();
    const Object& Members() const;
    Array& Items();
    const Array& Items() const;
    Value& operator[](const std::string& key);
    const Value& At(const std::string& key) const;
    std::string String() const;
    int64_t Integer() const;
    double Number() const;
    bool Boolean() const;
};
Value ParseText(const std::string& text);
std::string WriteText(const Value& value);
std::string TextBody(const Value& value, size_t indent = 0);
Value ParseBinary(const std::vector<uint8_t>& bytes);
std::vector<uint8_t> ReadFile(const std::string& path);
void WriteFile(const std::string& path, const std::vector<uint8_t>& data);
void WriteFile(const std::string& path, const std::string& data);
std::vector<uint8_t> ResourceBlock(const std::vector<uint8_t>& file, const std::string& block = "DATA");
Value ReadResource(const std::vector<uint8_t>& file, const std::string& block = "DATA");
std::string Quote(const std::string& text);
}
#endif
