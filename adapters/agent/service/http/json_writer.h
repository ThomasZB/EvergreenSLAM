/**
 * @file json_writer.h
 * @author hang chen (chen@hang.plus)
 * @brief Streaming JSON writer for the service's responses; the service never parses JSON.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_JSON_WRITER_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_JSON_WRITER_H_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace evergreenslam::agent {

class JsonWriter {
 public:
  JsonWriter& BeginObject();
  JsonWriter& EndObject();
  JsonWriter& BeginArray();
  JsonWriter& EndArray();
  JsonWriter& Key(std::string_view key);

  JsonWriter& String(std::string_view value);
  JsonWriter& Int(int64_t value);
  // Non-finite values are written as null.
  JsonWriter& Double(double value);
  JsonWriter& Bool(bool value);
  JsonWriter& Null();

  JsonWriter& Field(std::string_view key, std::string_view value) { return Key(key).String(value); }
  JsonWriter& Field(std::string_view key, const char* value) { return Key(key).String(value); }
  JsonWriter& Field(std::string_view key, const std::string& value) {
    return Key(key).String(value);
  }
  JsonWriter& Field(std::string_view key, int value) { return Key(key).Int(value); }
  JsonWriter& Field(std::string_view key, int64_t value) { return Key(key).Int(value); }
  JsonWriter& Field(std::string_view key, uint64_t value) {
    return Key(key).Int(static_cast<int64_t>(value));
  }
  JsonWriter& Field(std::string_view key, double value) { return Key(key).Double(value); }
  JsonWriter& Field(std::string_view key, bool value) { return Key(key).Bool(value); }
  JsonWriter& NullField(std::string_view key) { return Key(key).Null(); }
  JsonWriter& OptionalField(std::string_view key, const std::optional<std::string>& value);
  JsonWriter& OptionalField(std::string_view key, const std::optional<double>& value);

  const std::string& str() const { return out_; }

 private:
  void BeforeValue();
  void AppendQuoted(std::string_view value);

  std::string out_;
  // One entry per open container: whether it already holds an element.
  std::vector<bool> has_element_;
  bool after_key_ = false;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_JSON_WRITER_H_
