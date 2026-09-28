/**
 * @file json_writer.cc
 * @author hang chen (chen@hang.plus)
 * @brief Streaming JSON writer for the service's responses; the service never parses JSON.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/json_writer.h"

#include <cmath>
#include <cstdio>

namespace evergreenslam::agent {

void JsonWriter::BeforeValue() {
  if (after_key_) {
    after_key_ = false;
    return;
  }
  if (!has_element_.empty()) {
    if (has_element_.back()) {
      out_ += ',';
    }
    has_element_.back() = true;
  }
}

void JsonWriter::AppendQuoted(std::string_view value) {
  out_ += '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        out_ += "\\\"";
        break;
      case '\\':
        out_ += "\\\\";
        break;
      case '\n':
        out_ += "\\n";
        break;
      case '\r':
        out_ += "\\r";
        break;
      case '\t':
        out_ += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char escaped[8];
          std::snprintf(escaped, sizeof(escaped), "\\u%04x", static_cast<unsigned char>(c));
          out_ += escaped;
        } else {
          out_ += c;
        }
    }
  }
  out_ += '"';
}

JsonWriter& JsonWriter::BeginObject() {
  BeforeValue();
  out_ += '{';
  has_element_.push_back(false);
  return *this;
}

JsonWriter& JsonWriter::EndObject() {
  has_element_.pop_back();
  out_ += '}';
  return *this;
}

JsonWriter& JsonWriter::BeginArray() {
  BeforeValue();
  out_ += '[';
  has_element_.push_back(false);
  return *this;
}

JsonWriter& JsonWriter::EndArray() {
  has_element_.pop_back();
  out_ += ']';
  return *this;
}

JsonWriter& JsonWriter::Key(std::string_view key) {
  BeforeValue();
  AppendQuoted(key);
  out_ += ':';
  after_key_ = true;
  return *this;
}

JsonWriter& JsonWriter::String(std::string_view value) {
  BeforeValue();
  AppendQuoted(value);
  return *this;
}

JsonWriter& JsonWriter::Int(int64_t value) {
  BeforeValue();
  out_ += std::to_string(value);
  return *this;
}

JsonWriter& JsonWriter::Double(double value) {
  if (!std::isfinite(value)) {
    return Null();
  }
  BeforeValue();
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  out_ += buffer;
  return *this;
}

JsonWriter& JsonWriter::Bool(bool value) {
  BeforeValue();
  out_ += value ? "true" : "false";
  return *this;
}

JsonWriter& JsonWriter::Null() {
  BeforeValue();
  out_ += "null";
  return *this;
}

JsonWriter& JsonWriter::OptionalField(std::string_view key,
                                      const std::optional<std::string>& value) {
  Key(key);
  return value.has_value() ? String(*value) : Null();
}

JsonWriter& JsonWriter::OptionalField(std::string_view key, const std::optional<double>& value) {
  Key(key);
  return value.has_value() ? Double(*value) : Null();
}

}  // namespace evergreenslam::agent
