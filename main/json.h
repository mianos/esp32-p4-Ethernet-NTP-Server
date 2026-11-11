/*
 * cJSON with ownership.
 *
 * cJSON is a C library: every Create/Parse has a matching Delete/free that the
 * caller has to remember, and this firmware builds a JSON object in every
 * handler. json::Value is that ownership expressed as a std::unique_ptr, so a
 * handler reads as a sequence of additions instead of a chain of null checks
 * and frees:
 *
 *     json::Value doc = json::makeObject();
 *     if (!doc) {
 *         return httpd_resp_send_500(req);
 *     }
 *     json::addString(doc, "url", "/healthz");
 *     return sendJson(req, std::move(doc), 200);
 *
 * A Value may be null (the allocation failed); test it like a pointer. The
 * helpers are thin, so cJSON remains available through doc.get() when a case
 * needs it.
 */
#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "cJSON.h"

namespace json {

namespace detail {

struct Deleter {
    void operator()(cJSON *doc) const { cJSON_Delete(doc); }
};

}  // namespace detail

/* Owning handle to a cJSON tree. */
using Value = std::unique_ptr<cJSON, detail::Deleter>;

/* cJSON_CreateObject(); null if the allocation failed. */
inline Value makeObject()
{
    return Value{cJSON_CreateObject()};
}

/* cJSON_Parse(); null on a syntax error or a failed allocation. */
inline Value parse(std::string_view text)
{
    const std::string terminated{text};  // cJSON needs a terminator in the buffer
    return Value{cJSON_Parse(terminated.c_str())};
}

/* cJSON_PrintUnformatted as a std::string; empty if it could not serialise. */
inline std::string print(const Value &doc)
{
    std::unique_ptr<char, decltype(&cJSON_free)> text{
        doc ? cJSON_PrintUnformatted(doc.get()) : nullptr, &cJSON_free};
    return text ? std::string{text.get()} : std::string{};
}

inline void addString(Value &doc, const char *key, const std::string &value)
{
    cJSON_AddStringToObject(doc.get(), key, value.c_str());
}

inline void addString(Value &doc, const char *key, const char *value)
{
    cJSON_AddStringToObject(doc.get(), key, value != nullptr ? value : "");
}

inline void addNumber(Value &doc, const char *key, double value)
{
    cJSON_AddNumberToObject(doc.get(), key, value);
}

inline void addNumber(Value &doc, const char *key, int value)
{
    addNumber(doc, key, static_cast<double>(value));
}

inline void addBool(Value &doc, const char *key, bool value)
{
    cJSON_AddBoolToObject(doc.get(), key, value);
}

inline void addNull(Value &doc, const char *key)
{
    cJSON_AddNullToObject(doc.get(), key);
}

}  // namespace json
