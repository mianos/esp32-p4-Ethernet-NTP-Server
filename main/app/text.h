/* Bounded formatting into fixed buffers (put) and strict parsing (getList).
 * Fixed char buffers can be copied under a spinlock without allocating.
 * getList needs exactly the right number of fields, each followed by the
 * separator or the end, so "12x" is an error. */
#pragma once

#include <array>
#include <cstring>
#include <type_traits>
#include <charconv>
#include <cstddef>
#include <format>
#include <string_view>
#include <system_error>
#include <utility>

namespace text {

/* Fixed-size buffer; the last byte is kept for the terminator. */
template <std::size_t N> using Buf = std::array<char, N>;

template <std::size_t N, typename... Args>
void put(Buf<N> &dst, std::format_string<Args...> fmt, Args &&...args)
{
    static_assert(N > 0, "a text buffer needs room for the terminator");
    const auto r = std::format_to_n(dst.data(), dst.size() - 1, fmt, std::forward<Args>(args)...);
    *r.out       = '\0';
}

/* For C APIs that take a char array. */
template <std::size_t N, typename... Args>
void put(char (&dst)[N], std::format_string<Args...> fmt, Args &&...args)
{
    static_assert(N > 0, "a text buffer needs room for the terminator");
    const auto r = std::format_to_n(dst, N - 1, fmt, std::forward<Args>(args)...);
    *r.out       = '\0';
}

/* Split `text` on `sep` into exactly sizeof...(Ts) numbers. On any error the
 * outputs are left unchanged and false is returned. */
template <typename... Ts> bool getList(std::string_view text, char sep, Ts &...out)
{
    auto        fields = std::array<std::string_view, sizeof...(Ts)>{};
    std::size_t n      = 0;

    for (std::size_t i = 0;;) {
        if (n == fields.size()) {
            return false; /* more fields than there are places for them */
        }
        const std::size_t at = text.find(sep, i);
        fields[n++] =
            text.substr(i, at == std::string_view::npos ? std::string_view::npos : at - i);
        if (at == std::string_view::npos) {
            break;
        }
        i = at + 1;
    }
    if (n != fields.size()) {
        return false;
    }

    bool        ok  = true;
    std::size_t k   = 0;
    const auto  one = [&](auto &dst) {
        const std::string_view f = fields[k++];
        const auto             r = std::from_chars(f.data(), f.data() + f.size(), dst);
        ok                       = ok && r.ec == std::errc{} && r.ptr == f.data() + f.size();
    };
    (one(out), ...);
    return ok;
}

/* Little-endian field access (UBX and the target), sized by the type. */
template <typename T> void putLe(std::uint8_t *at, T v)
{
    static_assert(std::is_integral_v<T> || std::is_enum_v<T>, "putLe is for whole numbers");
    std::memcpy(at, &v, sizeof v);
}

template <typename T> T getLe(const std::uint8_t *at)
{
    static_assert(std::is_integral_v<T> || std::is_enum_v<T>, "getLe is for whole numbers");
    T v{};
    std::memcpy(&v, at, sizeof v);
    return v;
}

}  // namespace text
