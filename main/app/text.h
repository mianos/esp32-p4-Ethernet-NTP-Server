/* Text in and out of C++ types, without the C library's habit of trusting the
 * caller.
 *
 * The two jobs here are the two the printf family did badly. `put` formats into
 * storage whose size is part of its type: the bounds are checked while the
 * compiler can still see them, the specifier and the argument types cannot
 * disagree, and a value that does not fit is truncated rather than corrupting
 * what follows. `getList` is the read side: from_chars cannot walk past the end
 * of the string it was given, and it names the field that failed instead of
 * returning a bare count that every caller then had to compare against its own
 * number.
 *
 * Fixed storage rather than std::string is deliberate where a value is shared
 * between tasks: an array of char is POD, so it can be copied under a spinlock
 * without allocating in a critical section. It is the storage that is C, not the
 * code around it.
 *
 * Parsing is strict on purpose. A field must be followed by the delimiter or by
 * the end of the string, so "12x" is an error and not a 12, and the field count
 * has to match exactly -- a settings string that has grown a field is a
 * different thing from one that changed a value, and the old code silently
 * took the first n.
 */
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

/* A size in the type, for the reason in the header comment. One less than N is
 * always available to the terminator, so a full buffer is still a string. */
template <std::size_t N> using Buf = std::array<char, N>;

template <std::size_t N, typename... Args>
void put(Buf<N> &dst, std::format_string<Args...> fmt, Args &&...args)
{
    static_assert(N > 0, "a text buffer needs room for the terminator");
    const auto r = std::format_to_n(dst.data(), dst.size() - 1, fmt, std::forward<Args>(args)...);
    *r.out       = '\0';
}

/* A char array is what the C APIs in the SDK take, so both shapes are accepted:
 * put() on a Buf, and this on the three places that hand the address straight to
 * lwIP or inet_ntop. */
template <std::size_t N, typename... Args>
void put(char (&dst)[N], std::format_string<Args...> fmt, Args &&...args)
{
    static_assert(N > 0, "a text buffer needs room for the terminator");
    const auto r = std::format_to_n(dst, N - 1, fmt, std::forward<Args>(args)...);
    *r.out       = '\0';
}

/* Split `text` on `sep` into exactly sizeof...(Ts) numbers. All or nothing: a
 * short field list, a long one, an empty field or trailing junk all return false
 * and leave the outputs as they were found. */
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

/* Whole numbers in and out of a wire frame. UBX is little-endian on every
 * field and so is the target, so this is a memcpy with the size taken from the
 * type rather than written next to it -- which is where the "4" and the
 * uint16_t beside it drift apart. */
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
