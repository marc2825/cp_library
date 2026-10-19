#pragma once
#include <cstddef>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include "detail/integer_traits.hpp"

namespace cp {
struct DebugOptions {
    std::size_t max_depth = 8;
    std::size_t max_items = 32;
    std::size_t max_string = 256;
};

// Specialize with static void write(std::ostream&, const T&) for custom types.
template <class T> struct DebugFormatter {};

namespace debug_detail {
using std::begin;
using std::end;
template <class T, class = void> struct is_range : std::false_type {};
template <class T> struct is_range<T, std::void_t<decltype(begin(std::declval<const T&>())),
    decltype(end(std::declval<const T&>()))>> : std::true_type {};
template <class T, class = void> struct is_tuple : std::false_type {};
template <class T> struct is_tuple<T, std::void_t<decltype(std::tuple_size<T>::value)>> : std::true_type {};
template <class T, class = void> struct has_formatter : std::false_type {};
template <class T> struct has_formatter<T, std::void_t<decltype(DebugFormatter<T>::write(
    std::declval<std::ostream&>(), std::declval<const T&>()))>> : std::true_type {};
template <class T, class = void> struct is_streamable : std::false_type {};
template <class T> struct is_streamable<T, std::void_t<decltype(std::declval<std::ostream&>()
    << std::declval<const T&>())>> : std::true_type {};
template <class T> struct is_optional : std::false_type {};
template <class T> struct is_optional<std::optional<T>> : std::true_type {};
template <class T> struct is_variant : std::false_type {};
template <class... T> struct is_variant<std::variant<T...>> : std::true_type {};

inline void quoted(std::ostream& out, std::string_view s, char quote, std::size_t limit) {
    constexpr char hex[] = "0123456789abcdef";
    out.put(quote);
    const auto count = s.size() < limit ? s.size() : limit;
    for (std::size_t i = 0; i < count; ++i) {
        const auto c = static_cast<unsigned char>(s[i]);
        if (c == '\n') out << "\\n";
        else if (c == '\r') out << "\\r";
        else if (c == '\t') out << "\\t";
        else if (c == '\\' || c == static_cast<unsigned char>(quote)) out << '\\' << char(c);
        else if (c < 32 || c == 127) out << "\\x" << hex[c >> 4] << hex[c & 15];
        else out.put(char(c));
    }
    if (count < s.size()) out << "...";
    out.put(quote);
}

template <class T> void write(std::ostream& out, const T& value, const DebugOptions& opt, std::size_t depth);

template <class T, std::size_t... I>
void tuple(std::ostream& out, const T& value, const DebugOptions& opt, std::size_t depth,
           std::index_sequence<I...>) {
    out << '(';
    [[maybe_unused]] auto item = [&](auto index) {
        constexpr auto i = decltype(index)::value;
        if (i < opt.max_items) {
            if (i) out << ", ";
            write(out, std::get<i>(value), opt, depth + 1);
        }
    };
    (item(std::integral_constant<std::size_t, I>{}), ...);
    if (sizeof...(I) > opt.max_items) out << (opt.max_items ? ", ..." : "...");
    out << ')';
}

template <class T> void write(std::ostream& out, const T& value, const DebugOptions& opt, std::size_t depth) {
    using U = std::remove_cv_t<T>;
    if (depth > opt.max_depth) { out << "..."; return; }
    if constexpr (has_formatter<U>::value) DebugFormatter<U>::write(out, value);
    else if constexpr (std::is_same_v<U, std::string> || std::is_same_v<U, std::string_view>)
        quoted(out, value, '"', opt.max_string);
    else if constexpr (std::is_array_v<U> && std::is_same_v<std::remove_extent_t<U>, char>) {
        auto size = std::extent_v<U>;
        if (size && value[size - 1] == '\0') --size;
        quoted(out, std::string_view(value, size), '"', opt.max_string);
    } else if constexpr (std::is_same_v<U, char*> || std::is_same_v<U, const char*>) {
        if (!value) out << "null";
        else quoted(out, std::string_view(value), '"', opt.max_string);
    } else if constexpr (std::is_same_v<U, char>) quoted(out, std::string_view(&value, 1), '\'', 1);
    else if constexpr (std::is_same_v<U, bool>) out << (value ? "true" : "false");
    else if constexpr (detail::is_integer_v<U>) {
        using Unsigned = detail::unsigned_integer_t<U>;
        auto magnitude = static_cast<Unsigned>(value);
        if constexpr (std::numeric_limits<U>::is_signed) {
            if (value < 0) { out << '-'; magnitude = Unsigned{0} - magnitude; }
        }
        char buffer[128]; std::size_t size = 0;
        do { buffer[size++] = static_cast<char>('0' + magnitude % 10); magnitude /= 10; } while (magnitude);
        while (size) out.put(buffer[--size]);
    } else if constexpr (is_optional<U>::value) {
        if (!value) out << "nullopt";
        else { out << "optional("; write(out, *value, opt, depth + 1); out << ')'; }
    } else if constexpr (is_variant<U>::value) {
        if (value.valueless_by_exception()) out << "variant(valueless)";
        else {
            out << "variant[" << value.index() << "](";
            std::visit([&](const auto& x) { write(out, x, opt, depth + 1); }, value);
            out << ')';
        }
    } else if constexpr (std::is_same_v<U, std::monostate>) out << "monostate";
    else if constexpr (is_tuple<U>::value)
        tuple(out, value, opt, depth, std::make_index_sequence<std::tuple_size<U>::value>{});
    else if constexpr (is_range<U>::value) {
        out << '['; std::size_t count = 0;
        auto it = begin(value); const auto last = end(value);
        for (; it != last && count < opt.max_items; ++it, ++count) {
            if (count) out << ", ";
            write(out, *it, opt, depth + 1);
        }
        if (it != last) out << (count ? ", ..." : "...");
        out << ']';
    } else if constexpr (is_streamable<U>::value) out << value;
    else out << "<unprintable>";
}
} // namespace debug_detail

template <class T> std::string debug_string(const T& value, DebugOptions options = {}) {
    std::ostringstream out;
    debug_detail::write(out, value, options, 0);
    return out.str();
}

template <class... T>
void debug_log(std::ostream& out, const char* file, int line, const char* names, const T&... values) {
    out << '[' << file << ':' << line << "] " << names << " = "
        << debug_string(std::tie(values...)) << '\n';
}
} // namespace cp

// Disabled macros do not evaluate or type-check their arguments. Flags must be
// defined consistently before the first include of this header.
#if defined(CP_ENABLE_DEBUG) && !defined(CP_DISABLE_DEBUG) && !defined(ONLINE_JUDGE)
#define CP_DEBUG(...) ::cp::debug_log(std::cerr, __FILE__, __LINE__, #__VA_ARGS__, __VA_ARGS__)
#define CP_DEBUG_TO(stream, ...) ::cp::debug_log((stream), __FILE__, __LINE__, #__VA_ARGS__, __VA_ARGS__)
#else
#define CP_DEBUG(...) ((void)0)
#define CP_DEBUG_TO(...) ((void)0)
#endif
