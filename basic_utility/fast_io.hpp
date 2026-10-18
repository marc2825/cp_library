#pragma once
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <ios>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>
#include "detail/integer_traits.hpp"

namespace cp {
// Non-owning FILE*, buffered ASCII token input. EOF returns false; errors throw.
template <std::size_t BufferSize = 1 << 16> class BasicFastScanner {
    static_assert(BufferSize > 0, "buffer must be nonempty");
    std::FILE* file_;
    std::array<unsigned char, BufferSize> buffer_{};
    std::size_t position_ = 0, end_ = 0;
    int peek() {
        if (position_ == end_) {
            end_ = std::fread(buffer_.data(), 1, BufferSize, file_);
            position_ = 0;
            if (!end_) {
                if (std::ferror(file_)) throw std::ios_base::failure("FastScanner read failed");
                return EOF;
            }
        }
        return buffer_[position_];
    }
    static bool space(int c) { return c >= 0 && c <= 32; }
    bool skip_space() {
        int c;
        while (space(c = peek())) ++position_;
        return c != EOF;
    }
public:
    explicit BasicFastScanner(std::FILE* file = stdin) : file_(file) {
        if (!file) throw std::invalid_argument("null input stream");
    }
    BasicFastScanner(const BasicFastScanner&) = delete;
    BasicFastScanner& operator=(const BasicFastScanner&) = delete;
    bool read(char& value) {
        if (!skip_space()) return false;
        value = static_cast<char>(peek());
        ++position_;
        return true;
    }
    bool read(std::string& value) {
        if (!skip_space()) return false;
        std::string result;
        int c;
        while ((c = peek()) != EOF && !space(c)) {
            result.push_back(static_cast<char>(c));
            ++position_;
        }
        value = std::move(result);
        return true;
    }
    template <class T, std::enable_if_t<detail::is_integer_v<T> && !std::is_same<T, char>::value, int> = 0>
    bool read(T& value) {
        if (!skip_space()) return false;
        using U = detail::unsigned_integer_t<T>;
        const bool negative = peek() == '-';
        if (negative || peek() == '+') ++position_;
        U limit = static_cast<U>(std::numeric_limits<T>::max());
        if constexpr (std::numeric_limits<T>::is_signed) {
            if (negative) ++limit;
        }
        U magnitude = 0;
        bool digits = false, invalid = negative && !std::numeric_limits<T>::is_signed, overflow = false;
        int c;
        while ((c = peek()) != EOF && !space(c)) {
            ++position_;
            if (c < '0' || c > '9') { invalid = true; continue; }
            digits = true;
            const auto digit = static_cast<U>(c - '0');
            if (magnitude > (limit - digit) / 10) overflow = true;
            if (!overflow) magnitude = static_cast<U>(magnitude * 10 + digit);
        }
        if (!digits || invalid) throw std::invalid_argument("invalid integer token");
        if (overflow) throw std::out_of_range("integer token out of range");
        if constexpr (std::numeric_limits<T>::is_signed) {
            if (negative) {
                value = magnitude == limit ? std::numeric_limits<T>::lowest() : static_cast<T>(-static_cast<T>(magnitude));
                return true;
            }
        }
        value = static_cast<T>(magnitude);
        return true;
    }
    template <class T, std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
    bool read(T& value) {
        std::string token;
        if (!read(token)) return false;
        char* end = nullptr;
        errno = 0;
        T result;
        if constexpr (std::is_same<T, float>::value) result = std::strtof(token.c_str(), &end);
        else if constexpr (std::is_same<T, double>::value) result = std::strtod(token.c_str(), &end);
        else result = std::strtold(token.c_str(), &end);
        if (end != token.c_str() + token.size()) throw std::invalid_argument("invalid floating token");
        if (errno == ERANGE || !std::isfinite(result)) throw std::out_of_range("floating token out of range");
        value = result;
        return true;
    }
    bool read() { return true; }
    template <class T, class U, class... Rest> bool read(T& first, U& second, Rest&... rest) {
        return read(first) && read(second, rest...);
    }
};
using FastScanner = BasicFastScanner<>;

// Non-owning FILE*. Call flush() explicitly to observe write errors.
template <std::size_t BufferSize = 1 << 16> class BasicFastPrinter {
    static_assert(BufferSize > 0, "buffer must be nonempty");
    std::FILE* file_;
    std::array<char, BufferSize> buffer_{};
    std::size_t size_ = 0;
    bool failed_ = false;
    void drain() {
        if (failed_) throw std::ios_base::failure("FastPrinter is in failed state");
        std::size_t done = 0;
        while (done < size_) {
            const auto count = std::fwrite(buffer_.data() + done, 1, size_ - done, file_);
            if (!count) {
                failed_ = true;
                throw std::ios_base::failure("FastPrinter write failed");
            }
            done += count;
        }
        size_ = 0;
    }
    void floating(long double value, int precision, bool fixed) {
        if (!std::isfinite(value)) throw std::out_of_range("nonfinite output");
        const char* format = fixed ? "%.*Lf" : "%.*Lg";
        const int length = std::snprintf(nullptr, 0, format, precision, value);
        if (length < 0) throw std::runtime_error("floating formatting failed");
        std::vector<char> text(static_cast<std::size_t>(length) + 1);
        if (std::snprintf(text.data(), text.size(), format, precision, value) != length)
            throw std::runtime_error("floating formatting failed");
        write(std::string_view(text.data(), static_cast<std::size_t>(length)));
    }
public:
    explicit BasicFastPrinter(std::FILE* file = stdout) : file_(file) {
        if (!file) throw std::invalid_argument("null output stream");
    }
    BasicFastPrinter(const BasicFastPrinter&) = delete;
    BasicFastPrinter& operator=(const BasicFastPrinter&) = delete;
    ~BasicFastPrinter() noexcept { try { flush(); } catch (...) {} }
    void flush() {
        drain();
        if (std::fflush(file_) != 0) {
            failed_ = true;
            throw std::ios_base::failure("FastPrinter flush failed");
        }
    }
    bool failed() const { return failed_; }
    void write(char value) {
        if (failed_) throw std::ios_base::failure("FastPrinter is in failed state");
        if (size_ == BufferSize) drain();
        buffer_[size_++] = value;
    }
    void write(std::string_view value) { for (char c : value) write(c); }
    void write(const std::string& value) { write(std::string_view(value)); }
    void write(const char* value) { write(std::string_view(value)); }
    template <class T, std::enable_if_t<detail::is_integer_v<T> && !std::is_same<T, char>::value, int> = 0>
    void write(T value) {
        using U = detail::unsigned_integer_t<T>;
        U magnitude = static_cast<U>(value);
        if constexpr (std::numeric_limits<T>::is_signed) {
            if (value < 0) { write('-'); magnitude = U(0) - magnitude; }
        }
        char digits[std::numeric_limits<U>::digits10 + 1];
        std::size_t count = 0;
        do { digits[count++] = static_cast<char>('0' + magnitude % 10); magnitude /= 10; } while (magnitude);
        while (count) write(digits[--count]);
    }
    template <class T, std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
    void write(T value) { floating(value, std::numeric_limits<T>::max_digits10, false); }
    void write_fixed(long double value, int digits = 10) {
        if (digits < 0 || digits > 1000) throw std::invalid_argument("precision must be in [0,1000]");
        floating(value, digits, true);
    }
    void write() {}
    template <class T, class U, class... Rest> void write(const T& first, const U& second, const Rest&... rest) {
        write(first); write(' '); write(second, rest...);
    }
    template <class... T> void writeln(const T&... values) { write(values...); write('\n'); }
};
using FastPrinter = BasicFastPrinter<>;
} // namespace cp
