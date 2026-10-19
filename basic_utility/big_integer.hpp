#pragma once
#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>
#include <cstdint>
#include <istream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include "detail/integer_traits.hpp"

namespace cp {
// Arbitrary precision signed integer. Boost headers required; no link library.
// Division truncates toward zero; text streams always use decimal tokens.
class BigInt {
public:
    using backend_type = boost::multiprecision::cpp_int;
private:
    backend_type value_ = 0;
    static void base_check(unsigned base) {
        if (base != 10 && base != 16) throw std::invalid_argument("BigInt base must be 10 or 16");
    }
public:
    BigInt() = default;
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    BigInt(T value) : value_(value) {}
    explicit BigInt(backend_type value) : value_(std::move(value)) {}
    explicit BigInt(std::string_view text, unsigned base = 10) : BigInt(parse(text, base)) {}
    static BigInt parse(std::string_view text, unsigned base = 10) {
        base_check(base);
        std::size_t position = 0;
        const bool negative = !text.empty() && text.front() == '-';
        if (!text.empty() && (text.front() == '+' || negative)) ++position;
        if (base == 16 && text.size() - position >= 2 && text[position] == '0' &&
            (text[position + 1] == 'x' || text[position + 1] == 'X')) position += 2;
        if (position == text.size()) throw std::invalid_argument("empty BigInt digits");
        backend_type value = 0;
        // Decimal chunks avoid Boost's leading-zero octal interpretation.
        const std::size_t width = base == 10 ? 9 : 8;
        while (position < text.size()) {
            const auto count = std::min(width, text.size() - position);
            std::uint64_t chunk = 0, factor = 1;
            for (std::size_t i = 0; i < count; ++i) {
                const char c = text[position++];
                const unsigned digit = c >= '0' && c <= '9' ? unsigned(c - '0') :
                    c >= 'a' && c <= 'f' ? unsigned(c - 'a' + 10) :
                    c >= 'A' && c <= 'F' ? unsigned(c - 'A' + 10) : 16;
                if (digit >= base) throw std::invalid_argument("invalid BigInt digit");
                chunk = chunk * base + digit; factor *= base;
            }
            value *= factor; value += chunk;
        }
        if (negative) value = -value;
        return BigInt(std::move(value));
    }
    std::string to_string(unsigned base = 10) const {
        base_check(base);
        if (base == 10) return value_.str(0, std::ios_base::dec);
        const backend_type magnitude = value_ < 0 ? -value_ : value_;
        return (value_ < 0 ? "-" : "") + magnitude.str(0, std::ios_base::hex);
    }
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    T to_integral() const {
        if (value_ < backend_type(std::numeric_limits<T>::lowest()) ||
            value_ > backend_type(std::numeric_limits<T>::max()))
            throw std::out_of_range("BigInt does not fit target integer");
        return value_.template convert_to<T>();
    }
    int sign() const { return value_ == 0 ? 0 : value_ < 0 ? -1 : 1; }
    explicit operator bool() const { return value_ != 0; }
    const backend_type& native() const { return value_; }
    BigInt operator+() const { return *this; }
    BigInt operator-() const { return BigInt(backend_type(-value_)); }
    BigInt& operator+=(const BigInt& other) { value_ += other.value_; return *this; }
    BigInt& operator-=(const BigInt& other) { value_ -= other.value_; return *this; }
    BigInt& operator*=(const BigInt& other) { value_ *= other.value_; return *this; }
    BigInt& operator/=(const BigInt& other) {
        if (!other) throw std::domain_error("BigInt division by zero");
        value_ /= other.value_; return *this;
    }
    BigInt& operator%=(const BigInt& other) {
        if (!other) throw std::domain_error("BigInt division by zero");
        value_ %= other.value_; return *this;
    }
    friend BigInt operator+(BigInt a, const BigInt& b) { return a += b; }
    friend BigInt operator-(BigInt a, const BigInt& b) { return a -= b; }
    friend BigInt operator*(BigInt a, const BigInt& b) { return a *= b; }
    friend BigInt operator/(BigInt a, const BigInt& b) { return a /= b; }
    friend BigInt operator%(BigInt a, const BigInt& b) { return a %= b; }
    BigInt& operator++() { ++value_; return *this; }
    BigInt& operator--() { --value_; return *this; }
    BigInt operator++(int) { auto old = *this; ++*this; return old; }
    BigInt operator--(int) { auto old = *this; --*this; return old; }
    friend bool operator==(const BigInt& a, const BigInt& b) { return a.value_ == b.value_; }
    friend bool operator!=(const BigInt& a, const BigInt& b) { return !(a == b); }
    friend bool operator<(const BigInt& a, const BigInt& b) { return a.value_ < b.value_; }
    friend bool operator>(const BigInt& a, const BigInt& b) { return b < a; }
    friend bool operator<=(const BigInt& a, const BigInt& b) { return !(b < a); }
    friend bool operator>=(const BigInt& a, const BigInt& b) { return !(a < b); }
    friend std::ostream& operator<<(std::ostream& stream, const BigInt& value) { return stream << value.to_string(); }
    friend std::istream& operator>>(std::istream& stream, BigInt& value) {
        std::string token;
        if (!(stream >> token)) return stream;
        try { value = parse(token); }
        catch (const std::invalid_argument&) { stream.setstate(std::ios_base::failbit); }
        return stream;
    }
};
inline BigInt abs(const BigInt& value) { return value.sign() < 0 ? -value : value; }
inline std::pair<BigInt, BigInt> divmod(const BigInt& a, const BigInt& b) {
    if (!b) throw std::domain_error("BigInt division by zero");
    BigInt::backend_type quotient, remainder;
    boost::multiprecision::divide_qr(a.native(), b.native(), quotient, remainder);
    return {BigInt(std::move(quotient)), BigInt(std::move(remainder))};
}
inline BigInt gcd(const BigInt& a, const BigInt& b) {
    return BigInt(BigInt::backend_type(boost::multiprecision::gcd(a.native(), b.native())));
}
inline BigInt lcm(const BigInt& a, const BigInt& b) {
    return !a || !b ? BigInt(0) : abs((a / gcd(a, b)) * b);
}
} // namespace cp
