#pragma once
#include <istream>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include "big_integer.hpp"

namespace cp {
// Canonical arbitrary-precision fraction: denominator > 0, gcd == 1, zero == 0/1.
class BigRational {
    BigInt numerator_, denominator_ = 1;
public:
    BigRational() = default;
    BigRational(BigInt numerator, BigInt denominator = 1)
        : numerator_(std::move(numerator)), denominator_(std::move(denominator)) {
        if (!denominator_) throw std::domain_error("BigRational zero denominator");
        if (denominator_.sign() < 0) { numerator_ = -numerator_; denominator_ = -denominator_; }
        const auto common = gcd(numerator_, denominator_);
        numerator_ /= common; denominator_ /= common;
    }
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    BigRational(T value) : BigRational(BigInt(value)) {}
    BigRational(const BigRational&) = default;
    BigRational(BigRational&& other) : BigRational() { swap(other); }
    BigRational& operator=(BigRational other) noexcept { swap(other); return *this; }
    void swap(BigRational& other) noexcept {
        static_assert(std::is_nothrow_swappable_v<BigInt>);
        using std::swap; swap(numerator_, other.numerator_); swap(denominator_, other.denominator_);
    }
    friend void swap(BigRational& a, BigRational& b) noexcept { a.swap(b); }
    static BigRational parse(std::string_view text) {
        const auto slash = text.find('/');
        if (slash == std::string_view::npos) return BigRational(BigInt::parse(text));
        return BigRational(BigInt::parse(text.substr(0, slash)), BigInt::parse(text.substr(slash + 1)));
    }
    const BigInt& numerator() const { return numerator_; }
    const BigInt& denominator() const { return denominator_; }
    int sign() const { return numerator_.sign(); }
    explicit operator bool() const { return bool(numerator_); }
    std::string to_string() const { return numerator_.to_string() + "/" + denominator_.to_string(); }
    BigRational operator+() const { return *this; }
    BigRational operator-() const { return BigRational(-numerator_, denominator_); }
    friend BigRational operator+(const BigRational& a, const BigRational& b) {
        const auto g = gcd(a.denominator_, b.denominator_);
        const auto left = a.denominator_ / g, right = b.denominator_ / g;
        return BigRational(a.numerator_ * right + b.numerator_ * left, left * b.denominator_);
    }
    friend BigRational operator-(const BigRational& a, const BigRational& b) { return a + (-b); }
    friend BigRational operator*(const BigRational& a, const BigRational& b) {
        const auto g = gcd(a.numerator_, b.denominator_), h = gcd(b.numerator_, a.denominator_);
        return BigRational((a.numerator_/g)*(b.numerator_/h), (a.denominator_/h)*(b.denominator_/g));
    }
    friend BigRational operator/(const BigRational& a, const BigRational& b) {
        if (!b) throw std::domain_error("BigRational division by zero");
        return a * BigRational(b.denominator_, b.numerator_);
    }
    BigRational& operator+=(const BigRational& b) { return *this = *this + b; }
    BigRational& operator-=(const BigRational& b) { return *this = *this - b; }
    BigRational& operator*=(const BigRational& b) { return *this = *this * b; }
    BigRational& operator/=(const BigRational& b) { return *this = *this / b; }
    friend bool operator==(const BigRational& a, const BigRational& b) {
        return a.numerator_ == b.numerator_ && a.denominator_ == b.denominator_;
    }
    friend bool operator!=(const BigRational& a, const BigRational& b) { return !(a == b); }
    friend bool operator<(const BigRational& a, const BigRational& b) {
        return a.numerator_ * b.denominator_ < b.numerator_ * a.denominator_;
    }
    friend bool operator>(const BigRational& a, const BigRational& b) { return b < a; }
    friend bool operator<=(const BigRational& a, const BigRational& b) { return !(b < a); }
    friend bool operator>=(const BigRational& a, const BigRational& b) { return !(a < b); }
    friend std::ostream& operator<<(std::ostream& out, const BigRational& value) { return out << value.to_string(); }
    friend std::istream& operator>>(std::istream& in, BigRational& value) {
        std::string token; if (!(in >> token)) return in;
        try { value = parse(token); }
        catch (const std::invalid_argument&) { in.setstate(std::ios_base::failbit); }
        catch (const std::domain_error&) { in.setstate(std::ios_base::failbit); }
        return in;
    }
};
inline BigRational abs(const BigRational& x) { return x.sign() < 0 ? -x : x; }
} // namespace cp
