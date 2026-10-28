#pragma once
#include "../basic_utility/detail/rational_polynomial.hpp"
#include "partizan_game.hpp"
#include <limits>
#include <optional>
#include <string>
namespace cp {
// Ordered subfield Q(omega) of surreal numbers. omega exceeds every rational constant.
// This is not ordinal arithmetic, nimber arithmetic, or the whole surreal field.
class OmegaRational {
    using Polynomial = std::vector<BigRational>;
    Polynomial numerator_, denominator_{BigRational(1)};
    static Polynomial add(Polynomial a, const Polynomial &b) {
        a.resize(std::max(a.size(), b.size()));
        for (std::size_t i = 0; i < b.size(); i++)
            a[i] += b[i];
        rational_polynomial_detail::trim(a);
        return a;
    }
    static Polynomial multiply(const Polynomial &a, const Polynomial &b) {
        if (a.empty() || b.empty())
            return {};
        if (a.size() > std::numeric_limits<std::size_t>::max() - b.size() + 1)
            throw std::length_error("omega polynomial degree overflow");
        Polynomial c(a.size() + b.size() - 1);
        for (std::size_t i = 0; i < a.size(); i++)
            for (std::size_t j = 0; j < b.size(); j++)
                c[i + j] += a[i] * b[j];
        return c;
    }
    void normalize() {
        using namespace rational_polynomial_detail;
        trim(numerator_);
        trim(denominator_);
        if (denominator_.empty())
            throw std::domain_error("omega rational zero denominator");
        if (numerator_.empty()) {
            denominator_ = {1};
            return;
        }
        const auto common = gcd(numerator_, denominator_);
        numerator_ = divide(numerator_, common).first;
        denominator_ = divide(denominator_, common).first;
        const auto scale = denominator_.back();
        for (auto &x : numerator_)
            x /= scale;
        for (auto &x : denominator_)
            x /= scale;
    }

  public:
    OmegaRational() = default;
    OmegaRational(const OmegaRational &) = default;
    OmegaRational(OmegaRational &&other) : OmegaRational() { swap(other); }
    OmegaRational &operator=(OmegaRational other) noexcept {
        swap(other);
        return *this;
    }
    void swap(OmegaRational &other) noexcept {
        numerator_.swap(other.numerator_);
        denominator_.swap(other.denominator_);
    }
    friend void swap(OmegaRational &a, OmegaRational &b) noexcept { a.swap(b); }
    OmegaRational(BigRational constant) : numerator_{std::move(constant)} { normalize(); }
    template <class T, std::enable_if_t<detail::is_integer_v<T>, int> = 0>
    OmegaRational(T value) : OmegaRational(BigRational(value)) {}
    OmegaRational(Polynomial numerator, Polynomial denominator)
        : numerator_(std::move(numerator)), denominator_(std::move(denominator)) {
        normalize();
    }
    static OmegaRational omega() { return {{0, 1}, {1}}; }
    static OmegaRational infinitesimal() { return {{1}, {0, 1}}; }
    const Polynomial &numerator() const { return numerator_; }
    const Polynomial &denominator() const { return denominator_; }
    int sign() const { return numerator_.empty() ? 0 : numerator_.back().sign(); }
    explicit operator bool() const { return sign() != 0; }
    bool is_finite() const { return numerator_.size() <= denominator_.size(); }
    bool is_infinitesimal() const { return numerator_.size() < denominator_.size(); }
    std::optional<BigRational> standard_part() const {
        if (!is_finite())
            return std::nullopt;
        return numerator_.size() < denominator_.size() ? BigRational(0) : numerator_.back();
    }
    PartizanOutcome outcome() const {
        return sign() > 0   ? PartizanOutcome::left_wins
               : sign() < 0 ? PartizanOutcome::right_wins
                            : PartizanOutcome::previous_wins;
    }
    OmegaRational operator-() const {
        auto p = numerator_;
        for (auto &x : p)
            x = -x;
        return {std::move(p), denominator_};
    }
    friend OmegaRational operator+(const OmegaRational &a, const OmegaRational &b) {
        return {add(multiply(a.numerator_, b.denominator_), multiply(b.numerator_, a.denominator_)),
                multiply(a.denominator_, b.denominator_)};
    }
    friend OmegaRational operator-(const OmegaRational &a, const OmegaRational &b) { return a + (-b); }
    friend OmegaRational operator*(const OmegaRational &a, const OmegaRational &b) {
        return {multiply(a.numerator_, b.numerator_), multiply(a.denominator_, b.denominator_)};
    }
    friend OmegaRational operator/(const OmegaRational &a, const OmegaRational &b) {
        return {multiply(a.numerator_, b.denominator_), multiply(a.denominator_, b.numerator_)};
    }
    OmegaRational &operator+=(const OmegaRational &b) { return *this = *this + b; }
    OmegaRational &operator-=(const OmegaRational &b) { return *this = *this - b; }
    OmegaRational &operator*=(const OmegaRational &b) { return *this = *this * b; }
    OmegaRational &operator/=(const OmegaRational &b) { return *this = *this / b; }
    friend bool operator==(const OmegaRational &a, const OmegaRational &b) {
        return a.numerator_ == b.numerator_ && a.denominator_ == b.denominator_;
    }
    friend bool operator!=(const OmegaRational &a, const OmegaRational &b) { return !(a == b); }
    friend bool operator<(const OmegaRational &a, const OmegaRational &b) { return (a - b).sign() < 0; }
    friend bool operator>(const OmegaRational &a, const OmegaRational &b) { return b < a; }
    friend bool operator<=(const OmegaRational &a, const OmegaRational &b) { return !(b < a); }
    friend bool operator>=(const OmegaRational &a, const OmegaRational &b) { return !(a < b); }
    std::string to_string() const {
        auto text = [](const Polynomial &p) {
            std::string s;
            for (auto i = p.size(); i > 0; --i)
                if (p[i - 1]) {
                    if (!s.empty())
                        s += " + ";
                    s += "(" + p[i - 1].to_string() + ")";
                    if (i > 1)
                        s += "*w^" + std::to_string(i - 1);
                }
            return s.empty() ? std::string("0") : s;
        };
        return "(" + text(numerator_) + ")/(" + text(denominator_) + ")";
    }
};
} // namespace cp
