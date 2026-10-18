#pragma once
#include <cstdint>
#include <istream>
#include <limits>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include "integer_traits.hpp"
#include "../../number_theory_basic/mod_inverse.hpp"

namespace cp {
namespace detail {
// Derived supplies mod(), val(), integral construction and +=, -=, *=.
template <class Derived> struct ModIntOperators {
    Derived operator+() const { self().val(); return self(); }
    Derived operator-() const { return Derived(0) - self(); }
    Derived& operator++() { return self() += Derived(1); }
    Derived& operator--() { return self() -= Derived(1); }
    Derived operator++(int) { auto old = self(); ++self(); return old; }
    Derived operator--(int) { auto old = self(); --self(); return old; }
    template <class I, std::enable_if_t<is_integer_v<I>, int> = 0>
    Derived pow(I exponent) const {
        if constexpr (std::numeric_limits<I>::is_signed) {
            if (exponent < 0) throw std::invalid_argument("negative modint exponent");
        }
        if (static_cast<__uint128_t>(exponent) > std::numeric_limits<std::uint64_t>::max())
            throw std::out_of_range("modint exponent exceeds uint64");
        self().val();
        auto e = static_cast<std::uint64_t>(exponent);
        Derived result(1), base = self();
        while (e) {
            if (e & 1) result *= base;
            e >>= 1;
            if (e) base *= base;
        }
        return result;
    }
    std::optional<Derived> try_inv() const {
        const auto inverse = mod_inverse(static_cast<long long>(self().val()),
                                         static_cast<long long>(Derived::mod()));
        if (!inverse) return std::nullopt;
        return Derived(*inverse);
    }
    Derived inv() const {
        const auto inverse = try_inv();
        if (!inverse) throw std::domain_error("modint has no inverse");
        return *inverse;
    }
    Derived& operator/=(const Derived& other) { return self() *= other.inv(); }
    friend Derived operator+(Derived a, const Derived& b) { return a += b; }
    friend Derived operator-(Derived a, const Derived& b) { return a -= b; }
    friend Derived operator*(Derived a, const Derived& b) { return a *= b; }
    friend Derived operator/(Derived a, const Derived& b) { return a /= b; }
    friend bool operator==(const Derived& a, const Derived& b) { return a.val() == b.val(); }
    friend bool operator!=(const Derived& a, const Derived& b) { return !(a == b); }
    friend std::ostream& operator<<(std::ostream& out, const Derived& value) {
        return out << value.val();
    }
    friend std::istream& operator>>(std::istream& in, Derived& value) {
        std::string token;
        if (!(in >> token)) return in;
        std::size_t pos = 0;
        const bool negative = token[0] == '-';
        if (token[0] == '-' || token[0] == '+') ++pos;
        Derived result(0);
        bool valid = pos < token.size();
        for (; pos < token.size(); ++pos) {
            if (token[pos] < '0' || token[pos] > '9') { valid = false; break; }
            result = result * Derived(10) + Derived(token[pos] - '0');
        }
        if (!valid) in.setstate(std::ios::failbit);
        else value = negative ? -result : result;
        return in;
    }
private:
    Derived& self() { return static_cast<Derived&>(*this); }
    const Derived& self() const { return static_cast<const Derived&>(*this); }
};
} // namespace detail
} // namespace cp
