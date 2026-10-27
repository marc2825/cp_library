#pragma once
#include "../../convolution/convolution_mod.hpp"
#include "../polynomial.hpp"
#include <algorithm>
#include <utility>
#include <vector>

namespace cp {
namespace finite_polynomial_detail {
// Small-characteristic polynomial arithmetic; no NTT root/capacity condition on Mod.
template <int Mod> struct Algebra {
    using F = fps_detail::Field<Mod>;
    using P = std::vector<int>;
    static P trim(P a) {
        for (auto &x : a)
            x = F::norm(x);
        fps_detail::trim(a);
        return a;
    }
    static P add(const P &a, const P &b) { return trim(polynomial_add<Mod>(a, b)); }
    static P sub(const P &a, const P &b) { return trim(polynomial_subtract<Mod>(a, b)); }
    static P scale(P a, int c) {
        for (auto &x : a)
            x = F::mul(x, c);
        return trim(std::move(a));
    }
    static P mul(const P &a, const P &b) {
        if (a.empty() || b.empty())
            return {};
        if (std::min(a.size(), b.size()) > 32)
            return trim(convolution_mod(a, b, Mod));
        const auto size = convolution_detail::shape(a.size(), b.size(), std::vector<int>().max_size()).first;
        P c(size);
        for (std::size_t i = 0; i < a.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j)
                c[i + j] = F::add(c[i + j], F::mul(a[i], b[j]));
        return trim(std::move(c));
    }
    static std::pair<P, P> divmod(P a, P b) {
        a = trim(std::move(a));
        b = trim(std::move(b));
        if (b.empty())
            throw std::domain_error("polynomial division by zero");
        if (a.size() < b.size())
            return {{}, a};
        P q(a.size() - b.size() + 1);
        int inv = F::inv(b.back());
        for (std::size_t i = q.size(); i-- > 0;) {
            q[i] = F::mul(a[i + b.size() - 1], inv);
            for (std::size_t j = 0; j < b.size(); ++j)
                a[i + j] = F::sub(a[i + j], F::mul(q[i], b[j]));
        }
        return {trim(std::move(q)), trim(std::move(a))};
    }
    static P mod(const P &a, const P &b) { return divmod(a, b).second; }
    static P monic(P a) {
        a = trim(std::move(a));
        return a.empty() ? a : scale(a, F::inv(a.back()));
    }
    static P gcd(P a, P b) {
        a = trim(std::move(a));
        b = trim(std::move(b));
        while (!b.empty()) {
            auto r = mod(a, b);
            a = std::move(b);
            b = std::move(r);
        }
        return monic(std::move(a));
    }
    static P power(P a, std::uint64_t e, const P &m) {
        P r = mod({1}, m);
        a = mod(a, m);
        for (; e; e >>= 1) {
            if (e & 1)
                r = mod(mul(r, a), m);
            if (e > 1)
                a = mod(mul(a, a), m);
        }
        return r;
    }
};
} // namespace finite_polynomial_detail
} // namespace cp
