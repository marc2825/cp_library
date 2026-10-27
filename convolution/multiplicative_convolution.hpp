#pragma once
#include "../fps/detail/common.hpp"
namespace cp {
// c[k]=sum_{ij mod p=k} a[i]b[j], prime index modulus p=a.size()=b.size().
template <int Mod = 998244353, int Root = 3>
std::vector<int> multiplicative_convolution(const std::vector<int> &a, const std::vector<int> &b) {
    using F = fps_detail::Field<Mod>;
    const auto p = a.size();
    if (p != b.size() || p > 2000000000ULL || !convolution_detail::prime(static_cast<long long>(p)))
        throw std::invalid_argument("multiplicative convolution index modulus must be prime <= 2e9");
    std::vector<std::size_t> factors;
    auto r = p - 1;
    for (std::size_t d = 2; d <= r / d; ++d)
        if (r % d == 0) {
            factors.push_back(d);
            while (r % d == 0)
                r /= d;
        }
    if (r > 1)
        factors.push_back(r);
    std::size_t root = 1;
    for (;; ++root) {
        bool ok = true;
        for (auto d : factors)
            if (convolution_detail::power(root, (p - 1) / d, p) == 1)
                ok = false;
        if (ok)
            break;
    }
    std::vector<int> x(p - 1), y(p - 1), out(p);
    std::vector<std::size_t> index(p - 1);
    std::size_t v = 1;
    int sa = 0, sb = 0;
    for (std::size_t i = 0; i < p; ++i) {
        sa = F::add(sa, F::norm(a[i]));
        sb = F::add(sb, F::norm(b[i]));
    }
    out[0] = F::sub(F::add(F::mul(F::norm(a[0]), sb), F::mul(F::norm(b[0]), sa)),
                    F::mul(F::norm(a[0]), F::norm(b[0])));
    for (std::size_t i = 0; i < p - 1; ++i) {
        index[i] = v;
        x[i] = a[v];
        y[i] = b[v];
        v = v * root % p;
    }
    auto prod = fps_detail::multiply<Mod, Root>(x, y);
    for (std::size_t i = 0; i < prod.size(); ++i)
        out[index[i % (p - 1)]] = F::add(out[index[i % (p - 1)]], prod[i]);
    return out;
}
} // namespace cp
