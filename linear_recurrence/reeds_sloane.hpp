#pragma once
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace cp {
namespace reeds_sloane_detail {
// Reeds--Sloane (1985), Section IV. Pair length includes the numerator degree;
// trailing zero feedback coefficients must therefore be retained in the public result.
using Polynomial = std::vector<int>;
struct Pair {
    Polynomial denominator, numerator;
    std::size_t length() const {
        return std::max(denominator.empty() ? 0 : denominator.size() - 1, numerator.size());
    }
};
inline int norm(long long x, int m) {
    x %= m;
    return static_cast<int>(x < 0 ? x + m : x);
}
inline int inverse(int a, int m) {
    long long r = a, s = m, x = 1, y = 0;
    while (s) {
        auto q = r / s;
        auto next = r - q * s;
        r = s;
        s = next;
        next = x - q * y;
        x = y;
        y = next;
    }
    if (r != 1)
        throw std::logic_error("Reeds-Sloane divisor is not a unit");
    return norm(x, m);
}
inline void trim(Polynomial &a) {
    while (!a.empty() && !a.back())
        a.pop_back();
}
inline void subtract_shift(Polynomial &a, const Polynomial &b, std::size_t shift, int c, int m) {
    if (a.size() < b.size() + shift)
        a.resize(b.size() + shift);
    for (std::size_t i = 0; i < b.size(); ++i)
        a[i + shift] = norm(a[i + shift] - static_cast<long long>(c) * b[i], m);
    trim(a);
}
inline Pair prime_power(const Polynomial &sequence, int prime, int exponent, int modulus) {
    struct Discrepancy {
        int unit = 1, valuation = 0;
    };
    struct Archive {
        Pair pair;
        Discrepancy discrepancy;
        std::size_t time = 0;
    };
    auto split = [&](int d) {
        Discrepancy out{1, exponent};
        if (d) {
            out = {d, 0};
            while (out.unit % prime == 0) {
                out.unit /= prime;
                ++out.valuation;
            }
        }
        return out;
    };
    std::vector<int> powers(exponent + 1, 1);
    for (int i = 1; i <= exponent; ++i)
        powers[i] = powers[i - 1] * prime;
    std::vector<Pair> previous(exponent), current(exponent);
    std::vector<Discrepancy> discrepancies(exponent);
    std::vector<Archive> archive(exponent);
    for (int h = 0; h < exponent; ++h) {
        previous[h].denominator = {powers[h]};
        current[h] = previous[h];
        int d = norm(static_cast<long long>(sequence[0]) * powers[h], modulus);
        if (d)
            current[h].numerator = {d};
        discrepancies[h] = split(d);
    }
    for (std::size_t k = 1; k < sequence.size(); ++k) {
        for (int g = 0; g < exponent; ++g)
            if (current[g].length() > previous[g].length()) {
                int h = exponent - 1 - discrepancies[g].valuation;
                if (h < 0)
                    throw std::logic_error("Reeds-Sloane archive invariant");
                archive[g] = {previous[h], discrepancies[h], k - 1};
            }
        previous = current;
        for (int h = 0; h < exponent; ++h) {
            int delta = k < previous[h].numerator.size() ? norm(-previous[h].numerator[k], modulus) : 0;
            for (std::size_t i = 0; i < previous[h].denominator.size() && i <= k; ++i)
                delta = norm(delta + static_cast<long long>(previous[h].denominator[i]) * sequence[k - i],
                             modulus);
            discrepancies[h] = split(delta);
            if (!delta)
                continue;
            int g = exponent - 1 - discrepancies[h].valuation;
            if (!previous[g].length()) {
                current[h].numerator.resize(k + 1);
                current[h].numerator[k] =
                    norm(current[h].numerator[k] + static_cast<long long>(delta), modulus);
                trim(current[h].numerator);
            } else {
                const auto &old = archive[g];
                int difference = discrepancies[h].valuation - old.discrepancy.valuation;
                if (difference < 0)
                    throw std::logic_error("Reeds-Sloane valuation invariant");
                int scale = norm(static_cast<long long>(discrepancies[h].unit) *
                                     inverse(old.discrepancy.unit, modulus),
                                 modulus);
                scale = norm(static_cast<long long>(scale) * powers[difference], modulus);
                subtract_shift(current[h].denominator, old.pair.denominator, k - old.time, scale, modulus);
                subtract_shift(current[h].numerator, old.pair.numerator, k - old.time, scale, modulus);
            }
        }
    }
    return current[0];
}
} // namespace reeds_sloane_detail
// Minimal finite-sample recurrence s[i]=sum_j c[j]*s[i-1-j], i>=c.size().
// Runtime modulus 1..2e9, including prime powers and composite moduli. O(N^2 sum e + sqrt(modulus)).
inline std::vector<int> reeds_sloane(const std::vector<int> &samples, int modulus) {
    using namespace reeds_sloane_detail;
    if (modulus < 1 || modulus > 2000000000)
        throw std::invalid_argument("Reeds-Sloane modulus must be in [1,2e9]");
    if (modulus == 1 || samples.empty())
        return {};
    struct Component {
        int modulus;
        Pair pair;
    };
    std::vector<Component> components;
    int remaining = modulus;
    std::size_t length = 0;
    auto add = [&](int prime, int exponent, int power) {
        Polynomial s(samples.size());
        for (std::size_t i = 0; i < s.size(); ++i)
            s[i] = norm(samples[i], power);
        auto pair = prime_power(s, prime, exponent, power);
        length = std::max(length, pair.length());
        components.push_back({power, std::move(pair)});
    };
    for (int p = 2; p <= remaining / p; ++p)
        if (remaining % p == 0) {
            int e = 0, power = 1;
            do {
                remaining /= p;
                power *= p;
                ++e;
            } while (remaining % p == 0);
            add(p, e, power);
        }
    if (remaining > 1)
        add(remaining, 1, remaining);
    std::vector<int> result(length);
    for (const auto &component : components) {
        int other = modulus / component.modulus;
        int scale = norm(
            static_cast<long long>(other) * inverse(other % component.modulus, component.modulus), modulus);
        for (std::size_t j = 0; j < length; ++j) {
            int coefficient =
                j + 1 < component.pair.denominator.size() ? component.pair.denominator[j + 1] : 0;
            result[j] = norm(result[j] - static_cast<long long>(scale) * coefficient, modulus);
        }
    }
    return result;
}
} // namespace cp
