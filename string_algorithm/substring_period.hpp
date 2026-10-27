#pragma once
#include "detail/lce.hpp"
#include <cstdint>

namespace cp {
// Exact static substring period queries. Repetition periods must divide the interval length.
class SubstringPeriodIndex {
    string_algorithm_detail::LCEIndex lce_;
    std::vector<int> smallest_prime_;
    static std::vector<int> bytes(std::string_view text) {
        std::vector<int> out;
        for (unsigned char c : text)
            out.push_back(c);
        return out;
    }

  public:
    explicit SubstringPeriodIndex(std::string_view text = {})
        : lce_(bytes(text)), smallest_prime_(text.size() + 1) {
        for (int p = 2; p <= size(); ++p)
            if (!smallest_prime_[p]) {
                smallest_prime_[p] = p;
                if (p <= size() / p)
                    for (std::int64_t q = std::int64_t(p) * p; q <= size(); q += p)
                        if (!smallest_prime_[q])
                            smallest_prime_[q] = p;
            }
    }
    int size() const { return lce_.n; }
    bool is_period(int l, int r, int period) const {
        string_algorithm_detail::range(size(), l, r);
        if (period <= 0 || period > r - l)
            throw std::invalid_argument("period must be in [1,length]");
        return lce_.lce(l, l + period) >= r - l - period;
    }
    int minimum_repetition_period(int l, int r) const {
        string_algorithm_detail::range(size(), l, r);
        int answer = r - l, remaining = answer;
        while (remaining > 1) {
            const int p = smallest_prime_[remaining];
            while (remaining % p == 0)
                remaining /= p;
            while (answer % p == 0 && is_period(l, r, answer / p))
                answer /= p;
        }
        return answer;
    }
    int repetition_count(int l, int r) const {
        const int p = minimum_repetition_period(l, r);
        return p ? (r - l) / p : 0;
    }
};
} // namespace cp
