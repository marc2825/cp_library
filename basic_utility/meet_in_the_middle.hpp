#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cp {
struct SubsetSum {
    __int128_t sum;
    std::uint64_t mask;
    friend bool operator<(const SubsetSum& a, const SubsetSum& b) {
        return a.sum != b.sum ? a.sum < b.sum : a.mask < b.mask;
    }
};
// At most 23 items. Repeatedly merge sorted shifted lists: O(2^n) time and space.
inline std::vector<SubsetSum> sorted_subset_sums(const std::vector<std::int64_t>& values) {
    if (values.size() > 23) throw std::length_error("subset table supports at most 23 items");
    std::vector<SubsetSum> sums{{0, 0}};
    for (std::size_t bit = 0; bit < values.size(); ++bit) {
        std::vector<SubsetSum> next; next.reserve(sums.size() * 2);
        std::size_t i = 0, j = 0;
        while (i < sums.size() || j < sums.size()) {
            if (j == sums.size()) next.push_back(sums[i++]);
            else {
                SubsetSum shifted{sums[j].sum + values[bit], sums[j].mask | (UINT64_C(1) << bit)};
                if (i < sums.size() && sums[i] < shifted) next.push_back(sums[i++]);
                else { next.push_back(shifted); ++j; }
            }
        }
        sums.swap(next);
    }
    return sums;
}
class MeetInTheMiddle {
    std::size_t split_;
    std::vector<SubsetSum> left_, right_;
    std::uint64_t count_bound(__int128_t target, bool inclusive) const {
        std::uint64_t result = 0;
        std::size_t j = right_.size();
        for (const auto& a : left_) {
            while (j && (inclusive ? a.sum + right_[j - 1].sum > target : a.sum + right_[j - 1].sum >= target)) --j;
            result += j;
        }
        return result;
    }
public:
    explicit MeetInTheMiddle(const std::vector<std::int64_t>& values) : split_(values.size() / 2) {
        if (values.size() > 46) throw std::length_error("meet-in-the-middle supports at most 46 items");
        left_ = sorted_subset_sums({values.begin(), values.begin() + split_});
        right_ = sorted_subset_sums({values.begin() + split_, values.end()});
    }
    const std::vector<SubsetSum>& left_sums() const { return left_; }
    const std::vector<SubsetSum>& right_sums() const { return right_; }
    std::uint64_t count_leq(__int128_t target) const { return count_bound(target, true); }
    std::uint64_t count_equal(__int128_t target) const { return count_bound(target, true) - count_bound(target, false); }
    std::uint64_t count_range(__int128_t low, __int128_t high) const {
        if (high < low) throw std::invalid_argument("reversed subset sum range");
        return count_bound(high, false) - count_bound(low, false);
    }
    // Largest sum <= target, then smallest original-index bitmask (empty subset allowed).
    std::optional<SubsetSum> best_leq(__int128_t target) const {
        std::optional<SubsetSum> best;
        std::size_t j = right_.size();
        for (const auto& a : left_) {
            while (j && a.sum + right_[j - 1].sum > target) --j;
            if (!j) break;
            while (j > 1 && right_[j - 2].sum == right_[j - 1].sum) --j;
            SubsetSum candidate{a.sum + right_[j - 1].sum, a.mask | (right_[j - 1].mask << split_)};
            if (!best || best->sum < candidate.sum || (best->sum == candidate.sum && candidate.mask < best->mask)) best = candidate;
        }
        return best;
    }
};
} // namespace cp
