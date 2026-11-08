#pragma once
#include <algorithm>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
// 位置ごとの有限状態遷移をmin-plus積で管理。負費用可、nulloptは遷移なし。
class DynamicFiniteStateDP {
public:
    using Matrix = std::vector<std::vector<std::optional<long long>>>;
    struct Path { long long cost; std::vector<int> states; };
private:
    int states_, size_, base_ = 1;
    std::vector<Matrix> tree_;
    Matrix identity() const {
        Matrix result(states_, std::vector<std::optional<long long>>(states_));
        for (int i = 0; i < states_; ++i) result[i][i] = 0;
        return result;
    }
    void check_matrix(const Matrix& a) const {
        if (a.size() != static_cast<std::size_t>(states_)) throw std::invalid_argument("DP matrix dimension");
        for (const auto& row : a) if (row.size() != a.size()) throw std::invalid_argument("DP matrix dimension");
    }
    void check_range(int l, int r) const {
        if (l < 0 || l > r || r > size_) throw std::out_of_range("DP range");
    }
    Matrix multiply(const Matrix& a, const Matrix& b) const {
        Matrix c(states_, std::vector<std::optional<long long>>(states_));
        for (int i = 0; i < states_; ++i) for (int k = 0; k < states_; ++k) if (a[i][k])
            for (int j = 0; j < states_; ++j) if (b[k][j]) {
                const auto candidate = detail::dp_add(*a[i][k], *b[k][j]);
                if (!c[i][j] || candidate < *c[i][j]) c[i][j] = candidate;
            }
        return c;
    }
public:
    DynamicFiniteStateDP(int states, const std::vector<Matrix>& transitions) : states_(states), size_(0) {
        if (states <= 0 || transitions.size() > 100000000) throw std::invalid_argument("DP dimensions");
        size_ = static_cast<int>(transitions.size());
        for (const auto& a : transitions) check_matrix(a);
        while (base_ < size_) base_ *= 2;
        tree_.assign(2 * static_cast<std::size_t>(base_), identity());
        for (int i = 0; i < size_; ++i) tree_[base_+i] = transitions[i];
        for (int i = base_-1; i; --i) tree_[i] = multiply(tree_[2*i], tree_[2*i+1]);
    }
    int size() const { return size_; }
    int states() const { return states_; }
    // 更新経路を先に計算するためoverflow時にも元の状態を保持。
    void set(int position, Matrix value) {
        if (position < 0 || position >= size_) throw std::out_of_range("DP position");
        check_matrix(value);
        std::vector<std::pair<int, Matrix>> updates;
        int node = base_ + position; updates.emplace_back(node, std::move(value));
        while (node > 1) {
            auto parent = node % 2 ? multiply(tree_[node-1], updates.back().second)
                                   : multiply(updates.back().second, tree_[node+1]);
            node /= 2; updates.emplace_back(node, std::move(parent));
        }
        for (auto& update : updates) tree_[update.first].swap(update.second);
    }
    Matrix product(int l, int r) const {
        check_range(l,r); auto left = identity(), right = identity();
        for (l += base_, r += base_; l < r; l /= 2, r /= 2) {
            if (l & 1) left = multiply(left, tree_[l++]);
            if (r & 1) right = multiply(tree_[--r], right);
        }
        return multiply(left, right);
    }
    const Matrix& all_product() const { return tree_[1]; }
    // 復元だけは葉を順にDPする。O((r-l)*states^2)時間・O((r-l)*states)領域。
    std::optional<Path> restore(int l, int r, int source, int target) const {
        check_range(l,r);
        if (source < 0 || source >= states_ || target < 0 || target >= states_) throw std::out_of_range("DP state");
        std::vector<std::optional<long long>> dp(states_); dp[source] = 0;
        std::vector<std::vector<int>> parent(r-l, std::vector<int>(states_, -1));
        for (int p = l; p < r; ++p) {
            std::vector<std::optional<long long>> next(states_);
            const auto& a = tree_[base_+p];
            for (int i = 0; i < states_; ++i) if (dp[i]) for (int j = 0; j < states_; ++j) if (a[i][j]) {
                const auto candidate = detail::dp_add(*dp[i], *a[i][j]);
                if (!next[j] || candidate < *next[j]) { next[j] = candidate; parent[p-l][j] = i; }
            }
            dp.swap(next);
        }
        if (!dp[target]) return std::nullopt;
        Path result{*dp[target], {target}};
        for (int p = r-l; p-- > 0;) { target = parent[p][target]; result.states.push_back(target); }
        std::reverse(result.states.begin(), result.states.end());
        return result;
    }
};
} // namespace cp
