#pragma once

// merge(u,v,w): potential[v] - potential[u] = w。矛盾する制約は採用しない。
// 詳細: docs/union_find/potentialized_disjoint_set_union.md
#include <algorithm>
#include <cassert>
#include <optional>
#include <vector>

namespace cp {

enum class PotentialMergeResult { Merged, AlreadyConsistent, Contradiction };

// T は +, -, ==, T(0) で表現できる可換群。整数・modint・ベクトルなど。
template <class T = long long>
class PotentializedDisjointSetUnion {
public:
    using value_type = T;
    using MergeResult = PotentialMergeResult;

    explicit PotentializedDisjointSetUnion(int n = 0)
        : parent_or_size_(checked_size(n), -1), weight_(n, T(0)), component_count_(n) {}

    int vertex_count() const { return static_cast<int>(parent_or_size_.size()); }
    int component_count() const { return component_count_; }

    int leader(int v) {
        assert(0 <= v && v < vertex_count());
        if (parent_or_size_[v] < 0) return v;
        int previous = parent_or_size_[v];
        int representative = leader(previous);
        weight_[v] = weight_[previous] + weight_[v];
        parent_or_size_[v] = representative;
        return representative;
    }

    bool same(int u, int v) { return leader(u) == leader(v); }
    int size(int v) { return -parent_or_size_[leader(v)]; }

    // 現在の代表元を0とした相対値。代表元が変わればこの値も変わり得る。
    T potential(int v) {
        leader(v);
        return weight_[v];
    }

    // potential[v] - potential[u]。別成分なら nullopt。
    std::optional<T> diff(int u, int v) {
        if (!same(u, v)) return std::nullopt;
        return weight_[v] - weight_[u];
    }

    MergeResult merge(int u, int v, const T& difference) {
        int a = leader(u), b = leader(v);
        if (a == b) {
            return weight_[v] - weight_[u] == difference
                ? MergeResult::AlreadyConsistent : MergeResult::Contradiction;
        }
        // root b を root a の下につなぐときの potential[b] - potential[a]。
        T shift = weight_[u] + difference - weight_[v];
        if (-parent_or_size_[a] < -parent_or_size_[b]) {
            std::swap(a, b);
            shift = T(0) - shift;
        }
        parent_or_size_[a] += parent_or_size_[b];
        parent_or_size_[b] = a;
        weight_[b] = shift;
        --component_count_;
        return MergeResult::Merged;
    }

    std::vector<std::vector<int>> groups() {
        std::vector<int> group_index(vertex_count(), -1);
        std::vector<std::vector<int>> result;
        result.reserve(component_count_);
        for (int v = 0; v < vertex_count(); ++v) {
            int representative = leader(v);
            if (group_index[representative] == -1) {
                group_index[representative] = static_cast<int>(result.size());
                result.push_back({});
            }
            result[group_index[representative]].push_back(v);
        }
        return result;
    }

    void reset(int n) {
        assert(n >= 0);
        parent_or_size_.assign(n, -1);
        weight_.assign(n, T(0));
        component_count_ = n;
    }

private:
    // 根なら -成分サイズ、それ以外なら親。weight[v] = potential[v] - potential[parent[v]]。
    std::vector<int> parent_or_size_;
    std::vector<T> weight_;
    int component_count_;

    static int checked_size(int n) {
        assert(n >= 0);
        return n;
    }
};

} // namespace cp
