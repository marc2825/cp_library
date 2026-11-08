#pragma once
#include <algorithm>
#include <cstdint>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace cp {
namespace string_algorithm_detail {
// Immutable suffix-link Euler intervals, terminal end positions, and byte-only DAG path counts.
class SAMIndex {
    std::vector<int> enter_, leave_;
    std::vector<std::pair<int, int>> terminals_;
    std::vector<std::uint64_t> paths_;

  public:
    template <class States> SAMIndex(const States &states, const std::vector<int> &terminal_states) {
        const int n = static_cast<int>(states.size());
        std::vector<std::vector<int>> children(n);
        for (int v = 1; v < n; ++v)
            children[states[v].link].push_back(v);
        enter_.resize(n);
        leave_.resize(n);
        std::vector<std::pair<int, bool>> stack{{0, false}};
        int timer = 0;
        while (!stack.empty()) {
            auto [v, exit] = stack.back();
            stack.pop_back();
            if (exit) {
                leave_[v] = timer;
                continue;
            }
            enter_[v] = timer++;
            stack.emplace_back(v, true);
            for (int u : children[v])
                stack.emplace_back(u, false);
        }
        for (std::size_t i = 0; i < terminal_states.size(); ++i)
            terminals_.emplace_back(enter_[terminal_states[i]], static_cast<int>(i));
        std::sort(terminals_.begin(), terminals_.end());
        std::vector<int> order(n);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](int a, int b) { return states[a].length > states[b].length; });
        paths_.assign(n, 0);
        for (int v : order)
            for (const auto &edge : states[v].next)
                if (static_cast<unsigned>(edge.first) < 256)
                    paths_[v] += 1 + paths_[edge.second];
    }
    std::vector<int> occurrences(int state, int length) const {
        if (state < 0)
            return {};
        auto it = std::lower_bound(terminals_.begin(), terminals_.end(), std::make_pair(enter_[state], -1));
        std::vector<int> out;
        while (it != terminals_.end() && it->first < leave_[state]) {
            out.push_back(it->second - length + 1);
            ++it;
        }
        std::sort(out.begin(), out.end());
        return out;
    }
    int count(int state) const {
        if (state < 0)
            return 0;
        auto a = std::lower_bound(terminals_.begin(), terminals_.end(), std::make_pair(enter_[state], -1));
        auto b = std::lower_bound(terminals_.begin(), terminals_.end(), std::make_pair(leave_[state], -1));
        return static_cast<int>(b - a);
    }
    std::uint64_t distinct_count() const { return paths_[0]; }
    template <class States> std::optional<std::string> kth(const States &states, std::uint64_t k) const {
        if (k >= paths_[0])
            return std::nullopt;
        int v = 0;
        std::string out;
        for (;;) {
            for (auto [c, u] : states[v].next) {
                if (static_cast<unsigned>(c) >= 256)
                    continue;
                const auto block = 1 + paths_[u];
                if (k >= block) {
                    k -= block;
                    continue;
                }
                out.push_back(static_cast<char>(c));
                if (!k)
                    return out;
                --k;
                v = u;
                break;
            }
        }
    }
};
template <class States> int sam_walk(const States &states, std::string_view pattern) {
    int v = 0;
    for (unsigned char c : pattern) {
        auto it = states[v].next.find(c);
        if (it == states[v].next.end())
            return -1;
        v = it->second;
    }
    return v;
}
} // namespace string_algorithm_detail
} // namespace cp
