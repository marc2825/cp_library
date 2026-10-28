#pragma once
#include <algorithm>
#include <climits>
#include <functional>
#include <queue>
#include <stdexcept>
#include <vector>
namespace cp {
using MatroidOracle = std::function<bool(const std::vector<int> &)>;
struct MatroidUnionResult {
    std::vector<int> assignment; // -1 means not selected, otherwise index of a matroid
    std::vector<std::vector<int>> sets;
    int cardinality = 0;
    bool complete = false;
    std::size_t oracle_calls = 0;
};
// Maximum subset partitionable among the supplied matroids; BFS reassigns elements along exchanges.
inline MatroidUnionResult matroid_union(int n, const std::vector<MatroidOracle> &oracles) {
    if (n < 0)
        throw std::invalid_argument("negative union ground size");
    if (oracles.size() > static_cast<std::size_t>(INT_MAX))
        throw std::length_error("too many matroids");
    MatroidUnionResult out;
    out.assignment.assign(n, -1);
    out.sets.resize(oracles.size());
    for (const auto &oracle : oracles) {
        ++out.oracle_calls;
        if (!oracle(std::vector<int>{}))
            throw std::invalid_argument("matroid must contain empty set");
    }
    for (;;) {
        std::vector<int> parent(n, -1);
        std::queue<int> queue;
        for (int e = 0; e < n; e++)
            if (out.assignment[e] < 0) {
                parent[e] = -2;
                queue.push(e);
            }
        int last = -1;
        std::size_t target = 0;
        auto test = [&](std::size_t j, int remove, int add) {
            auto s = out.sets[j];
            if (remove >= 0)
                s.erase(std::find(s.begin(), s.end(), remove));
            s.push_back(add);
            ++out.oracle_calls;
            return oracles[j](s);
        };
        while (!queue.empty() && last < 0) {
            const int e = queue.front();
            queue.pop();
            for (std::size_t j = 0; j < oracles.size() && last < 0; j++) {
                if (out.assignment[e] >= 0 && static_cast<std::size_t>(out.assignment[e]) == j)
                    continue;
                if (test(j, -1, e)) {
                    last = e;
                    target = j;
                    break;
                }
                for (int removed : out.sets[j])
                    if (parent[removed] == -1 && test(j, removed, e)) {
                        parent[removed] = e;
                        queue.push(removed);
                    }
            }
        }
        if (last < 0)
            break;
        int color = static_cast<int>(target);
        for (int v = last; v >= 0; v = parent[v]) {
            const int old = out.assignment[v];
            out.assignment[v] = color;
            color = old;
        }
        for (auto &set : out.sets)
            set.clear();
        for (int e = 0; e < n; e++)
            if (out.assignment[e] >= 0)
                out.sets[out.assignment[e]].push_back(e);
        ++out.cardinality;
    }
    out.complete = out.cardinality == n;
    return out;
}
inline MatroidUnionResult matroid_partition(int n, const std::vector<MatroidOracle> &oracles) {
    return matroid_union(n, oracles);
}
} // namespace cp
