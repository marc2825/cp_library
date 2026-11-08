#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace cp {

struct BipartiteMatchingResult {
    int cardinality = 0;
    std::vector<int> left_match, right_match; // -1 means unmatched.
    std::vector<std::pair<int, int>> pairs; // Ordered by left vertex.
};

// Iterative Hopcroft-Karp; left/right vertices have separate 0-based indices.
class BipartiteMatching {
    int right_size_;
    std::vector<std::vector<int>> adjacency_;
public:
    BipartiteMatching(int left_size, int right_size) : right_size_(right_size) {
        assert(left_size >= 0 && right_size >= 0);
        adjacency_.resize(left_size);
    }
    int left_size() const { return static_cast<int>(adjacency_.size()); }
    int right_size() const { return right_size_; }
    void add_edge(int left, int right) {
        assert(0 <= left && left < left_size() && 0 <= right && right < right_size());
        adjacency_[left].push_back(right);
    }

    // Recomputes from the empty matching; returned results own their arrays.
    BipartiteMatchingResult maximum_matching() const {
        BipartiteMatchingResult result;
        auto &left = result.left_match, &right = result.right_match;
        left.assign(left_size(), -1); right.assign(right_size(), -1);
        std::vector<int> distance(left_size()), vertices, via;
        std::vector<std::size_t> next(left_size());
        vertices.reserve(left_size()); via.reserve(left_size());
        const int infinity = std::numeric_limits<int>::max();
        while (true) {
            std::fill(distance.begin(), distance.end(), -1);
            std::queue<int> queue;
            for (int u = 0; u < left_size(); ++u) {
                if (left[u] == -1) { distance[u] = 0; queue.push(u); }
            }
            int shortest = infinity;
            while (!queue.empty()) {
                int u = queue.front(); queue.pop();
                if (distance[u] >= shortest) continue;
                for (int v : adjacency_[u]) {
                    int mate = right[v];
                    if (mate == -1) shortest = distance[u] + 1;
                    else if (distance[mate] == -1) {
                        distance[mate] = distance[u] + 1; queue.push(mate);
                    }
                }
            }
            if (shortest == infinity) break;
            std::fill(next.begin(), next.end(), 0);
            for (int root = 0; root < left_size(); ++root) {
                if (left[root] != -1 || distance[root] != 0) continue;
                vertices.assign(1, root); via.clear();
                while (!vertices.empty()) {
                    int u = vertices.back();
                    bool descended = false, augmented = false;
                    while (next[u] < adjacency_[u].size()) {
                        int v = adjacency_[u][next[u]++], mate = right[v];
                        if (mate == -1 && distance[u] + 1 == shortest) {
                            left[u] = v; right[v] = u;
                            for (int i = static_cast<int>(via.size()) - 1; i >= 0; --i) {
                                left[vertices[i]] = via[i]; right[via[i]] = vertices[i];
                            }
                            ++result.cardinality; augmented = true; break;
                        }
                        if (mate != -1 && distance[mate] == distance[u] + 1) {
                            via.push_back(v); vertices.push_back(mate);
                            descended = true; break;
                        }
                    }
                    if (augmented) break;
                    if (descended) continue;
                    distance[u] = -1; vertices.pop_back();
                    if (!via.empty()) via.pop_back();
                }
            }
        }
        for (int u = 0; u < left_size(); ++u)
            if (left[u] != -1) result.pairs.emplace_back(u, left[u]);
        return result;
    }
};

} // namespace cp
