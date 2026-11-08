#include <iostream>
#include <vector>
#include "heuristic_search/beam_search.hpp"
struct State { int depth, weight, value; };
int main() {
    const std::vector<int> weight{3, 4, 2}, value{4, 5, 3};
    cp::BeamSearchOptions options;
    options.width = 4;
    options.max_depth = 3;
    options.goal = cp::SearchGoal::maximize;
    const auto result = cp::beam_search(State{0, 0, 0},
        [](const State& s) { return s.value; },
        [&](const State& s) {
            const int i = s.depth;
            if (i == 3) return std::vector<State>{};
            std::vector<State> children{{i + 1, s.weight, s.value}};
            if (s.weight + weight[i] <= 5)
                children.push_back({i + 1, s.weight + weight[i], s.value + value[i]});
            return children;
        }, options);
    std::cout << result.best().score << ' ' << result.best().state.weight << '\n';
}
