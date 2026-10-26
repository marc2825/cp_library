#pragma once
#include <cstddef>
#include <vector>

namespace cp {
template<class State> struct StateSearchEdge {
    State to;
    long long cost = 1;
    int action = -1; // 利用者が指定する操作ID
};
template<class State> struct StateSearchResult {
    long long cost = 0;
    std::vector<State> states;
    std::vector<int> actions;
    std::size_t expanded = 0;
};
} // namespace cp
