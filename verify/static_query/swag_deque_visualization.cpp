#include "static_query/swag_deque.hpp"
#include "test_values.hpp"
#include <iostream>
#include <utility>
#include <vector>
int main() {
    cp::SWAGDeque<std::string, static_test::concatenate, static_test::empty_string> queue;
    for (auto [kind, value] : std::vector<std::pair<char, int>>{
            {'R',4}, {'R',2}, {'R',2}, {'R',5}, {'l',0}, {'L',9},
            {'r',0}, {'r',0}, {'r',0}, {'R',7}, {'q',0}, {'c',0}, {'q',0}}) {
        if (kind == 'L') queue.push_front(std::to_string(value));
        if (kind == 'R') queue.push_back(std::to_string(value));
        if (kind == 'l') queue.pop_front();
        if (kind == 'r') queue.pop_back();
        if (kind == 'c') queue.clear();
        auto copy = queue;
        std::cout << "{\"size\":" << queue.size() << ",\"product\":\"" << queue.prod() << "\",\"values\":[";
        bool first = true;
        while (!copy.empty()) {
            if (!first) std::cout << ',';
            first = false; std::cout << '"' << copy.front() << '"'; copy.pop_front();
        }
        std::cout << "]}\n";
    }
}
