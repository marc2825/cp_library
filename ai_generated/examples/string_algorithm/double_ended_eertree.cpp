#include "string_algorithm/double_ended_eertree.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::DoubleEndedEertree e;
    e.push_back('a');
    e.push_back('b');
    e.push_front('a');
    e.push_back('a');
    e.pop_front();
    e.push_front('b');
    std::cout << e.distinct_count() << ' ' << e.total_occurrences() << ' ' << e.longest_prefix() << ' '
              << e.longest_suffix() << '\n';
}
