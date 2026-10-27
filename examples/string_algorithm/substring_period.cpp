#include "string_algorithm/substring_period.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::SubstringPeriodIndex index("zzabababx");
    std::cout << index.is_period(2, 8, 4) << ' ' << index.minimum_repetition_period(2, 8) << ' '
              << index.repetition_count(2, 8) << '\n';
}
