#include "string_algorithm/online_z.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::OnlineZ z("ababa");
    std::cout << z.query(2) << ' ';
    z.append('b');
    std::cout << z.query(2) << '\n';
    print(z.values());
}
