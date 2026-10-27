#include "string_algorithm/semi_local_lcs.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::SemiLocalLCS s("abac", "baca");
    cp::PrefixSubstringLCS p("abac", "baca");
    std::cout << p.query(3, 1, 4) << ' ' << s.string_substring(1, 4) << ' ' << s.substring_string(1, 4) << ' '
              << s.prefix_suffix(3, 1) << ' ' << s.suffix_prefix(1, 3) << '\n';
}
