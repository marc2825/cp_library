#include <iostream>
#include <string>
#include "string_algorithm/suffix_array.hpp"
#include "string_algorithm/lcp_array.hpp"
int main() {
    std::string s = "banana";
    auto sa = cp::suffix_array(s);
    for (int x : cp::lcp_array(s, sa)) std::cout << x << ' ';
    std::cout << '\n';
}
