#include <iostream>
#include "string_algorithm/rolling_hash.hpp"
int main() {
    cp::RollingHash a("abracadabra"), b("abrax");
    std::cout << a.equal(a, 0, 4, 7, 11) << '\n';
    std::cout << a.lcp(b, 0, 0) << '\n';
    std::cout << (a.concat(a.get(0, 4), a.get(4, 7), 3) == a.get(0, 7)) << '\n';
}
