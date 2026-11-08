#include <iostream>
#include "static_query/dynamic_bitset.hpp"
int main() {
    cp::DynamicBitset s(130); s.set(63); s.set(64);
    std::cout << (s<<1).find_next(64) << " " << s.count() << "\n";
}
