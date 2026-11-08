#include <iostream>
#include "static_query/succinct_bit_vector.hpp"
int main() {
    cp::SuccinctBitVector s(5); s.set(1); s.set(4); s.build();
    std::cout << s.rank(4) << " " << s.select(1) << "\n";
}
