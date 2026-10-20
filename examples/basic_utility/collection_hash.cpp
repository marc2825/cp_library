#include <iostream>
#include "basic_utility/collection_hash.hpp"
int main() {
    cp::MultisetHash<int> a(42), b(42);
    a.insert(1); a.insert(1); a.insert(2); a.erase(1);
    b.insert(2); b.insert(1);
    std::cout << (a.fingerprint() == b.fingerprint()) << ' ' << a.size() << '\n';
    cp::SetHash<int> set(42);
    std::cout << set.insert(1) << ' ' << set.insert(1) << '\n';
}
