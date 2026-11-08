#include <iostream>
#include "static_query/fast_set.hpp"
int main() {
    cp::FastSet s(130); s.insert(3); s.insert(64); s.insert(129);
    std::cout<<s.next(4)<<" "<<s.prev(128)<<"\n";
}
