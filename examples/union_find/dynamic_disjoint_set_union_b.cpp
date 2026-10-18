#include <iostream>
#include "union_find/dynamic_disjoint_set_union.hpp"
#include <string>
int main() {
    cp::DynamicDisjointSetUnion<std::string> d; d.merge("alice","bob");
    std::cout << d.size("alice") << "\n";
}
