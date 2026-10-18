#include <iostream>
#include "ordered_structure/lazy_balanced_bst.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::LazyBalancedBST<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id> s(A::values({1,2,3})); s.apply(0,3,{2,1}); s.reverse(0,3);
    std::cout << s.get(0).sum << "\n";
}
