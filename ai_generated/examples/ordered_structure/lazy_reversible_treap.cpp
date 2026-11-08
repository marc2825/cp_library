#include <iostream>
#include "ordered_structure/lazy_reversible_treap.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::LazyReversibleTreap<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id> s(A::values({1,2,3}));
    s.apply(0,2,{2,1}); s.reverse(0,3); std::cout<<s.get(1).sum<<" "<<s.all_prod().sum<<"\n";
}
