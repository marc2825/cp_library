#include <iostream>
#include "range_query/dynamic_lazy_segtree.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::DynamicLazySegtree<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id,A::zero> s(1000000000000LL);
    s.apply(10,20,{2,3}); std::cout<<s.prod(0,100).sum<<"\n";
}
