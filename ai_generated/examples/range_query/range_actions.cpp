#include <iostream>
#include "range_query/range_actions.hpp"
#include "range_query/dual_segtree.hpp"
using A=cp::AffineSumAction<long long>;
long long point_mapping(A::F f,long long x) { return f.a*x+f.b; }
int main() {
    cp::ActionLazySegtree<A> lazy(A::values({1,2,3,4}));
    cp::DualSegtree<long long,A::F,point_mapping,A::composition,A::id> dual(
        std::vector<long long>{1,2,3,4});
    lazy.apply(0,4,{1,5}); dual.apply(0,4,{1,5}); // add 5
    lazy.apply(1,3,{0,2}); dual.apply(1,3,{0,2}); // assign 2
    std::cout<<lazy.all_prod().sum<<' '<<dual.get(2)<<'\n';
}
