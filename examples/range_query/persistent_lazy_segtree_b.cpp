#include <iostream>
#include "range_query/persistent_lazy_segtree.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::PersistentLazySegtree<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id> s(A::values({1,2,3}));
    auto root=s.initial_root(); auto next=s.apply(root,0,2,{2,1});
    std::cout << s.prod(root,0,3).sum << " " << s.prod(next,0,3).sum << "\n";
}
