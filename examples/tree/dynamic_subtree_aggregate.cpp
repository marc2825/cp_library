#include <iostream>
#include "../../tree/dynamic_subtree_aggregate.hpp"
#include "../../range_query/range_actions.hpp"
int main() {
    using A = cp::AddSumAction<long long>;
    cp::DynamicSubtreeAggregate<A> tree(A::values({1, 2, 3, 4}));
    tree.link(0, 1); tree.link(1, 2); tree.link(1, 3);
    tree.subtree_apply(0, 1, 10);
    std::cout << tree.subtree_prod(0, 1)->sum << '\n';
    tree.cut(1, 3); tree.link(0, 3);
    std::cout << tree.subtree_prod(3, 0)->sum << '\n';
    std::cout << tree.path_prod(2, 3)->sum << '\n';
}
