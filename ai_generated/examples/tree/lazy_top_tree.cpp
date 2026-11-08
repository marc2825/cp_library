#include <iostream>
#include "../../tree/lazy_top_tree.hpp"
#include "../../range_query/range_actions.hpp"
int main() {
    using Spec = cp::TopTreeMonoidAction<cp::AffineSumAction<long long>>;
    cp::LazyTopTree<Spec> tree({{{1, 1}}, {{2, 1}}, {{3, 1}}, {{4, 1}}});
    tree.link(0, 1); tree.link(1, 2); tree.link(1, 3);
    tree.path_apply(0, 2, {{2, 1}});
    tree.subtree_apply(0, 1, {{0, 5}});
    std::cout << tree.path_cluster(0, 2)->spine.sum << '\n';
    tree.cut(1, 3); tree.link(0, 3);
    std::cout << tree.component_cluster(0).all().sum << '\n';
}
