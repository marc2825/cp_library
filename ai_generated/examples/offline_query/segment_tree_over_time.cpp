#include <iostream>
#include <utility>
#include "offline_query/segment_tree_over_time.hpp"
#include "union_find/rollback_disjoint_set_union.hpp"
int main() {
    cp::SegmentTreeOverTime<std::pair<int,int>> timeline(4);
    timeline.add(0,3,{0,1}); timeline.add(1,2,{1,2});
    cp::RollbackDisjointSetUnion dsu(3);
    timeline.run(dsu,[](auto& d,const auto& e){ d.merge(e.first,e.second); },
        [](auto& d,int t){ std::cout << t << ": " << d.component_count() << "\n"; });
    std::cout << "restored " << dsu.component_count() << "\n";
}
