#include <iostream>
#include "range_query/segment_tree_cover.hpp"
int main() {
    cp::SegmentTreeCover layout(8);
    for (auto k : layout.cover(1,7)) {
        auto [l,r]=layout.interval(k);
        std::cout << k << ": [" << l << "," << r << ")\n";
    }
}
