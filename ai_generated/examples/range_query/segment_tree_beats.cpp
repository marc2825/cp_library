#include <iostream>
#include "range_query/segment_tree_beats.hpp"
int main() {
    cp::SegmentTreeBeats s(std::vector<long long>{5, 1, 7});
    s.range_chmin(0, 3, 4); s.range_add(1, 3, 2); s.range_chmax(0, 2, 4);
    std::cout << s.range_sum(0, 3) << ' ' << s.range_min(0, 3) << ' ' << s.range_max(0, 3) << '\n';
}
