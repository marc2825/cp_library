#include <iostream>
#include <vector>
#include "basic_utility/rollback_array.hpp"
#include "union_find/rollback_disjoint_set_union_with_data.hpp"
int main() {
    cp::RollbackArray<int> sums(std::vector<int>{2,3,7});
    cp::RollbackDisjointSetUnionWithData<cp::RollbackArray<int>> dsu(3,sums);
    auto saved=dsu.snapshot();
    dsu.merge(0,1,[](auto& data,int kept,int absorbed){ data.set(kept,data[kept]+data[absorbed]); });
    std::cout << sums[dsu.leader(1)] << " " << dsu.component_count() << "\n";
    dsu.rollback(saved);
    std::cout << sums[0] << " " << sums[1] << " " << dsu.component_count() << "\n";
}
