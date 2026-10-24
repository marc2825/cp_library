#include <iostream>
#include "../../union_find/group_potentialized_disjoint_set_union.hpp"
#include <array>
struct Group {
    using S=std::array<int,3>;
    static S e(){return {0,1,2};}
    static S op(S a,S b){return {a[b[0]],a[b[1]],a[b[2]]};}
    static S inverse(S a){S b{};for(int i=0;i<3;++i)b[a[i]]=i;return b;}
};
int main(){
    cp::GroupPotentializedDisjointSetUnion<Group> dsu(3);
    dsu.merge(0,1,{1,0,2}); dsu.merge(1,2,{0,2,1});
    auto p=*dsu.diff(0,2);
    std::cout<<p[0]<<' '<<p[1]<<' '<<p[2]<<'\n';
}
