#pragma once
#include <climits>
#include <cstddef>
#include <queue>
#include <stdexcept>
#include <vector>

namespace cp {
struct MatroidIntersectionResult {std::vector<int> selected;std::size_t oracle_calls=0;};
// oracle(vector<int> distinct_elements)->bool はマトロイドの独立性判定。
// 最短増加路をBFSで探す非重み付き版。O(r n^2)回のoracle呼出し。
template <class First,class Second> MatroidIntersectionResult matroid_intersection(int n,First first,Second second){
    if(n<0)throw std::invalid_argument("negative matroid ground size");
    MatroidIntersectionResult result;std::vector<unsigned char> selected(n);
    if(!first(std::vector<int>{})||!second(std::vector<int>{}))throw std::invalid_argument("matroids must contain the empty set");
    result.oracle_calls=2;
    for(;;){
        std::vector<int> inside,outside;for(int i=0;i<n;++i)(selected[i]?inside:outside).push_back(i);
        auto test=[&](auto&oracle,int remove,int add){std::vector<int> set;for(int e:inside)if(e!=remove)set.push_back(e);set.push_back(add);++result.oracle_calls;return oracle(set);};
        std::vector<int> parent(n,-1);std::queue<int> queue;
        for(int e:outside)if(test(first,-1,e)){parent[e]=-2;queue.push(e);}
        int target=-1;
        while(!queue.empty()&&target<0){
            const int e=queue.front();queue.pop();
            if(!selected[e]){
                if(test(second,-1,e)){target=e;break;}
                for(int x:inside)if(parent[x]<0&&parent[x]!=-2&&test(second,x,e)){parent[x]=e;queue.push(x);}
            }else for(int x:outside)if(parent[x]==-1&&test(first,e,x)){parent[x]=e;queue.push(x);}
        }
        if(target<0){result.selected=std::move(inside);return result;}
        for(int v=target;v>=0;v=parent[v])selected[v]^=1;
    }
}
} // namespace cp
