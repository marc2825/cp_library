#pragma once
#include <cassert>
#include <numeric>
#include <utility>
#include <vector>

namespace cp {
// 無向辺追加。奇閉路のある成分の状態を保持し、全体の二部性もO(1)。
class BipartiteDisjointSetUnion {
    std::vector<int> parent_,size_,parity_; std::vector<bool> bad_;
    int bad_count_=0;
    std::pair<int,int> find(int v) {
        assert(0<=v && v<int(parent_.size()));
        if (parent_[v]==v) return {v,0};
        auto [r,p]=find(parent_[v]); parity_[v]^=p; parent_[v]=r;
        return {r,parity_[v]};
    }
public:
    explicit BipartiteDisjointSetUnion(int n):parent_(n),size_(n,1),parity_(n),bad_(n) {
        assert(n>=0); std::iota(parent_.begin(),parent_.end(),0);
    }
    int leader(int v) { return find(v).first; }
    int size(int v) { return size_[leader(v)]; }
    bool same(int u,int v) { return leader(u)==leader(v); }
    bool is_bipartite() const { return bad_count_==0; }
    bool component_is_bipartite(int v) { return !bad_[leader(v)]; }
    // 返り値は追加後の成分の二部性。同じ色の端点を結ぶ辺も削除せず保持。
    bool add_edge(int u,int v) {
        auto [a,x]=find(u); auto [b,y]=find(v);
        if (a==b) {
            if (x==y && !bad_[a]) { bad_[a]=true; ++bad_count_; }
        } else {
            if (size_[a]<size_[b]) std::swap(a,b);
            bad_count_-=int(bad_[a])+int(bad_[b]);
            parent_[b]=a; parity_[b]=x^y^1; size_[a]+=size_[b]; bad_[a]=bad_[a]||bad_[b];
            bad_count_+=int(bad_[a]);
        }
        return !bad_[a];
    }
};
} // namespace cp
