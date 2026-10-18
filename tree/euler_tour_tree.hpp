#pragma once
// 説明: docs/tree/euler_tour_tree.md
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>
namespace cp {
// 可換モノイドの動的森。自己token1個と辺token2個をimplicit treapで保持。
template<class S,S(*op)(S,S),S(*e)()> class EulerTourTree {
    struct Node{int left=-1,right=-1,parent=-1,size=1,vertices=0;std::uint64_t priority;S value,prod;};
    std::vector<Node> nodes_;int n_;std::map<std::pair<int,int>,std::pair<int,int>> edges_;std::uint64_t random_=0x243f6a8885a308d3ULL;
    std::uint64_t random_priority(){random_+=0x9e3779b97f4a7c15ULL;auto z=random_;z=(z^(z>>30))*0xbf58476d1ce4e5b9ULL;z=(z^(z>>27))*0x94d049bb133111ebULL;return z^(z>>31);}
    int size_of(int v)const{return v==-1?0:nodes_[v].size;}
    int vertices_of(int v)const{return v==-1?0:nodes_[v].vertices;}
    S prod_of(int v)const{return v==-1?e():nodes_[v].prod;}
    void pull(int v){auto& x=nodes_[v];x.size=1+size_of(x.left)+size_of(x.right);x.vertices=(v<n_)+vertices_of(x.left)+vertices_of(x.right);x.prod=op(op(prod_of(x.left),x.value),prod_of(x.right));if(x.left!=-1)nodes_[x.left].parent=v;if(x.right!=-1)nodes_[x.right].parent=v;}
    int merge(int a,int b){if(a==-1){if(b!=-1)nodes_[b].parent=-1;return b;}if(b==-1){nodes_[a].parent=-1;return a;}
        if(nodes_[a].priority<nodes_[b].priority){int child=merge(nodes_[a].right,b);nodes_[a].right=child;pull(a);nodes_[a].parent=-1;return a;}
        int child=merge(a,nodes_[b].left);nodes_[b].left=child;pull(b);nodes_[b].parent=-1;return b;
    }
    std::pair<int,int> split(int v,int k){if(v==-1)return {-1,-1};int left=size_of(nodes_[v].left);
        if(k<=left){auto [a,b]=split(nodes_[v].left,k);nodes_[v].left=b;pull(v);nodes_[v].parent=-1;return {a,v};}
        auto [a,b]=split(nodes_[v].right,k-left-1);nodes_[v].right=a;pull(v);nodes_[v].parent=-1;return {v,b};
    }
    int root_of(int v)const{while(nodes_[v].parent!=-1)v=nodes_[v].parent;return v;}
    int rank_of(int v)const{int answer=size_of(nodes_[v].left);while(nodes_[v].parent!=-1){int p=nodes_[v].parent;if(nodes_[p].right==v)answer+=size_of(nodes_[p].left)+1;v=p;}return answer;}
    int rotate(int v){int position=rank_of(v);auto [a,b]=split(root_of(v),position);return merge(b,a);}
    int token(S value){int id=static_cast<int>(nodes_.size());nodes_.push_back({-1,-1,-1,1,id<n_?1:0,random_priority(),value,value});return id;}
    void check(int v)const{assert(0<=v && v<n_);(void)v;}
public:
    explicit EulerTourTree(const std::vector<S>& values):n_(static_cast<int>(values.size())){for(const auto& x:values)token(x);}
    explicit EulerTourTree(int n):EulerTourTree(std::vector<S>(checked_size(n),e())){}
    int size()const{return n_;}
    bool connected(int u,int v)const{check(u);check(v);return root_of(u)==root_of(v);}
    void make_root(int v){check(v);rotate(v);}
    bool link(int u,int v){check(u);check(v);if(connected(u,v))return false;int a=rotate(u),b=rotate(v),x=token(e()),y=token(e());merge(merge(merge(a,x),b),y);edges_[std::minmax(u,v)]={x,y};return true;}
    bool cut(int u,int v){check(u);check(v);auto it=edges_.find(std::minmax(u,v));if(it==edges_.end())return false;auto [x,y]=it->second;int r=rotate(x);int y_position=rank_of(y);auto [first,rest]=split(r,1);auto [middle,last]=split(rest,y_position-1);auto [second,tail]=split(last,1);(void)first;(void)middle;(void)second;(void)tail;edges_.erase(it);return true;}
    S get(int v)const{check(v);return nodes_[v].value;}
    void set(int v,S x){check(v);nodes_[v].value=std::move(x);for(int p=v;p!=-1;p=nodes_[p].parent)pull(p);}
    S component_prod(int v)const{check(v);return nodes_[root_of(v)].prod;}
    int component_size(int v)const{check(v);return nodes_[root_of(v)].vertices;}
    std::size_t allocated_nodes()const{return nodes_.size();}
private:
    static int checked_size(int n){if(n<0)throw std::invalid_argument("negative ETT size");return n;}
};
} // namespace cp
