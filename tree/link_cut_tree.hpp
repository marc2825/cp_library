#pragma once
#include <algorithm>
#include <cassert>
#include <optional>
#include <utility>
#include <vector>

namespace cp {
// 動的な森のlink/cut・頂点set・方向付きパス積。ならしO(log n)。
// opはモノイド。辺の値は辺用頂点を追加して表現。docs/tree/link_cut_tree.md
template<class S,auto op,auto e> class LinkCutTree {
    struct Node { int child[2]={0,0},parent=0,size=1; bool reverse=false; S value=e(),forward=e(),backward=e(); };
    std::vector<Node> nodes_;
    bool aux_root(int x) const { int p=nodes_[x].parent; return !p || (nodes_[p].child[0]!=x && nodes_[p].child[1]!=x); }
    int length(int x) const { return x?nodes_[x].size:0; }
    S product(int x,bool reverse=false) const { return x?(reverse?nodes_[x].backward:nodes_[x].forward):e(); }
    void pull(int x) {
        auto &n=nodes_[x]; int a=n.child[0],b=n.child[1]; n.size=1+length(a)+length(b);
        n.forward=op(op(product(a),n.value),product(b)); n.backward=op(op(product(b,true),n.value),product(a,true));
    }
    void flip(int x) { if(x) { auto &n=nodes_[x]; std::swap(n.child[0],n.child[1]); std::swap(n.forward,n.backward); n.reverse=!n.reverse; } }
    void push(int x) { if(nodes_[x].reverse) { flip(nodes_[x].child[0]); flip(nodes_[x].child[1]); nodes_[x].reverse=false; } }
    void rotate(int x) {
        int p=nodes_[x].parent,g=nodes_[p].parent,d=nodes_[p].child[1]==x,b=nodes_[x].child[d^1];
        if(!aux_root(p)) nodes_[g].child[nodes_[g].child[1]==p]=x;
        nodes_[x].parent=g; nodes_[x].child[d^1]=p; nodes_[p].parent=x;
        nodes_[p].child[d]=b; if(b) nodes_[b].parent=p; pull(p); pull(x);
    }
    void splay(int x) {
        std::vector<int> path{x}; for(int v=x;!aux_root(v);) { v=nodes_[v].parent; path.push_back(v); }
        for(auto it=path.rbegin();it!=path.rend();++it) push(*it);
        while(!aux_root(x)) {
            int p=nodes_[x].parent,g=nodes_[p].parent;
            if(!aux_root(p)) rotate((nodes_[p].child[1]==x)==(nodes_[g].child[1]==p)?p:x);
            rotate(x);
        }
    }
    void access(int x) { int last=0; for(int v=x;v;v=nodes_[v].parent) { splay(v); nodes_[v].child[1]=last; pull(v); last=v; } splay(x); }
    void reroot(int x) { access(x); flip(x); }
    int root(int x) { access(x); push(x); while(nodes_[x].child[0]) { x=nodes_[x].child[0]; push(x); } splay(x); return x; }
    int index(int v) const { assert(0<=v && v<size()); return v+1; }
public:
    explicit LinkCutTree(int n=0):nodes_(checked(n)+1){}
    explicit LinkCutTree(const std::vector<S>& a):LinkCutTree(int(a.size())) { for(int i=0;i<size();++i) nodes_[i+1].value=nodes_[i+1].forward=nodes_[i+1].backward=a[i]; }
    int size() const { return int(nodes_.size())-1; }
    void make_root(int v) { reroot(index(v)); }
    bool connected(int u,int v) { int a=index(u),b=index(v); return a==b || root(a)==root(b); }
    bool link(int u,int v) { int a=index(u),b=index(v); reroot(a); if(root(b)==a) return false; nodes_[a].parent=b; return true; }
    bool cut(int u,int v) {
        int a=index(u),b=index(v); reroot(a); access(b);
        if(nodes_[b].child[0]!=a || nodes_[a].child[1]) return false;
        nodes_[b].child[0]=0; nodes_[a].parent=0; pull(b); return true;
    }
    void set(int v,const S& x) { int k=index(v); access(k); nodes_[k].value=x; pull(k); }
    S get(int v) { int k=index(v); access(k); return nodes_[k].value; }
    std::optional<S> path_prod(int u,int v) { if(!connected(u,v)) return std::nullopt; int a=index(u),b=index(v); reroot(a); access(b); return nodes_[b].forward; }
    int kth_on_path(int u,int v,int k) {
        if(k<0 || !connected(u,v)) return -1;
        int a=index(u),x=index(v); reroot(a); access(x); if(k>=length(x)) return -1;
        for(;;) { push(x); int left=length(nodes_[x].child[0]); if(k==left) { splay(x); return x-1; } if(k<left) x=nodes_[x].child[0]; else { k-=left+1; x=nodes_[x].child[1]; } }
    }
private:
    static int checked(int n) { assert(n>=0); return n; }
};
} // namespace cp
