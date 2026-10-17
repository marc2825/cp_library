#pragma once
#include <cassert>
#include <stdexcept>
#include <utility>
#include <vector>
#include "mo.hpp"
#include "../tree/doubling_lca.hpp"

namespace cp {
// 木/森の頂点パスクエリ。Euler列の出現回数の偶奇で頂点をtoggle。
class MoOnTree {
    DoublingLCA<int> lca_; Mo mo_; std::vector<int> tour_,in_,out_,extra_;
    template<class T> static Graph<int> unweighted(const Graph<T>& g) {
        if(g.directed()) throw std::invalid_argument("MoOnTree requires undirected forest");
        Graph<int> h(g.size()); for(const auto& e:g.edges()) h.add_edge(e.from,e.to,1); return h;
    }
public:
    template<class T> explicit MoOnTree(const Graph<T>& g,int root=0):lca_(unweighted(g),root),mo_(2*g.size()),in_(g.size()),out_(g.size()) {
        for(int r:lca_.tree.roots) {
            std::vector<std::pair<int,bool>> stack{{r,false}};
            while(!stack.empty()) {
                auto [v,exit]=stack.back(); stack.pop_back();
                if(exit) { out_[v]=int(tour_.size()); tour_.push_back(v); continue; }
                in_[v]=int(tour_.size()); tour_.push_back(v); stack.emplace_back(v,true);
                for(auto it=g[v].rbegin();it!=g[v].rend();++it)
                    if(lca_.tree.parent[it->to]==v) stack.emplace_back(it->to,false);
            }
        }
    }
    int add_query(int u,int v) {
        int w=lca_.lca(u,v); if(w==-1) throw std::invalid_argument("vertices in different components");
        if(in_[u]>in_[v]) std::swap(u,v);
        int id;
        if(w==u) { id=mo_.add_query(in_[u],in_[v]+1); extra_.push_back(-1); }
        else { id=mo_.add_query(out_[u],in_[v]+1); extra_.push_back(w); }
        return id;
    }
    template<class Add,class Remove,class Answer> void run(Add add,Remove remove,Answer answer,int block_size=0) const {
        std::vector<bool> active(in_.size(),false);
        auto toggle_vertex=[&](int v) { if(active[v]) remove(v); else add(v); active[v]=!active[v]; };
        auto toggle=[&](int p) { toggle_vertex(tour_[p]); };
        mo_.run(toggle,toggle,[&](int id) {
            if(extra_[id]!=-1) toggle_vertex(extra_[id]);
            answer(id);
            if(extra_[id]!=-1) toggle_vertex(extra_[id]);
        },block_size);
    }
};
} // namespace cp
