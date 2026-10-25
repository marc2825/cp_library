#pragma once
#include <limits>
#include <stdexcept>
#include <vector>
#include "graph.hpp"
#include "../range_query/segment_tree_cover.hpp"

namespace cp {
// Snapshot builder. Original vertices keep IDs [0,n); auxiliary arcs have cost zero.
template<class T=long long> class SegmentTreeGraph {
public:
    enum class Kind { Original, IntoRange, OutOfRange, Connector };
    struct VertexInfo { Kind kind; int begin,end,query_id; };
private:
    int n_,queries_=0;
    SegmentTreeCover cover_;
    std::vector<int> down_,up_,edge_query_;
    std::vector<VertexInfo> vertices_;
    std::vector<Edge<T>> edges_;
    static int checked(int n) {if (n<0) throw std::invalid_argument("negative segment graph size"); return n;}
    int vertex(VertexInfo info) {
        if (vertices_.size()>=std::size_t(std::numeric_limits<int>::max())) throw std::length_error("segment graph vertices");
        vertices_.push_back(info); return int(vertices_.size())-1;
    }
    void arc(int u,int v,T cost,int query) {
        if (edges_.size()>=std::size_t(std::numeric_limits<int>::max())) throw std::length_error("segment graph edges");
        edges_.push_back({u,v,cost,int(edges_.size())}); edge_query_.push_back(query);
    }
    void point(int v) const {if (v<0 || v>=n_) throw std::out_of_range("segment graph point");}
    void range(int l,int r) const {if (l<0 || l>r || r>n_) throw std::out_of_range("segment graph range");}
    int query() {if (queries_==std::numeric_limits<int>::max()) throw std::length_error("segment graph queries"); return queries_++;}
public:
    explicit SegmentTreeGraph(int n):n_(checked(n)),cover_(n_),down_(cover_.node_capacity(),-1),up_(cover_.node_capacity(),-1) {
        for (int v=0; v<n_; ++v) {vertex({Kind::Original,v,v+1,-1}); down_[cover_.leaf(v)]=up_[cover_.leaf(v)]=v;}
        for (std::size_t k=cover_.base(); k-->1;) {
            auto [l,r]=cover_.interval(k); if (l==r) continue;
            down_[k]=vertex({Kind::IntoRange,int(l),int(r),-1});
            up_[k]=vertex({Kind::OutOfRange,int(l),int(r),-1});
            for (std::size_t child:{2*k,2*k+1}) if (down_[child]>=0) {
                arc(down_[k],down_[child],T(0),-1); arc(up_[child],up_[k],T(0),-1);
            }
        }
    }
    int original_size() const {return n_;}
    int vertex_count() const {return int(vertices_.size());}
    int query_count() const {return queries_;}
    const std::vector<VertexInfo>& vertex_info() const {return vertices_;}
    const std::vector<int>& edge_query_ids() const {return edge_query_;}
    int add_edge(int u,int v,T cost) {point(u);point(v);int id=query();arc(u,v,cost,id);return id;}
    int add_point_to_range(int u,int l,int r,T cost) {
        point(u);range(l,r);int id=query();for (auto k:cover_.cover(l,r)) arc(u,down_[k],cost,id);return id;
    }
    int add_range_to_point(int l,int r,int v,T cost) {
        range(l,r);point(v);int id=query();for (auto k:cover_.cover(l,r)) arc(up_[k],v,cost,id);return id;
    }
    int add_range_to_range(int l,int r,int a,int b,T cost) {
        range(l,r);range(a,b);int id=query();if (l==r || a==b) return id;
        int v=vertex({Kind::Connector,-1,-1,id});
        for (auto k:cover_.cover(l,r)) arc(up_[k],v,T(0),-1);
        for (auto k:cover_.cover(a,b)) arc(v,down_[k],cost,id);
        return id;
    }
    Graph<T> graph() const {
        Graph<T> g(vertex_count(),GraphKind::Directed);
        for (const auto& e:edges_) g.add_edge(e.from,e.to,e.cost);
        return g;
    }
};
} // namespace cp
