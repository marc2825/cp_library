#pragma once
// 説明: docs/tree/static_top_tree.md
#include <cassert>
#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "heavy_light_decomposition.hpp"
namespace cp {
// 固定された森のrake/compress式。HLDの各chain/rakeを平衡化し高さO(log²N)。
class StaticTopTree {
public:
    enum class Kind { vertex,rake,edge,compress };
    struct Node {Kind kind;int left=-1,right=-1,parent=-1,top=-1,bottom=-1,edge_id=-1;};
    std::vector<Node> nodes;
    std::vector<int> evaluation_order,roots,component_root,edge_node;
    template<class T> explicit StaticTopTree(const Graph<T>& g,int root=0) {
        Graph<int> topology(g.size(),g.kind());for(auto e:g.edges())topology.add_edge(e.from,e.to);
        HeavyLightDecomposition<int> hld(topology,root);int n=g.size();nodes.resize(n,{Kind::vertex});component_root.resize(n);edge_node.resize(g.edge_count(),-1);
        for(int v=0;v<n;++v)nodes[v].top=nodes[v].bottom=v;
        std::vector<int> path_root(n,-1);
        auto make=[&](Kind kind,int a,int b,int id,int boundary){
            int index=static_cast<int>(nodes.size());nodes.push_back({kind,a,b,-1,boundary,boundary,id});
            nodes[a].parent=index;if(b!=-1)nodes[b].parent=index;
            if(kind==Kind::compress){nodes[index].top=nodes[a].top;nodes[index].bottom=nodes[b].bottom;}
            if(id!=-1)edge_node[id]=index;
            evaluation_order.push_back(index);return index;
        };
        std::function<int(const std::vector<int>&,int,int,bool,int)> balanced=[&](const std::vector<int>& items,int l,int r,bool path,int boundary){
            if(r-l==1)return items[l];
            int m=l+(r-l)/2,a=balanced(items,l,m,path,boundary),b=balanced(items,m,r,path,boundary);
            int id=path?hld.tree.parent_edge[nodes[b].top]:-1;
            return make(path?Kind::compress:Kind::rake,a,b,id,boundary);
        };
        for(int v:hld.tree.postorder) {
            std::vector<int> light;
            for(auto e:topology[v])if(hld.tree.parent[e.to]==v && e.to!=hld.heavy[v])light.push_back(make(Kind::edge,path_root[e.to],-1,e.idx,v));
            if(!light.empty()){int p=balanced(light,0,static_cast<int>(light.size()),false,v);nodes[v].left=p;nodes[p].parent=v;}
            evaluation_order.push_back(v);
            if(hld.head[v]==v){std::vector<int> chain;for(int u=v;u!=-1;u=hld.heavy[u])chain.push_back(u);path_root[v]=balanced(chain,0,static_cast<int>(chain.size()),true,v);}
        }
        for(int r:hld.tree.roots)roots.push_back(path_root[r]);
        for(int v=0;v<n;++v)component_root[v]=roots[hld.tree.component[v]];
    }
};
// Spec: Vertex,Path,Point と point_identity/rake/add_vertex/compress/to_point。
// rakeは可換モノイド。compress(P,Q,e)のeは上側bottom→下側top。
template<class Spec,class Weight=long long> class StaticTopTreeDP {
public:
    using Vertex=typename Spec::Vertex;using Path=typename Spec::Path;using Point=typename Spec::Point;
protected:
    Spec spec_;StaticTopTree topology_;std::vector<Vertex> values_;std::vector<Edge<Weight>> edges_;
    std::vector<std::optional<Path>> paths_;std::vector<std::optional<Point>> points_;
    Edge<Weight> connector(int i)const{auto edge=edges_[topology_.nodes[i].edge_id];auto node=topology_.nodes[i];edge.from=node.kind==StaticTopTree::Kind::edge?node.top:topology_.nodes[node.left].bottom;edge.to=topology_.nodes[node.kind==StaticTopTree::Kind::edge?node.left:node.right].top;return edge;}
    void pull(int i){auto node=topology_.nodes[i];using K=StaticTopTree::Kind;
        if(node.kind==K::vertex)paths_[i]=spec_.add_vertex(node.left==-1?spec_.point_identity():*points_[node.left],values_[i],i);
        else if(node.kind==K::rake)points_[i]=spec_.rake(*points_[node.left],*points_[node.right]);
        else if(node.kind==K::edge)points_[i]=spec_.to_point(*paths_[node.left],connector(i));
        else paths_[i]=spec_.compress(*paths_[node.left],*paths_[node.right],connector(i));
    }
    void refresh(int i){for(;i!=-1;i=topology_.nodes[i].parent)pull(i);}
public:
    StaticTopTreeDP(const Graph<Weight>& g,std::vector<Vertex> values,Spec spec=Spec(),int root=0):spec_(std::move(spec)),topology_(g,root),values_(std::move(values)),edges_(g.edges()),paths_(topology_.nodes.size()),points_(topology_.nodes.size()){
        if(values_.size()!=static_cast<std::size_t>(g.size()))throw std::invalid_argument("top tree vertex values size");
        for(int i:topology_.evaluation_order)pull(i);
    }
    int size()const{return static_cast<int>(values_.size());}
    const StaticTopTree& topology()const{return topology_;}
    const Vertex& get_vertex(int v)const{assert(0<=v && v<size());return values_[v];}
    void set_vertex(int v,const Vertex& x){assert(0<=v && v<size());values_[v]=x;refresh(v);}
    void set_edge_cost(int id,const Weight& w){assert(0<=id && id<static_cast<int>(edges_.size()));edges_[id].cost=w;refresh(topology_.edge_node[id]);}
    const Path& component_prod(int v)const{assert(0<=v && v<size());return *paths_[topology_.component_root[v]];}
};
} // namespace cp
