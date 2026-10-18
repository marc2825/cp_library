#pragma once
#include <climits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../union_find/disjoint_set_union.hpp"
namespace cp {
class GraphicMatroid {
    int vertices_;std::vector<std::pair<int,int>> edges_;
public:
    GraphicMatroid(int vertices,std::vector<std::pair<int,int>> edges):vertices_(vertices),edges_(std::move(edges)){
        if(vertices<0)throw std::invalid_argument("negative graphic matroid vertex count");
        if(edges_.size()>static_cast<std::size_t>(INT_MAX))throw std::length_error("too many matroid elements");
        for(auto[u,v]:edges_)if(u<0||v<0||u>=vertices||v>=vertices)throw std::out_of_range("graphic matroid edge endpoint");
    }
    int size()const{return int(edges_.size());}
    bool independent(const std::vector<int>&elements)const{
        DisjointSetUnion dsu(vertices_);for(int e:elements){if(e<0||e>=size())throw std::out_of_range("graphic matroid element");const auto[u,v]=edges_[e];if(!dsu.merge(u,v))return false;}return true;
    }
    bool operator()(const std::vector<int>&elements)const{return independent(elements);}
};
} // namespace cp
