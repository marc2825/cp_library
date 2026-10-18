#pragma once
#include <climits>
#include <stdexcept>
#include <utility>
#include <vector>
namespace cp {
class PartitionMatroid {
    std::vector<int> group_,capacity_;
public:
    PartitionMatroid(std::vector<int> groups,std::vector<int> capacities):group_(std::move(groups)),capacity_(std::move(capacities)){
        if(group_.size()>static_cast<std::size_t>(INT_MAX))throw std::length_error("too many matroid elements");
        for(int c:capacity_)if(c<0)throw std::invalid_argument("negative partition capacity");
        for(int g:group_)if(g<0||static_cast<std::size_t>(g)>=capacity_.size())throw std::out_of_range("partition group");
    }
    int size()const{return int(group_.size());}
    bool independent(const std::vector<int>&elements)const{
        std::vector<int> count(capacity_.size());std::vector<unsigned char> seen(group_.size());
        for(int e:elements){if(e<0||e>=size())throw std::out_of_range("partition matroid element");if(seen[e])return false;seen[e]=1;const int group=group_[e];if(count[group]==capacity_[group])return false;++count[group];}return true;
    }
    bool operator()(const std::vector<int>&elements)const{return independent(elements);}
};
} // namespace cp
