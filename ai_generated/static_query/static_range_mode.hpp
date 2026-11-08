#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>
#include "../ordered_structure/coordinate_compression.hpp"
namespace cp {
// 同頻度ではCompare順の最小値を返す。中央の完全blockと両端を組み合わせる。
template<class T,class Compare=std::less<T>> class StaticRangeMode {
    CoordinateCompression<T,Compare> values_;
    std::vector<int> a_;
    std::vector<std::vector<int>> positions_,best_;
    int block_,blocks_;
    int frequency(int key,int l,int r) const {
        const auto& p=positions_[key];
        return int(std::lower_bound(p.begin(),p.end(),r)-std::lower_bound(p.begin(),p.end(),l));
    }
public:
    explicit StaticRangeMode(const std::vector<T>& a,Compare compare=Compare{},int block_size=0)
        : values_(a,compare),a_(a.size()),positions_(values_.size()) {
        assert(block_size>=0);block_=block_size?block_size:std::max(1,int(std::sqrt(a.size()+1)));
        blocks_=(size()+block_-1)/block_;best_.assign(blocks_,std::vector<int>(blocks_,-1));
        for(int i=0;i<size();++i) {a_[i]=*values_.index(a[i]);positions_[a_[i]].push_back(i);}
        for(int b=0;b<blocks_;++b) {
            std::vector<int> count(values_.size());int best=-1;
            for(int i=b*block_;i<size();++i) {
                int x=a_[i];++count[x];
                if(best==-1 || count[x]>count[best] || (count[x]==count[best] && x<best)) best=x;
                best_[b][i/block_]=best;
            }
        }
    }
    int size() const {return int(a_.size());}
    std::optional<std::pair<T,int>> mode(int l,int r) const {
        assert(0<=l && l<=r && r<=size());if(l==r)return std::nullopt;
        int first=(l+block_-1)/block_,last=r/block_,winner=-1,count=0;
        auto consider=[&](int key){int f=frequency(key,l,r);if(f>count || (f==count && key<winner)){winner=key;count=f;}};
        if(first<last) {
            consider(best_[first][last-1]);
            for(int i=l;i<first*block_;++i) consider(a_[i]);
            for(int i=last*block_;i<r;++i) consider(a_[i]);
        }else for(int i=l;i<r;++i)consider(a_[i]);
        return std::make_pair(values_.value(winner),count);
    }
};
} // namespace cp
