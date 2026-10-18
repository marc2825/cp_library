#pragma once
// 説明: docs/static_query/sqrt_tree.md
#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

namespace cp {
// 静的モノイド。O(n log log n)構築/領域、O(1) prod。更新なし。
template<class S,auto op,auto e> class SqrtTree {
    struct Layer { int log,half,blocks; std::vector<S> prefix,suffix,between; };
    int n_,base_=1; std::vector<S> values_; std::vector<Layer> layers_; std::vector<int> lookup_;
public:
    explicit SqrtTree(const std::vector<S>& a={}) : n_(int(a.size())) {
        int log=0; while(base_<n_) { base_*=2; ++log; }
        values_=a; values_.resize(base_,e()); lookup_.resize(log);
        for(int k=log;k>1;k=(k+1)/2) {
            int half=(k+1)/2,width=1<<half,blocks=1<<(k-half),index=int(layers_.size());
            for(int bit=half;bit<k;++bit) lookup_[bit]=index;
            Layer layer{k,half,blocks,std::vector<S>(base_,e()),std::vector<S>(base_,e()),std::vector<S>(base_,e())};
            for(int start=0;start<base_;start+=(1<<k)) {
                for(int b=0;b<blocks;++b) {
                    int l=start+b*width,r=l+width; S p=e(),s=e();
                    for(int i=l;i<r;++i) layer.prefix[i]=p=op(p,values_[i]);
                    for(int i=r-1;i>=l;--i) layer.suffix[i]=s=op(values_[i],s);
                }
                for(int l=0;l<blocks;++l) {
                    S p=e();
                    for(int r=l;r<blocks;++r) layer.between[start+l*blocks+r]=p=op(p,layer.prefix[start+(r+1)*width-1]);
                }
            }
            layers_.push_back(std::move(layer));
        }
    }
    int size() const { return n_; }
    S prod(int l,int r) const {
        assert(0<=l && l<=r && r<=n_); if(l==r) return e(); if(r-l==1) return values_[l];
        unsigned diff=unsigned(l^(r-1)); int bit=31-__builtin_clz(diff);
        if(bit==0) return op(values_[l],values_[r-1]);
        const auto& layer=layers_[lookup_[bit]]; int base=(l>>layer.log)<<layer.log;
        int a=(l-base)>>layer.half,b=(r-1-base)>>layer.half;
        S result=layer.suffix[l];
        if(a+1<b) result=op(result,layer.between[base+(a+1)*layer.blocks+b-1]);
        return op(result,layer.prefix[r-1]);
    }
};
} // namespace cp
