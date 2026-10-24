#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "wavelet_matrix.hpp"

namespace cp {
// Seaweed / unit-Monge積。構築O(n log² n)、半開区間のstrict LISをO(log n)。
// tko919/library (CC0), commit 3925f75892c0705fb22d6e3f892ae1b5b4fac32d を改変。
// 出典・変更点: docs/static_query/static_range_lis.md
// 同値は位置の逆順にrankを付ける。Compareはstrict weak ordering。
template<class T,class Compare=std::less<T>> class StaticRangeLIS {
    static constexpr int sentinel=std::numeric_limits<int>::max();
    int size_=0,padded_=1;
    WaveletMatrix<int> wavelet_;
public:
    explicit StaticRangeLIS(const std::vector<T>& values={},Compare compare=Compare{}) : size_(int(values.size())) {
        if(values.size()>(1U<<29)) throw std::length_error("range LIS too large");
        while(padded_<size_) padded_*=2;
        std::vector<int> order(size_),permutation(padded_);
        std::iota(order.begin(),order.end(),0);
        std::sort(order.begin(),order.end(),[&](int a,int b) {
            if(compare(values[a],values[b])) return true;
            if(compare(values[b],values[a])) return false;
            return a>b;
        });
        for(int i=0;i<size_;++i) permutation[order[i]]=i;
        for(int i=size_;i<padded_;++i) permutation[i]=i;
        wavelet_=WaveletMatrix<int>(seaweed(permutation));
    }
    int size() const { return size_; }
    int query(int l,int r) const {
        assert(0<=l && l<=r && r<=size_);
        return r-l-wavelet_.range_freq(0,r,l,padded_);
    }
private:
    std::vector<int> seaweed(const std::vector<int>& p){
        int n=int(p.size());
        if(n==1)return {sentinel};
        std::vector<int> lo,hi,lpos,hpos;
        for(int i=0;i<n;++i){
            if(p[i]<n/2){
                lo.push_back(p[i]);
                lpos.push_back(i);
            }
            else{
                hi.push_back(p[i]-n/2);
                hpos.push_back(i);
            }
        }
        auto x=seaweed(lo),y=seaweed(hi);
        std::vector<int> s(n),t(n);
        std::iota(s.begin(),s.end(),0);
        std::iota(t.begin(),t.end(),0);
        int xi=0,yi=0;
        for(int i=0;i<n;++i){
            if(p[i]<n/2){
                if(x[xi]==sentinel)s[i]=sentinel;
                else s[i]=lpos[x[xi]];
                xi++;
            }
            else{
                if(y[yi]==sentinel)t[i]=sentinel;
                else t[i]=hpos[y[yi]];
                yi++;
            }
        }
        std::vector<int> a(n,sentinel),b(n);
        std::vector<int> revs(n,sentinel);
        for(int i=0;i<n;++i)if(s[i]!=sentinel)revs[s[i]]=i;
        int pos=n-1;
        std::vector<int> tos(n,sentinel),tot(n,sentinel);
        for(int i=n-1;i>=0;i--)if(revs[i]!=sentinel){
            a[revs[i]]=pos;
            tos[pos--]=i;
        }
        for(int i=0;i<n;++i)if(a[i]==sentinel)a[i]=pos--;
        pos=0;
        std::vector<int> used(n);
        for(int i=0;i<n;++i)if(t[i]!=sentinel){
            b[pos]=t[i];
            tot[pos++]=i;
            used[t[i]]=1;
        }
        for(int i=0;i<n;++i)if(!used[i]){
            b[pos++]=i;
        }
        auto c=squaredot(a,b);
        std::vector<int> res(n,sentinel);
        for(int i=0;i<n;++i)if(tot[i]!=sentinel and tos[c[i]]!=sentinel){
            res[tot[i]]=tos[c[i]];
        }
        return res;
    }
    std::vector<int> squaredot(const std::vector<int>& a,const std::vector<int>& b){
        int n=int(a.size());
        if(n==1)return {0};
        std::vector<int> alo,ahi,blo,bhi;
        std::vector<int> inva(n);
        for(int i=0;i<n;++i)inva[a[i]]=i;
        int aloi=0,ahii=0;
        std::vector<int> azip(n),alov,ahiv;
        for(int i=0;i<n;++i){
            if(inva[i]<n/2){
                azip[inva[i]]=aloi++;
                alov.push_back(i);
            }
            else{
                azip[inva[i]]=ahii++;
                ahiv.push_back(i);
            }
            if(b[i]<n/2)blo.push_back(b[i]);
            else bhi.push_back(b[i]-n/2);
        }
        for(int i=0;i<n;++i){
            if(i<n/2)alo.push_back(azip[i]);
            else ahi.push_back(azip[i]);
        }
        auto sublo=squaredot(alo,blo);
        auto subhi=squaredot(ahi,bhi);
        std::vector<int> clo(n,sentinel),chi(n,sentinel),res(n,sentinel);
        int subloi=0,subhii=0;
        for(int i=0;i<n;++i){
            if(b[i]<n/2)clo[i]=alov[sublo[subloi++]];
            else chi[i]=ahiv[subhi[subhii++]];
        }
        std::vector<int> invclo(n,sentinel),invchi(n,sentinel);
        for(int i=0;i<n;++i){
            if(clo[i]!=sentinel)invclo[clo[i]]=i;
            if(chi[i]!=sentinel)invchi[chi[i]]=i;
        }
        int cloi=n,cloj=-1,chii=n,chij=-1;
        int ldelta=0,hdelta=0;
        for(;;){
            if(cloi<0 and chij>=n)break;
            if(ldelta>0){
                cloj++;
                if(cloj<n and chi[cloj]!=sentinel and chi[cloj]<cloi)ldelta--;
                if(cloj<n and clo[cloj]!=sentinel and clo[cloj]>=cloi)ldelta--;
            }
            else{
                cloi--;
                if(cloi>=0 and invchi[cloi]!=sentinel and invchi[cloi]<=cloj)ldelta++;
                if(cloi>=0 and invclo[cloi]!=sentinel and invclo[cloi]>cloj)ldelta++;
            }
            if(0<=cloj and cloj<n and clo[cloj]!=sentinel and clo[cloj]<=cloi)res[cloj]=clo[cloj];
            if(hdelta>=0){
                chij++;
                if(chij<n and chi[chij]!=sentinel and chi[chij]<chii)hdelta--;
                if(chij<n and clo[chij]!=sentinel and clo[chij]>=chii)hdelta--;
            }
            else{
                chii--;
                if(chii>=0 and invchi[chii]!=sentinel and invchi[chii]<=chij)hdelta++;
                if(chii>=0 and invclo[chii]!=sentinel and invclo[chii]>chij)hdelta++;
            }
            if(0<=chij and chij<n and chi[chij]!=sentinel and chi[chij]>=chii)res[chij]=chi[chij];
            if(cloi==chii and cloj==chij)res[cloj]=cloi;
        }
        return res;
    }
};
} // namespace cp
