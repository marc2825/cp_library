#pragma once
// 説明: docs/dp_optimization/min_plus_convolution_b.md
#include <algorithm>
#include <deque>
#include <utility>
#include <vector>
#include "../number_theory_convolution/detail/arithmetic.hpp"

namespace cp {
namespace min_plus_detail {
template<class T> T sum(T a,const T& b) { detail::arithmetic_add_assign(a,b); return a; }
}
// c[k]=min_{i+j=k}(a[i]+b[j])。両列が離散凸ならO(n+m)。
template<class T> std::vector<T> min_plus_convolution_convex_convex(const std::vector<T>& a,const std::vector<T>& b) {
    if(a.empty() || b.empty()) return {};
    std::vector<T> c; c.reserve(a.size()+b.size()-1); std::size_t i=0,j=0; c.push_back(min_plus_detail::sum(a[0],b[0]));
    while(i+1<a.size() || j+1<b.size()) {
        if(j+1==b.size() || (i+1<a.size() && min_plus_detail::sum(a[i+1],b[j])<min_plus_detail::sum(a[i],b[j+1]))) ++i; else ++j;
        c.push_back(min_plus_detail::sum(a[i],b[j]));
    }
    return c;
}
// aは任意、bは離散凸。帯状Monge行列の単調最小位置を直接探索、O((n+m)log(n+m))。
template<class T> std::vector<T> min_plus_convolution_arbitrary_convex(const std::vector<T>& a,const std::vector<T>& b) {
    if(a.empty() || b.empty()) return {};
    int n=int(a.size()),m=int(b.size()); std::vector<T> c(n+m-1,T(0));
    auto solve=[&](auto&& self,int top,int bottom,int left,int right)->void {
        if(top>=bottom) return;
        int k=top+(bottom-top)/2,lo=std::max(left,k-m+1),hi=std::min(right,k+1),best=lo;
        T value=min_plus_detail::sum(a[lo],b[k-lo]);
        for(int i=lo+1;i<hi;++i) { T v=min_plus_detail::sum(a[i],b[k-i]); if(v<value) { value=v; best=i; } }
        c[k]=value; self(self,top,k,left,best+1); self(self,k+1,bottom,best,right);
    };
    solve(solve,0,n+m-1,0,n); return c;
}
template<class T> std::vector<T> min_plus_convolution_convex_arbitrary(const std::vector<T>& a,const std::vector<T>& b) { return min_plus_convolution_arbitrary_convex(b,a); }
// 両列が離散凹なら、反対角線の最小は有効区間のどちらかの端にある。
template<class T> std::vector<T> min_plus_convolution_concave_concave(const std::vector<T>& a,const std::vector<T>& b){
    if(a.empty() || b.empty())return {};
    std::vector<T> c;c.reserve(a.size()+b.size()-1);for(std::size_t k=0;k<a.size()+b.size()-1;++k){auto left=k<b.size()?0:k-b.size()+1,right=std::min(k,a.size()-1);c.push_back(std::min(min_plus_detail::sum(a[left],b[k-left]),min_plus_detail::sum(a[right],b[k-right])));}return c;
}
// Max-Plusの凹+凹は傾きの大きい順にmerge。
template<class T> std::vector<T> max_plus_convolution_concave_concave(const std::vector<T>& a,const std::vector<T>& b){
    if(a.empty() || b.empty())return {};
    std::vector<T> c;c.reserve(a.size()+b.size()-1);std::size_t i=0,j=0;c.push_back(min_plus_detail::sum(a[0],b[0]));while(i+1<a.size() || j+1<b.size()){if(j+1==b.size() || (i+1<a.size() && min_plus_detail::sum(a[i],b[j+1])<min_plus_detail::sum(a[i+1],b[j])))++i;else ++j;c.push_back(min_plus_detail::sum(a[i],b[j]));}return c;
}
// aは任意、bは離散凹。aをb.size()以下のblockに分け、前後の部分包絡を使う。
// O(n log(m+1)+m)時間、O(n+m)領域。符号反転は行わない。
template<class T> std::vector<T> min_plus_convolution_arbitrary_concave(const std::vector<T>& a,const std::vector<T>& b){
    if(a.empty() || b.empty())return {};
    int m=static_cast<int>(b.size());std::vector<T> reversed_b(b.rbegin(),b.rend()),result(a.size()+b.size()-1,T(0));std::vector<bool> filled(result.size());
    auto half=[&](const std::vector<T>& part,const std::vector<T>& kernel){
        std::deque<std::pair<int,int>> envelope;std::vector<T> values;values.reserve(m);
        auto value=[&](int row,int col){return min_plus_detail::sum(part[col],kernel[row-col]);};
        for(int row=0;row<m;++row){
            while(!envelope.empty() && envelope.front().second<row)envelope.pop_front();
            if(row<static_cast<int>(part.size())){
                while(!envelope.empty() && !(value(envelope.front().second,envelope.front().first)<value(envelope.front().second,row)))envelope.pop_front();
                if(envelope.empty())envelope.emplace_front(row,m-1);
                else{int lo=row,hi=envelope.front().second;while(lo<hi){int mid=lo+(hi-lo)/2;if(value(mid,envelope.front().first)<value(mid,row))hi=mid;else lo=mid+1;}if(lo>row)envelope.emplace_front(row,lo-1);}
            }
            values.push_back(value(row,envelope.front().first));
        }
        return values;
    };
    for(std::size_t start=0;start<a.size();start+=b.size()){
        auto length=std::min(b.size(),a.size()-start);std::vector<T> part(a.begin()+start,a.begin()+start+length);auto left=half(part,b);std::reverse(part.begin(),part.end());auto right=half(part,reversed_b);
        auto save=[&](std::size_t index,const T& value){if(!filled[index] || value<result[index]){filled[index]=true;result[index]=value;}};
        for(int k=0;k<m;++k){save(start+static_cast<std::size_t>(k),left[k]);save(start+length+b.size()-2-static_cast<std::size_t>(k),right[k]);}
    }
    return result;
}
template<class T> std::vector<T> min_plus_convolution_concave_arbitrary(const std::vector<T>& a,const std::vector<T>& b){return min_plus_convolution_arbitrary_concave(b,a);}
} // namespace cp
