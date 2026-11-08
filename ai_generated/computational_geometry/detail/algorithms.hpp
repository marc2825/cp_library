#pragma once
#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>
#include "../common.hpp"
namespace cp { namespace geometry { namespace detail {
template<class K>std::vector<typename K::P> hull(std::vector<typename K::P> p,bool keep,K k){
    for(auto a:p)k.validate(a);
    std::sort(p.begin(),p.end());p.erase(std::unique(p.begin(),p.end()),p.end());if(p.size()<3)return p;
    bool collinear=true;for(std::size_t i=2;i<p.size();++i)if(k.turn(p[0],p[1],p[i]))collinear=false;
    if(collinear)return keep?p:std::vector<typename K::P>{p.front(),p.back()};
    const auto build=[&](auto first,auto last){std::vector<typename K::P> chain;for(;first!=last;++first){while(chain.size()>1&&(keep?k.turn(chain[chain.size()-2],chain.back(),*first)<0:k.turn(chain[chain.size()-2],chain.back(),*first)<=0))chain.pop_back();chain.push_back(*first);}return chain;};
    auto lower=build(p.begin(),p.end()),upper=build(p.rbegin(),p.rend());lower.pop_back();upper.pop_back();lower.insert(lower.end(),upper.begin(),upper.end());return lower;
}
template<class K>Location polygon_contains(const std::vector<typename K::P>&p,typename K::P q,K k){
    k.validate(q);for(auto a:p)k.validate(a);std::ptrdiff_t winding=0;
    for(std::size_t i=0;i<p.size();++i){const auto a=p[i],b=p[(i+1)%p.size()];if(k.on({a,b},q))return Location::boundary;const int turn=k.turn(a,b,q);if(a.y<=q.y&&b.y>q.y&&turn>0)++winding;if(a.y>q.y&&b.y<=q.y&&turn<0)--winding;}
    return winding?Location::inside:Location::outside;
}
// 正規化済みのstrict CCW凸包。全頂点を毎回検査しない。
template<class K>Location convex_contains(const std::vector<typename K::P>&p,typename K::P q,K k){
    k.validate(q);const auto n=p.size();if(!n)return Location::outside;if(n==1)return k.same(p[0],q)?Location::boundary:Location::outside;if(n==2)return k.on({p[0],p[1]},q)?Location::boundary:Location::outside;
    const int first=k.turn(p[0],p[1],q),last=k.turn(p[0],p.back(),q);if(first<0||last>0)return Location::outside;if(!first)return k.on({p[0],p[1]},q)?Location::boundary:Location::outside;if(!last)return k.on({p[0],p.back()},q)?Location::boundary:Location::outside;
    std::size_t left=1,right=n-1;while(right-left>1){const auto mid=left+(right-left)/2;if(k.turn(p[0],p[mid],q)>=0)left=mid;else right=mid;}
    const int side=k.turn(p[left],p[right],q);return side>0?Location::inside:!side&&k.on({p[left],p[right]},q)?Location::boundary:Location::outside;
}
template<class Metric>struct PairResult{Metric squared_distance;std::size_t first,second;};
template<class K>std::optional<PairResult<typename K::Metric>> diameter(const std::vector<typename K::P>&p,K k){
    for(auto a:p)k.validate(a);
    if(p.empty())return std::nullopt;
    if(p.size()==1)return PairResult<typename K::Metric>{0,0,0};
    PairResult<typename K::Metric> best{k.distance2(p[0],p[1]),0,1};const auto update=[&](std::size_t a,std::size_t b){if(a==b)return;const auto d=k.distance2(p[a],p[b]);if(d>best.squared_distance){if(a>b)std::swap(a,b);best={d,a,b};}};
    if(p.size()==2)return best;
    for(std::size_t i=0,j=1;i<p.size();++i){const auto next=(i+1)%p.size();const auto area=[&](std::size_t at){return k.magnitude(k.side(p[i],p[next],p[at]));};while(area((j+1)%p.size())>area(j))j=(j+1)%p.size();update(i,j);update(next,j);if(area((j+1)%p.size())==area(j)){update(i,(j+1)%p.size());update(next,(j+1)%p.size());}}
    return best;
}
template<class K>std::optional<PairResult<typename K::Metric>> closest(const std::vector<typename K::P>&points,K k){
    using P=typename K::P;using M=typename K::Metric;struct Entry{P point;std::size_t index;};std::vector<Entry>a;for(std::size_t i=0;i<points.size();++i){k.validate(points[i]);a.push_back({points[i],i});}if(a.size()<2)return std::nullopt;
    std::sort(a.begin(),a.end(),[](const Entry&a,const Entry&b){return a.point<b.point||(!(b.point<a.point)&&a.index<b.index);});for(std::size_t i=1;i<a.size();++i)if(a[i-1].point==a[i].point)return PairResult<M>{0,std::min(a[i-1].index,a[i].index),std::max(a[i-1].index,a[i].index)};
    PairResult<M>best{k.distance2(a[0].point,a[1].point),std::min(a[0].index,a[1].index),std::max(a[0].index,a[1].index)};std::vector<Entry>work(a.size());const auto by_y=[](const Entry&a,const Entry&b){return a.point.y!=b.point.y?a.point.y<b.point.y:a.point.x<b.point.x;};const auto update=[&](const Entry&u,const Entry&v){const auto d=k.distance2(u.point,v.point);if(d<best.squared_distance)best={d,std::min(u.index,v.index),std::max(u.index,v.index)};};
    std::function<void(std::size_t,std::size_t)>solve=[&](std::size_t left,std::size_t right){if(right-left<=3){for(std::size_t i=left;i<right;++i)for(std::size_t j=i+1;j<right;++j)update(a[i],a[j]);std::sort(a.begin()+left,a.begin()+right,by_y);return;}const auto mid=left+(right-left)/2;const auto split=a[mid].point.x;solve(left,mid);solve(mid,right);std::merge(a.begin()+left,a.begin()+mid,a.begin()+mid,a.begin()+right,work.begin()+left,by_y);std::copy(work.begin()+left,work.begin()+right,a.begin()+left);std::size_t count=0;for(std::size_t i=left;i<right;++i)if(k.axis2(a[i].point.x,split)<best.squared_distance){for(std::size_t j=count;j>0;--j){const auto& candidate=work[left+j-1];if(k.axis2(a[i].point.y,candidate.point.y)>=best.squared_distance)break;update(a[i],candidate);}work[left+count++]=a[i];}};
    solve(0,a.size());return best;
}
} } } // namespace cp::geometry::detail
