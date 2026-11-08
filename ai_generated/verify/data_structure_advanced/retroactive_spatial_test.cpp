#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <set>
#include <vector>
#include "../../ordered_structure/partially_retroactive_priority_queue.hpp"
#include "../../range_query/dynamic_nearest_neighbor.hpp"
#define CHECK(x) do {if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
std::mt19937 rng(353134);
struct Event{int type=0,key=0;};
template<class Compare> std::optional<std::multiset<int,Compare>> replay(const std::vector<Event>& history){
    std::multiset<int,Compare> q;
    for(auto e:history){if(e.type==1)q.insert(e.key);else if(e.type==-1){if(q.empty())return std::nullopt;q.erase(q.begin());}}
    return q;
}
template<class Compare> void retroactive(){
    std::vector<long long> times;for(int i=0;i<50;++i)times.push_back(i*7-100);
    cp::PartiallyRetroactivePriorityQueue<int,long long,Compare> queue(times);std::vector<Event> history(times.size());
    for(int step=0;step<50000;++step){int p=rng()%times.size(),kind=rng()%3,value=int(rng()%31)-15;auto next=history;bool occupied=history[p].type!=0;
        if(kind==0)next[p]={1,value};else if(kind==1)next[p]={-1,0};else next[p]={};
        auto expected=replay<Compare>(next);bool valid=bool(expected) && (kind==2?occupied:!occupied);bool result=kind==0?queue.insert(times[p],value):kind==1?queue.insert_delete_min(times[p]):queue.erase(times[p]);CHECK(result==valid);if(valid)history=next;
        auto now=replay<Compare>(history);CHECK(now);CHECK(queue.size()==int(now->size()));CHECK(queue.empty()==now->empty());if(now->empty())CHECK(!queue.min());else CHECK(queue.min()==*now->begin());for(int i=0;i<50;++i)CHECK(queue.contains_operation(times[i])==(history[i].type!=0));
    }
    bool threw=false;try{queue.insert(-99,0);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
}
void exhaustive(){
    // 全てのvalidな短い履歴を構築し、任意の1操作を削除する。
    for(int length=0;length<=7;++length){int count=1;for(int i=0;i<length;++i)count*=3;std::vector<long long> times(length);for(int i=0;i<length;++i)times[i]=i;
        for(int mask=0;mask<count;++mask){std::vector<Event> history(length);int code=mask;for(auto& e:history){int x=code%3;code/=3;e=x==0?Event{-1,0}:Event{1,x};}auto expected=replay<std::less<int>>(history);if(!expected)continue;
            cp::PartiallyRetroactivePriorityQueue<int> original(times);for(int i=0;i<length;++i)CHECK(history[i].type==1?original.insert(i,history[i].key):original.insert_delete_min(i));
            for(int p=0;p<length;++p){auto q=original;auto h=history;h[p]={};auto result=replay<std::less<int>>(h);CHECK(q.erase(p)==bool(result));if(result){CHECK(q.size()==int(result->size()));if(result->empty())CHECK(!q.min());else CHECK(q.min()==*result->begin());}}
        }
    }
}
template<std::size_t D> void spatial(){
    using Tree=cp::DynamicNearestNeighbor<D>;using Point=typename Tree::Point;Tree tree;std::vector<std::optional<Point>> points;
    for(int step=0;step<3000;++step){Point point;for(auto& x:point)x=int(rng()%61)-30;
        if(points.empty() || rng()%3==0){CHECK(tree.insert(point)==int(points.size()));points.push_back(point);}
        else {int id=rng()%points.size();bool active=bool(points[id]);if(rng()%2){CHECK(tree.erase(id)==active);points[id].reset();}else{CHECK(tree.move(id,point)==active);if(active)points[id]=point;}}
        CHECK(tree.size()==std::count_if(points.begin(),points.end(),[](auto& x){return bool(x);}));
        for(int q=0;q<3;++q){for(auto& x:point)x=int(rng()%101)-50;std::optional<typename Tree::Result> best;for(int id=0;id<int(points.size());++id)if(points[id]){long double d=0;for(std::size_t k=0;k<D;++k){auto x=point[k]-(*points[id])[k];d+=x*x;}if(!best || d<best->squared_distance)best=typename Tree::Result{id,d};}auto actual=tree.nearest(point);CHECK(bool(actual)==bool(best));if(best)CHECK(actual->id==best->id && actual->squared_distance==best->squared_distance);}
        if(step%61==0)tree.rebuild();
    }
    Point zero{};auto first=tree.insert(zero);tree.insert(zero);CHECK(tree.nearest(zero)->id<=first);bool threw=false;zero[0]=std::numeric_limits<long double>::infinity();try{tree.insert(zero);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
}
void large_history(){
#ifdef _GLIBCXX_DEBUG
    constexpr int n=1000;
#else
    constexpr int n=50000;
#endif
    std::vector<long long> times(2*n);for(int i=0;i<2*n;++i)times[i]=i;
    cp::PartiallyRetroactivePriorityQueue<int> queue(times);
    for(int i=0;i<n;++i)CHECK(queue.insert(i,n-i));
    for(int i=0;i<n;++i)CHECK(queue.insert_delete_min(n+i));
    CHECK(queue.empty());
    for(int i=n-1;i>=0;--i){CHECK(queue.erase(n+i));CHECK(queue.min()==i+1);}
    CHECK(queue.size()==n);
}
int main(){large_history();retroactive<std::less<int>>();retroactive<std::greater<int>>();exhaustive();spatial<1>();spatial<2>();spatial<3>();spatial<5>();spatial<10>();std::cout<<"retroactive and spatial OK\n";}
