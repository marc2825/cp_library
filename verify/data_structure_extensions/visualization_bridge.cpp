#include <functional>
#include <iostream>
#include <vector>
#include "ordered_structure/double_ended_priority_queue.hpp"
#include "static_query/static_range_mex.hpp"
#include "static_query/range_majority.hpp"
#include "range_query/point_set_range_frequency.hpp"
#include "ordered_structure/persistent_leftist_heap.hpp"
#include "range_query/dynamic_segtree.hpp"
#include "range_query/segment_tree_cover.hpp"
#include "offline_query/segment_tree_over_time.hpp"
#include "union_find/rollback_disjoint_set_union_with_data.hpp"
#include "basic_utility/rollback_array.hpp"
long long sum(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::DoubleEndedPriorityQueue<int> q;for (int x:{4,1,4,9})q.push(x);q.pop_max();q.pop_min();
    std::cout<<"double_ended_priority_queue {\"min\":"<<q.min()<<",\"max\":"<<q.max()<<",\"size\":"<<q.size()<<"}\n";
    cp::StaticRangeMex mex(std::vector<int>{0,1,0,3,2});std::cout<<"static_range_mex ["<<mex.mex(0,5)<<','<<mex.mex(1,4)<<','<<mex.mex(2,2)<<"]\n";
    cp::RangeMajority<int> majority({2,1,2,2,3,1});std::cout<<"range_majority ["<<*majority.majority(0,4)<<','<<(majority.majority(0,6)?"0":"null")<<"]\n";
    cp::PointSetRangeFrequency<int> f({2,1,2,3});int first=f.frequency(0,4,2);f.set(1,2);f.set(3,100);
    std::cout<<"point_set_range_frequency {\"values\":[";for(int i=0;i<4;++i)std::cout<<(i?",":"")<<f.get(i);std::cout<<"],\"answers\":["<<first<<','<<f.frequency(0,3,2)<<"]}\n";
    cp::PersistentLeftistHeap<int,std::greater<int>> h;std::vector<int> roots{0};roots.push_back(h.push(0,4));roots.push_back(h.push(roots[1],1));roots.push_back(h.push(roots[1],7));roots.push_back(h.meld(roots[2],roots[3]));roots.push_back(h.pop(roots[4]));
    std::cout<<"persistent_leftist_heap {\"tops\":[null";for(std::size_t i=1;i<roots.size();++i)std::cout<<','<<h.top(roots[i]);std::cout<<"],\"sizes\":[";for(std::size_t i=0;i<roots.size();++i)std::cout<<(i?",":"")<<h.size(roots[i]);std::cout<<"]}\n";
    using Tree=cp::DynamicSegtree<long long,sum,zero>;std::vector<Tree> v;v.emplace_back(10);v.push_back(v[0]);v.back().set(2,5);v.push_back(v[1]);v.back().set(8,7);auto parts=v[2].split_copy(5);v.push_back(parts.first);v.push_back(parts.second);v.push_back(v[3]);v.back().set(2,9);v.push_back(v[5].merged(v[4]));
    std::cout<<"dynamic_segtree_merge_split {\"totals\":[";for(std::size_t i=0;i<v.size();++i)std::cout<<(i?",":"")<<v[i].all_prod();std::cout<<"]}\n";
    cp::SegmentTreeCover c(8);auto nodes=c.cover(1,7);std::cout<<"segment_tree_cover {\"nodes\":[";for(std::size_t i=0;i<nodes.size();++i)std::cout<<(i?",":"")<<nodes[i];std::cout<<"],\"intervals\":[";for(std::size_t i=0;i<nodes.size();++i){auto [l,r]=c.interval(nodes[i]);std::cout<<(i?",":"")<<'['<<l<<','<<r<<']';}std::cout<<"]}\n";
    cp::SegmentTreeOverTime<std::pair<int,int>> t(4);t.add(0,3,{0,1});t.add(1,2,{1,2});cp::RollbackDisjointSetUnion d(3);std::cout<<"segment_tree_over_time [";t.run(d,[](auto& s,auto e){s.merge(e.first,e.second);},[](auto& s,int time){std::cout<<(time?",":"")<<s.component_count();});std::cout<<"]\n";
    cp::RollbackArray<int> data(std::vector<int>{2,3,7});cp::RollbackDisjointSetUnionWithData<cp::RollbackArray<int>> ds(3,data);auto snap=ds.snapshot();int before=ds.component_count();ds.merge(0,1,[](auto& a,int x,int y){a.set(x,a[x]+a[y]);});int during=ds.component_count();ds.rollback(snap);
    std::cout<<"rollback_disjoint_set_union_with_data {\"components\":"<<ds.component_count()<<",\"sums\":["<<data[0]<<','<<data[1]<<','<<data[2]<<"],\"history\":["<<before<<','<<during<<','<<ds.component_count()<<"]}\n";
}
