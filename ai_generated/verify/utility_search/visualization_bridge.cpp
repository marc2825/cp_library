#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include "basic_utility/all.hpp"
#include "range_query/monoid_catalog.hpp"
template<class T>void array(const std::vector<T>& a){std::cout<<'[';bool first=true;for(const auto&v:a){if(!first)std::cout<<',';first=false;std::cout<<v;}std::cout<<']';}
template<class T>void nested(const std::vector<std::vector<T>>& a){std::cout<<'[';bool first=true;for(const auto&v:a){if(!first)std::cout<<',';first=false;array(v);}std::cout<<']';}
int main(){
    {std::vector<int>a{2,1,3,1,2};int sum=0;auto w=cp::two_pointers(a.size(),[&](auto i){return sum+a[i]<=4;},[&](auto i){sum+=a[i];},[&](auto i){sum-=a[i];});auto [l,r]=w.longest();std::cout<<"two_pointers {\"right\":";array(w.right);std::cout<<",\"longest\":["<<l<<','<<r<<"],\"count\":"<<w.count()<<"}\n";}
    {auto a=cp::maximum_subarray(std::vector<int>{-2,3,-1,3,-5});std::cout<<"maximum_subarray {\"sum\":"<<a->sum<<",\"left\":"<<a->left<<",\"right\":"<<a->right<<"}\n";}
    {cp::DynamicMex m;std::vector<std::uint64_t>history;for(auto [op,x]:std::vector<std::pair<int,int>>{{1,0},{1,1},{1,1},{0,1},{1,3},{0,0}}){if(op)m.insert(x);else m.erase(x);history.push_back(m.mex());}std::cout<<"dynamic_mex {\"mex\":"<<m.mex()<<",\"size\":"<<m.size()<<",\"history\":";array(history);std::cout<<"}\n";}
    {cp::MeetInTheMiddle m({3,-2,5,1});auto b=m.best_leq(6);std::cout<<"meet_in_the_middle {\"best\":{\"sum\":"<<static_cast<long long>(b->sum)<<",\"mask\":"<<b->mask<<"},\"countEqual\":"<<m.count_equal(6)<<",\"countLeq\":"<<m.count_leq(6)<<"}\n";}
    {std::vector<int>p{2,0,1,4,3};std::cout<<"permutation {\"inverse\":";array(cp::permutation_inverse(p));std::cout<<",\"cycles\":";nested(cp::permutation_cycles(p));std::cout<<",\"sign\":"<<cp::permutation_sign(p)<<",\"power\":";array(cp::permutation_power(p,-2));std::cout<<"}\n";}
    {auto a=cp::fibonacci_search(-10,11,[](std::int64_t x){return std::abs(x-3);});std::cout<<"unimodal_search {\"position\":"<<a.position<<",\"value\":"<<a.value<<"}\n";}
    {cp::MultisetHash<std::int64_t> m(42);m.insert(1);m.insert(1);m.insert(2);m.erase(1);auto f=m.fingerprint();std::cout<<"collection_hash {\"seed\":\""<<f.seed<<"\",\"size\":"<<f.size<<",\"first\":\""<<f.first<<"\",\"second\":\""<<f.second<<"\"}\n";}
    {cp::PermutationTree t({1,0,3,5,2,4});std::vector<std::tuple<int,int,std::string>>nodes;for(const auto&v:t.nodes()){std::string type=v.type==cp::PermutationTree::Type::Leaf?"leaf":v.type==cp::PermutationTree::Type::Increasing?"increasing":v.type==cp::PermutationTree::Type::Decreasing?"decreasing":"prime";nodes.emplace_back(v.left,v.right,type);}std::sort(nodes.begin(),nodes.end());std::cout<<"permutation_tree {\"count\":"<<t.common_interval_count()<<",\"nodes\":[";bool first=true;for(const auto&[l,r,type]:nodes){if(!first)std::cout<<',';first=false;std::cout<<'['<<l<<','<<r<<",\""<<type<<"\"]";}std::cout<<"]}\n";}
    {cp::ParenthesesTable t(3);auto s=t.unrank(3);std::cout<<"parentheses {\"count\":\""<<t.count()<<"\",\"text\":\""<<s<<"\",\"rank\":\""<<t.rank(s)<<"\",\"children\":";nested(cp::parentheses_to_tree(s));std::cout<<"}\n";}
    {std::vector<std::vector<int>>p;cp::enumerate_integer_partitions(5,[&](const auto&a){p.push_back(a);return true;});std::cout<<"integer_partitions {\"count\":\""<<cp::integer_partition_count(5)<<"\",\"partitions\":";nested(p);std::cout<<",\"more\":false}\n";}
    {std::vector<std::vector<int>>p;cp::enumerate_set_partitions(3,[&](const auto&a){p.push_back(a);return true;});std::cout<<"set_partitions {\"count\":\""<<cp::set_partition_count(3)<<"\",\"partitions\":";nested(p);std::cout<<",\"more\":false}\n";}
    {cp::PartiallyPersistentArray<int>a(2,0);a.set(0,2);a.set(1,5);a.set(0,7);auto first=cp::first_true_version(0,a.time()+1,[&](auto t){return a.get(t,0)>=4;});std::vector<std::vector<int>>history;for(int t=0;t<=a.time();++t)history.push_back({a.get(t,0),a.get(t,1)});std::cout<<"partial_persistent_search {\"time\":"<<a.time()<<",\"first\":"<<*first<<",\"history\":";nested(history);std::cout<<"}\n";}
    {cp::RetroactiveOperations<cp::AffineFunctionMonoid<long long>> t({10,20,30});std::vector<long long> h;
        t.set(10,{2,1});h.push_back(t.all()(1));t.set(30,{3,0});h.push_back(t.all()(1));t.set(20,{1,4});h.push_back(t.all()(1));t.erase(10);h.push_back(t.all()(1));
        std::cout<<"retroactive_operations {\"size\":"<<t.size()<<",\"before\":\""<<t.before(20)(1)<<"\",\"through\":\""<<t.through(20)(1)<<"\",\"current\":\""<<t.all()(1)<<"\",\"history\":[";
        for(std::size_t i=0;i<h.size();++i){if(i)std::cout<<',';std::cout<<'"'<<h[i]<<'"';}std::cout<<"]}\n";}
    {bool first=true;std::cout<<"special_integer_enumeration {\"values\":[";cp::enumerate_palindromic_integers(90,140,[&](auto x){if(!first)std::cout<<',';first=false;std::cout<<'"'<<x<<'"';return true;});std::cout<<"],\"more\":false}\n";}
}
