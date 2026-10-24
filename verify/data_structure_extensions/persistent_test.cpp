#include <algorithm>
#include <climits>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include "ordered_structure/persistent_leftist_heap.hpp"
#include "range_query/dynamic_segtree.hpp"
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << "\n"; std::abort(); } } while (false)
std::string cat(std::string a,std::string b) { return a+b; }
std::string empty_string() { return {}; }
long long sum(long long a,long long b) { return a+b; }
long long zero() { return 0; }
struct Key { int v; explicit Key(int x):v(x){} };
struct Compare { bool operator()(const Key& a,const Key& b) const { return a.v>b.v; } };
int main() {
    std::mt19937 rng(8163);
    cp::PersistentLeftistHeap<int,std::greater<int>> heap;
    std::vector<int> roots{0}; std::vector<std::multiset<int>> oracle(1);
    for (int t=0;t<12000;++t) {
        int a=rng()%roots.size(),b=rng()%roots.size(),root; auto next=oracle[a];
        int op=rng()%3;
        if (op==0 || next.empty()) { int x=int(rng()%101)-50; root=heap.push(roots[a],x); next.insert(x); }
        else if (op==1 || next.size()+oracle[b].size()>256) { root=heap.pop(roots[a]); next.erase(next.begin()); }
        else { root=heap.meld(roots[a],roots[b]); next.insert(oracle[b].begin(),oracle[b].end()); }
        roots.push_back(root); oracle.push_back(next);
        for (int i:{a,b,int(roots.size())-1}) {
            CHECK(heap.size(roots[i])==oracle[i].size()); CHECK(heap.empty(roots[i])==oracle[i].empty());
            if (!oracle[i].empty()) CHECK(heap.top(roots[i])==*oracle[i].begin());
        }
        if (t%97==0) { auto copy=next; while (!copy.empty()) { CHECK(heap.top(root)==*copy.begin()); root=heap.pop(root); copy.erase(copy.begin()); } CHECK(!root); }
    }
    cp::PersistentLeftistHeap<Key,Compare> custom; int k=custom.push(custom.push(0,Key(7)),Key(2)); CHECK(custom.top(k).v==2);
    std::vector<int> a(200000); std::iota(a.begin(),a.end(),0);
    cp::PersistentLeftistHeap<int> bulk; int root=bulk.build(a.begin(),a.end()); CHECK(bulk.node_count()==a.size());
    CHECK(bulk.size(root)==a.size() && bulk.top(root)==199999);
    for (int i=199999;i>=0;--i) { CHECK(bulk.top(root)==i); root=bulk.pop(root); } CHECK(root==0);
    cp::PersistentLeftistHeap<int> doubling; root=doubling.push(0,1);
    for (unsigned i=1;i<sizeof(std::size_t)*CHAR_BIT;++i) root=doubling.meld(root,root);
    bool overflow=false; try { (void)doubling.meld(root,root); } catch (const std::length_error&) { overflow=true; } CHECK(overflow);
    using Tree=cp::DynamicSegtree<std::string,cat,empty_string>;
    std::vector<Tree> trees; std::vector<std::map<int,std::string>> maps;
    trees.emplace_back(31); maps.emplace_back();
    for (int t=0;t<6000;++t) {
        int i=rng()%trees.size(),op=rng()%4; auto tree=trees[i]; auto m=maps[i];
        if (op==0) { int p=rng()%31; std::string s(1,char('a'+rng()%3)); tree.set(p,s); m[p]=s; }
        if (op==1) { int p=rng()%32; auto parts=tree.split_copy(p); auto right=tree.split(p); CHECK(tree.all_prod()==parts.first.all_prod() && right.all_prod()==parts.second.all_prod()); m.erase(m.lower_bound(p),m.end()); }
        if (op==2) {
            int j=rng()%trees.size(); auto other=trees[j];
            auto combine=[](const auto& x,const auto& y){ return x.substr(0,2)+y.substr(0,2); };
            auto merged=tree.merged(other,combine); tree.merge(other,combine); CHECK(!other.node_count()); CHECK(tree.all_prod()==merged.all_prod());
            for (const auto& [p,s]:maps[j]) { auto it=m.find(p); if (it==m.end()) m[p]=s; else it->second=combine(it->second,s); }
        }
        if (op==3) { tree.clear(); m.clear(); }
        for (int l=0;l<=31;++l) {
            int r=l+rng()%(32-l); std::string want;
            for (auto it=m.lower_bound(l);it!=m.end() && it->first<r;++it) want+=it->second;
            CHECK(tree.prod(l,r)==want);
        }
        trees.push_back(std::move(tree)); maps.push_back(std::move(m));
        for (int p=0;p<31;++p) { auto it=maps[i].find(p); CHECK(trees[i].get(p)==(it==maps[i].end()?"":it->second)); }
    }
    Tree s(4); s.set(1,"a"); auto saved=s;
    bool threw=false; try { s.merge(s); } catch (const std::invalid_argument&) { threw=true; } CHECK(threw && s.get(1)=="a");
    Tree wrong(3); threw=false; try { s.merge(wrong); } catch (const std::invalid_argument&) { threw=true; } CHECK(threw);
    Tree other(4); other.set(1,"b"); threw=false;
    try { s.merge(other,[](auto,auto)->std::string{throw std::runtime_error("merge");}); } catch (const std::runtime_error&) { threw=true; }
    CHECK(threw && s.get(1)=="a" && other.get(1)=="b" && saved.get(1)=="a");
    Tree explicit_zero(4); explicit_zero.set(2,""); Tree also_zero(4); also_zero.set(2,""); int calls=0;
    CHECK(explicit_zero.merged(also_zero,[&](auto,auto){++calls;return "x";}).get(2)=="x" && calls==1);
    cp::DynamicSegtree<long long,sum,zero> huge(LLONG_MAX); huge.set(LLONG_MAX-1,7); huge.set(0,3);
    auto tail=huge.split(LLONG_MAX-1); CHECK(huge.all_prod()==3 && tail.get(LLONG_MAX-1)==7); huge.merge(tail); CHECK(huge.all_prod()==10);
    cp::DynamicSegtree<long long,sum,zero> none(0); auto none2=none.split(0); none.merge(none2); CHECK(none.all_prod()==0 && !none.node_count());
    std::cout << "persistent tests: 12000 heap branches, 200000-element build/drain, size overflow, 6000 tree branches, noncommutative merge, ownership and exceptions passed\n";
}
