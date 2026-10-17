#include <algorithm>
#include <cstdint>
#include <deque>
#include <iostream>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include "ordered_structure/all.hpp"
#include "static_query/binary_trie.hpp"
#include "static_query/fast_set.hpp"
#include "range_query/range_actions.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("ordered differential mismatch");}
using A=cp::AffineSumAction<long long>;
using Lazy=cp::LazyReversibleTreap<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id>;
std::string concat(std::string a,std::string b){return a+b;} std::string blank(){return {};}
std::string letter_map(int f,std::string s){for(char& c:s)c=char('a'+(c-'a'+f)%26);return s;}
int compose(int f,int g){return(f+g)%26;}int identity(){return 0;}
int main(){
    std::mt19937 rng(987123);
    cp::Treap<int> treap;cp::RBST<int> rbst;cp::OrderedSet<int> unique;std::multiset<int> expected;std::set<int> distinct;
    cp::BinaryTrie<std::uint64_t> trie;std::vector<std::uint64_t> values;
    for(int step=0;step<10000;++step){
        int x=int(rng()%501)-250;
        if(rng()%2){treap.insert(x);rbst.insert(x);expected.insert(x);check(unique.insert(x)==distinct.insert(x).second);}
        else{bool found=expected.find(x)!=expected.end();check(treap.erase(x)==found&&rbst.erase(x)==found);if(found)expected.erase(expected.find(x));check(unique.erase(x)==bool(distinct.erase(x)));}
        std::vector<int> a(expected.begin(),expected.end()),b(distinct.begin(),distinct.end());
        check(treap.size()==int(a.size())&&rbst.size()==int(a.size())&&unique.size()==int(b.size()));
        int k=a.empty()?0:int(rng()%a.size());check(treap.kth(k)==rbst.kth(k));if(!a.empty())check(treap.kth(k)==a[k]);
        int less=int(std::lower_bound(a.begin(),a.end(),x)-a.begin());check(treap.order_of_key(x)==less&&rbst.order_of_key(x)==less);
        if(!b.empty())check(unique.kth(int(rng()%b.size())).has_value());
        std::uint64_t y=(std::uint64_t(rng())<<32)|rng();int action=rng()%4;
        if(action==0){trie.insert(y);values.push_back(y);}else if(action==1){trie.xor_all(y);for(auto& v:values)v^=y;}
        else if(action==2&&!values.empty()){auto p=rng()%values.size();check(trie.erase(values[p]));values.erase(values.begin()+p);}
        else{bool found=std::find(values.begin(),values.end(),y)!=values.end();check(trie.erase(y)==found);if(found)values.erase(std::find(values.begin(),values.end(),y));}
        std::sort(values.begin(),values.end());check(trie.size()==static_cast<long long>(values.size()));
        if(!values.empty()){
            auto p=rng()%values.size();check(trie.kth(p)==values[p]);auto b=values;std::sort(b.begin(),b.end(),[&](auto u,auto v){return (u^y)<(v^y);});check(trie.kth_xor(p,y)==b[p]);
        }
        check(trie.order_of_key(y)==std::lower_bound(values.begin(),values.end(),y)-values.begin());
    }
    cp::BinaryTrie<unsigned,4> duplicates;duplicates.insert(0);duplicates.insert(15);duplicates.insert(15);duplicates.xor_all(15);check(duplicates.count(0)==2&&duplicates.count(15)==1);
    check(!duplicates.kth(-1)&&!duplicates.kth(3));
    cp::Treap<int,std::greater<int>> descending;descending.insert(2);descending.insert(4);check(descending.kth(0)==4&&descending.order_of_key(2)==1);
    for(int n:{0,1,63,64,65,4095,4096,4097,200000}){
        cp::FastSet set(n);std::set<int> oracle;
        for(int step=0;step<2000;++step){
            if(n){int x=rng()%n;if(rng()%2)check(set.insert(x)==oracle.insert(x).second);else check(set.erase(x)==bool(oracle.erase(x)));}
            int x=int(rng()%(n+101))-50;auto it=oracle.lower_bound(x);check(set.next(x)==(it==oracle.end()?n:*it));
            it=oracle.upper_bound(x);check(set.prev(x)==(it==oracle.begin()?-1:*std::prev(it)));check(set.size()==int(oracle.size()));
        }
        for(int x:std::vector<int>(oracle.begin(),oracle.end())) { check(set.erase(x)); }
        check(set.next(0)==n&&set.prev(n)==-1);
    }
    Lazy lazy;std::vector<long long> a;
    for(int step=0;step<10000;++step){
        int n=int(a.size()),l=rng()%(n+1),r=rng()%(n+1);if(l>r)std::swap(l,r);
        int action=rng()%7;
        if(action==0||n==0){long long x=int(rng()%31)-15;lazy.insert(l,{x,1});a.insert(a.begin()+l,x);}
        else if(action==1){int p=rng()%n;lazy.erase(p);a.erase(a.begin()+p);}
        else if(action==2){lazy.reverse(l,r);std::reverse(a.begin()+l,a.begin()+r);}
        else if(action==3){A::F f{int(rng()%3)-1,int(rng()%11)-5};lazy.apply(l,r,f);for(int i=l;i<r;++i)a[i]=f.a*a[i]+f.b;}
        else if(action==4){int p=rng()%n;lazy.set(p,{7,1});a[p]=7;}
        else if(action==5){int p=rng()%(n-(r-l)+1);lazy.move(l,r,p);auto b=std::vector<long long>(a.begin()+l,a.begin()+r);a.erase(a.begin()+l,a.begin()+r);a.insert(a.begin()+p,b.begin(),b.end());}
        else{long long sum=0;for(int i=l;i<r;++i)sum+=a[i];check(lazy.prod(l,r).sum==sum);}
        auto actual=lazy.to_vector();check(actual.size()==a.size());for(int i=0;i<int(a.size());++i)check(actual[i].sum==a[i]&&actual[i].length==1);
    }
    cp::LazyReversibleTreap<std::string,concat,blank,int,letter_map,compose,identity> text(std::vector<std::string>{"a","b","c","d"});
    text.apply(0,4,1);text.reverse(0,4);text.apply(1,3,2);check(text.all_prod()=="efeb");check(text.prod(0,3)=="efe");
    cp::PersistentStack<int> stack;cp::PersistentQueue<int> queue;
    std::vector<int> qr{0};std::vector<std::deque<int>> versions(1);
    for(int step=0;step<20000;++step){
        int b=rng()%versions.size();auto v=versions[b];
        if(v.empty()||rng()%3){int x=int(rng());qr.push_back(queue.push(qr[b],x));v.push_back(x);}
        else{qr.push_back(queue.pop(qr[b]));v.pop_front();}
        versions.push_back(v);check(queue.size(qr.back())==int(v.size()));
        if(!v.empty())check(queue.front(qr.back())==v.front()&&queue.back(qr.back())==v.back());
    }
    std::vector<int> roots{0};std::vector<std::vector<int>> stacks(1);
    for(int step=0;step<20000;++step){int b=rng()%roots.size();auto v=stacks[b];if(v.empty()||rng()%2){int x=int(rng());roots.push_back(stack.push(roots[b],x));v.push_back(x);}else{roots.push_back(stack.pop(roots[b]));v.pop_back();}stacks.push_back(v);check(stack.size(roots.back())==int(v.size()));if(!v.empty())check(stack.top(roots.back())==v.back());}
    int q=0;for(int i=0;i<100000;++i)q=queue.push(q,i);for(int i=0;i<100000;++i){check(queue.front(q)==i);q=queue.pop(q);}check(queue.empty(q));
    std::cout<<"ordered: 10000 tree/trie/sequence operations, FastSet boundaries, 40000 persistent branches and 100000 queue elements passed\n";
}
