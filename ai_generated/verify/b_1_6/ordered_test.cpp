#include "common.hpp"
#include "ordered_structure/all.hpp"
#include "range_query/range_actions.hpp"
#include <queue>

template<class Tree> void mutable_tree() {
    Tree tree; std::multiset<int> oracle;
    for(int i=0;i<30000;++i) {
        int x=pick(900)-450;
        if(pick(5)<3) { tree.insert(x); oracle.insert(x); }
        else { auto it=oracle.find(x); CHECK(tree.erase(x)==(it!=oracle.end())); if(it!=oracle.end()) oracle.erase(it); }
        CHECK(tree.size()==int(oracle.size())); CHECK(tree.count(x)==int(oracle.count(x)));
        CHECK(tree.order_of_key(x)==std::distance(oracle.begin(),oracle.lower_bound(x)));
        if(!oracle.empty()) { int k=pick(int(oracle.size())); auto it=oracle.begin(); std::advance(it,k); CHECK(tree.kth(k)==*it); }
    }
    for(int x:std::vector<int>(oracle.begin(),oracle.end())) CHECK(tree.erase(x));
    CHECK(tree.size()==0 && !tree.kth(0) && !tree.erase(0));
}
template<class Tree> void invariants() {
    for(int trial=0;trial<40;++trial) {
        Tree tree; std::vector<int> a(350); std::iota(a.begin(),a.end(),0);
        if(trial%3) std::shuffle(a.begin(),a.end(),rng);
        for(int x:a) { tree.insert(x); CHECK(tree.validate()); }
        std::shuffle(a.begin(),a.end(),rng);
        for(int x:a) { CHECK(tree.erase(x)); CHECK(tree.validate()); }
    }
}
template<class Heap> void heap_test() {
    Heap a,b; std::priority_queue<int> expected;
    for(int i=0;i<15000;++i) { int x=pick(2000); if(i%2) a.push(x); else b.push(x); expected.push(x); }
    a.meld(b); CHECK(b.empty());
    while(!expected.empty()) { CHECK(a.top()==expected.top()); a.pop(); expected.pop(); }
    CHECK(a.empty()); a.push(5); Heap c=std::move(a); CHECK(a.empty() && c.top()==5); b=std::move(c); CHECK(c.empty() && b.top()==5);
    b.clear(); CHECK(b.empty());
}
int main() {
    mutable_tree<cp::SplayTree<int>>(); mutable_tree<cp::RedBlackTree<int>>(); mutable_tree<cp::WeightBalancedTree<int>>();
    invariants<cp::RedBlackTree<int>>(); invariants<cp::WeightBalancedTree<int>>();
    cp::PersistentBalancedBST<int> initial; std::vector<cp::PersistentBalancedBST<int>> versions{initial}; std::vector<std::multiset<int>> sets(1);
    for(int i=0;i<5000;++i) {
        int v=pick(int(versions.size())),x=pick(41)-20; auto values=sets[v];
        if(pick(2)) { versions.push_back(versions[v].insert(x)); values.insert(x); }
        else { versions.push_back(versions[v].erase(x)); auto it=values.find(x); if(it!=values.end()) values.erase(it); }
        sets.push_back(values); v=pick(int(versions.size())); CHECK(versions[v].size()==int(sets[v].size()));
        for(int k=0;k<versions[v].size();++k) { auto it=sets[v].begin(); std::advance(it,k); CHECK(versions[v].kth(k)==*it); }
        CHECK(versions[v].count(x)==int(sets[v].count(x))); CHECK(versions[v].order_of_key(x)==std::distance(sets[v].begin(),sets[v].lower_bound(x)));
    }
    heap_test<cp::SkewHeap<int>>(); heap_test<cp::LeftistHeap<int>>();
    cp::ErasablePriorityQueue<int> queue; std::multiset<int> oracle;
    for(int i=0;i<30000;++i) {
        int x=pick(400),op=pick(3);
        if(op==0) { queue.push(x); oracle.insert(x); }
        else if(op==1) { auto it=oracle.find(x); CHECK(queue.erase(x)==(it!=oracle.end())); if(it!=oracle.end()) oracle.erase(it); }
        else if(!oracle.empty()) { CHECK(queue.top()==*oracle.rbegin()); queue.pop(); oracle.erase(std::prev(oracle.end())); }
        CHECK(queue.size()==oracle.size()); if(!oracle.empty()) CHECK(queue.top()==*oracle.rbegin());
    }
    cp::RadixHeap<std::uint64_t,int> radix; std::priority_queue<std::pair<std::uint64_t,int>,std::vector<std::pair<std::uint64_t,int>>,std::greater<std::pair<std::uint64_t,int>>> expected;
    std::uint64_t last=0;
    for(int i=0;i<20000;++i) {
        if(expected.empty() || pick(3)) { auto key=last+unsigned(pick(100000)); radix.push(key,i); expected.emplace(key,i); }
        else { CHECK(radix.top_key()==expected.top().first); last=radix.top_key(); radix.pop(); expected.pop(); }
    }
    using Action=cp::AffineSumAction<long long>;
    cp::LazyBalancedBST<Action::S,Action::op,Action::e,Action::F,Action::mapping,Action::composition,Action::id> lazy(Action::values({1,2,3}));
    lazy.apply(0,3,{2,1}); lazy.reverse(0,3); lazy.insert(1,{10,1}); CHECK(lazy.prod(0,2).sum==17);
    std::cout<<"ordered: 90000 multiset operations, red-black/weight invariants, 5000 persistent branches, meld/move, lazy erasure and monotone radix passed\n";
}
