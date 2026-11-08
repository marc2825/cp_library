#include <algorithm>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include "ordered_structure/double_ended_priority_queue.hpp"
#include "static_query/static_range_mex.hpp"
#include "static_query/range_majority.hpp"
#include "range_query/point_set_range_frequency.hpp"
#include "range_query/segment_tree_cover.hpp"
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << "\n"; std::abort(); } } while (false)
struct Key { int v; explicit Key(int x):v(x){} };
struct Less { bool reverse; bool operator()(const Key& a,const Key& b) const { return reverse?a.v>b.v:a.v<b.v; } };
int main() {
    std::mt19937 rng(721865);
    cp::DoubleEndedPriorityQueue<int> q;
    std::vector<int> sorted;
    for (int i=0;i<100000;++i) {
        if (sorted.empty() || rng()%2) {
            int x=int(rng()%31)-15; q.push(x); sorted.insert(std::lower_bound(sorted.begin(),sorted.end(),x),x);
        } else if (rng()%2) { q.pop_min(); sorted.erase(sorted.begin()); }
        else { q.pop_max(); sorted.pop_back(); }
        CHECK(q.size()==sorted.size()); CHECK(q.empty()==sorted.empty());
        if (!q.empty()) { CHECK(q.min()==sorted.front()); CHECK(q.max()==sorted.back()); }
    }
    auto copy=q; q.clear(); CHECK(q.empty()); CHECK(copy.size()==sorted.size());
    cp::DoubleEndedPriorityQueue<Key,Less> reverse(Less{true}); reverse.push(Key(1)); reverse.push(Key(8));
    CHECK(reverse.min().v==8 && reverse.max().v==1);
    std::size_t intervals=0;
    for (int n=0;n<=8;++n) {
        int total=1; for (int i=0;i<n;++i) total*=3;
        for (int code=0;code<total;++code) {
            int v=code; std::vector<int> a(n); for (int& x:a) { x=v%3; v/=3; }
            cp::StaticRangeMex mex(a); cp::RangeMajority<int> majority(a);
            for (int l=0;l<=n;++l) for (int r=l;r<=n;++r) {
                ++intervals; std::map<int,int> count; for (int i=l;i<r;++i) ++count[a[i]];
                int m=0; while (count.count(m)) ++m; CHECK(mex.mex(l,r)==m);
                std::optional<int> want;
                for (auto [x,c]:count) if (c>(r-l)/2) want=x;
                CHECK(majority.majority(l,r)==want);
                if (want) CHECK(majority.candidate(l,r)==want);
                for (int x=-1;x<=3;++x) CHECK(majority.frequency(l,r,x)==count[x]);
            }
        }
    }
    cp::StaticRangeMex extremes(std::vector<long long>{LLONG_MIN,LLONG_MAX,-1,0,1});
    CHECK(extremes.mex(0,5)==2 && extremes.mex(0,3)==0);
    cp::StaticRangeMex unsigned_values(std::vector<unsigned long long>{ULLONG_MAX,0,1});
    CHECK(unsigned_values.mex(0,3)==2);
    for (int n=0;n<=70;++n) {
        std::vector<int> a(n); cp::PointSetRangeFrequency<int> dynamic(a);
        for (int t=0;t<2000;++t) {
            if (n && rng()%2) { int p=rng()%n,x=int(rng()%41)-20; a[p]=x; dynamic.set(p,x); CHECK(dynamic.get(p)==x); }
            int l=rng()%(n+1),r=rng()%(n+1),x=int(rng()%41)-20; if (l>r) std::swap(l,r);
            CHECK(dynamic.frequency(l,r,x)==std::count(a.begin()+l,a.begin()+r,x));
        }
    }
    cp::RangeMajority<Key,Less> keys({Key(4),Key(1),Key(4)},Less{true}); CHECK(keys.majority(0,3)->v==4);
    cp::PointSetRangeFrequency<Key,Less> keyed({Key(1),Key(3)},Less{true}); keyed.set(1,Key(1)); CHECK(keyed.frequency(0,2,Key(1))==2);
    cp::RangeMajority<std::string> words({"a","b","a"}); CHECK(words.majority(0,3)=="a");
    for (int n=0;n<=96;++n) {
        cp::SegmentTreeCover c(n);
        for (int l=0;l<=n;++l) for (int r=l;r<=n;++r) {
            auto nodes=c.cover(l,r); int end=l;
            std::vector<std::size_t> callback; c.for_each(l,r,[&](auto k){callback.push_back(k);}); CHECK(nodes==callback);
            for (auto k:nodes) { auto [a,b]=c.interval(k); CHECK(a==end && a<b && b<=r); end=b; }
            CHECK(end==r); CHECK(nodes.size()<=16);
        }
        for (int p=0;p<n;++p) CHECK(c.interval(c.leaf(p))==std::make_pair(p,p+1));
    }
    cp::SegmentTreeCover huge(INT_MAX); CHECK(huge.interval(1)==std::make_pair(0,INT_MAX));
    int end=1; for (auto k:huge.cover(1,INT_MAX)) { auto [l,r]=huge.interval(k); CHECK(l==end); end=r; } CHECK(end==INT_MAX);
    std::vector<int> big(200000); std::iota(big.begin(),big.end(),0);
    cp::StaticRangeMex big_mex(big); CHECK(big_mex.mex(0,200000)==200000 && big_mex.mex(1,200000)==0);
    cp::RangeMajority<int> big_majority(big); CHECK(!big_majority.majority(0,200000));
    std::fill(big.begin(),big.end(),1); cp::PointSetRangeFrequency<int> big_dynamic(big);
    for (int p=0;p<200000;p+=2) big_dynamic.set(p,2);
    CHECK(big_dynamic.frequency(0,200000,1)==100000);
    std::cout << "query tests: 100000 queue operations, " << intervals << " exhaustive intervals, 142000 frequency operations, covers and 200000-element cases passed\n";
}
