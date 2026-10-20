#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include "basic_utility/all.hpp"

#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+": " #x); } while (false)
template<class E, class F> void throws(F f) { bool caught=false; try { f(); } catch(const E&) { caught=true; } CHECK(caught); }
std::mt19937_64 rng(20261020);
void windows() {
    for(int trial=0;trial<1500;++trial) {
        std::vector<int> a(rng()%30); for(auto& x:a) x=rng()%8;
        int limit=rng()%25,sum=0,adds=0,removes=0,checks=0;
        auto got=cp::two_pointers(a.size(),[&](std::size_t i){++checks;return sum+a[i]<=limit;},[&](std::size_t i){sum+=a[i];++adds;},[&](std::size_t i){sum-=a[i];++removes;});
        std::vector<std::pair<std::size_t,std::size_t>> expected,actual;
        std::pair<std::size_t,std::size_t> longest{0,0};
        for(std::size_t l=0;l<a.size();++l) {
            int s=0; std::size_t right=l;
            for(std::size_t r=l;r<a.size();++r) {s+=a[r];if(s<=limit){expected.emplace_back(l,r+1);right=r+1;}}
            CHECK(got.right[l]==right);
            if(right-l>longest.second-longest.first)longest={l,right};
        }
        CHECK(got.for_each([&](auto l,auto r){actual.emplace_back(l,r);return true;}));
        CHECK(actual==expected&&got.count()==expected.size()&&got.longest()==longest);
        CHECK(sum==0&&adds==removes&&adds<=int(a.size())&&checks<=2*int(a.size()));
        int called=0;const bool complete=got.for_each([&](auto,auto){++called;return false;});
        CHECK(complete==expected.empty()&&called==!expected.empty());
        std::map<int,int> counts;
        auto distinct=cp::two_pointers(a.size(),[&](auto i){return counts[a[i]]==0;},[&](auto i){++counts[a[i]];},[&](auto i){--counts[a[i]];});
        for(std::size_t l=0;l<a.size();++l){std::set<int>s;auto r=l;while(r<a.size()&&s.insert(a[r]).second)++r;CHECK(distinct.right[l]==r);}
    }
}
void kadane() {
    for(int trial=0;trial<2500;++trial) {
        std::vector<long long>a(rng()%25);for(auto&x:a)x=int(rng()%15)-7;
        for(bool empty:{false,true}) {
            auto actual=cp::maximum_subarray(a,empty);
            std::optional<std::tuple<long long,std::size_t,std::size_t>> best;
            if(empty)best={0,0,0};
            for(std::size_t l=0;l<a.size();++l)for(std::size_t r=l+1;r<=a.size();++r){
                const auto sum=std::accumulate(a.begin()+l,a.begin()+r,0LL);auto key=std::make_tuple(-sum,l,r);
                if(!best||key<*best)best=key;
            }
            CHECK(bool(actual)==bool(best));if(actual)CHECK(std::make_tuple(-actual->sum,actual->left,actual->right)==*best);
        }
    }
    CHECK(cp::maximum_subarray(std::vector<long long>{LLONG_MIN,LLONG_MIN})->sum==LLONG_MIN);
    throws<std::overflow_error>([]{cp::maximum_subarray(std::vector<long long>{LLONG_MAX,1});});
    auto big=cp::maximum_subarray(std::vector<cp::BigInt>{cp::BigInt("100000000000000000000000"),-1,2});
    CHECK(big->sum==cp::BigInt("100000000000000000000001"));
}
void mex() {
    cp::DynamicMex m;std::map<std::int64_t,int> counts;
    for(int i=0;i<20000;++i){
        std::int64_t x=int(rng()%80)-10;if(i%200==0)x=INT64_MAX;if(i%201==0)x=INT64_MIN;
        if(rng()%2){m.insert(x);++counts[x];}else {CHECK(m.erase(x)==(counts[x]>0));if(counts[x])--counts[x];}
        std::uint64_t size=0,want=0;for(auto [key,n]:counts){(void)key;size+=n;}
        while(counts[static_cast<std::int64_t>(want)])++want;
        CHECK(m.mex()==want&&m.count(x)==static_cast<std::uint64_t>(counts[x])&&m.size()==size);
        if(i%999==0){m.clear();counts.clear();CHECK(m.mex()==0&&m.size()==0);}
    }
}
void subsets() {
    for(int trial=0;trial<220;++trial){
        int n=trial%17;std::vector<std::int64_t>a(n);for(auto&x:a)x=int(rng()%19)-9;
        if(trial%17==0&&!a.empty())a[0]=INT64_MAX;
        if(trial%19==0&&!a.empty())a[0]=INT64_MIN;
        std::vector<cp::SubsetSum> sums;
        for(std::uint64_t mask=0;mask<(UINT64_C(1)<<n);++mask){__int128_t sum=0;for(int i=0;i<n;++i)if(mask>>i&1)sum+=a[i];sums.push_back({sum,mask});}
        auto sorted=sums;std::sort(sorted.begin(),sorted.end());auto table=cp::sorted_subset_sums(a);CHECK(table.size()==sums.size());
        for(std::size_t i=0;i<table.size();++i)CHECK(table[i].sum==sorted[i].sum&&table[i].mask==sorted[i].mask);
        cp::MeetInTheMiddle mitm(a);
        for(int q=-30;q<=30;q+=5){std::uint64_t le=0,eq=0,range=0;std::optional<cp::SubsetSum>best;
            for(auto s:sums){le+=s.sum<=q;eq+=s.sum==q;range+=q<=s.sum&&s.sum<q+7;
                if(s.sum<=q&&(!best||s.sum>best->sum||(s.sum==best->sum&&s.mask<best->mask)))best=s;}
            auto actual=mitm.best_leq(q);CHECK(mitm.count_leq(q)==le&&mitm.count_equal(q)==eq&&mitm.count_range(q,q+7)==range);
            CHECK(bool(actual)==bool(best));if(best)CHECK(actual->sum==best->sum&&actual->mask==best->mask);
        }
    }
    cp::MeetInTheMiddle extreme({INT64_MAX,INT64_MAX,INT64_MIN,INT64_MIN});
    CHECK(extreme.count_equal(-2)==1&&extreme.best_leq(static_cast<__int128_t>(INT64_MAX)*2)->sum==static_cast<__int128_t>(INT64_MAX)*2);
    CHECK(extreme.count_equal(std::numeric_limits<__int128_t>::min())==0);
    throws<std::length_error>([]{cp::sorted_subset_sums(std::vector<std::int64_t>(24));});
    throws<std::length_error>([]{cp::MeetInTheMiddle m(std::vector<std::int64_t>(47));});
    throws<std::invalid_argument>([]{cp::MeetInTheMiddle({}).count_range(1,0);});
}
void permutations() {
    for(int n=0;n<=8;++n){std::vector<int>p(n),identity(n);std::iota(p.begin(),p.end(),0);identity=p;
        do {CHECK(cp::is_permutation(p));auto inverse=cp::permutation_inverse(p);CHECK(cp::permutation_compose(p,inverse)==identity);
            int inv=0;for(int i=0;i<n;++i)for(int j=i+1;j<n;++j)inv+=p[i]>p[j];CHECK(cp::permutation_sign(p)==(inv%2?-1:1));
            auto cycles=cp::permutation_cycles(p);std::vector<int>seen(n);int previous=-1;
            for(const auto&cycle:cycles){CHECK(cycle.front()>previous);previous=cycle.front();for(std::size_t i=0;i<cycle.size();++i){CHECK(p[cycle[i]]==cycle[(i+1)%cycle.size()]);++seen[cycle[i]];}}
            CHECK(std::all_of(seen.begin(),seen.end(),[](int x){return x==1;}));
            for(int exponent=-3;exponent<=3;++exponent){auto want=identity;for(int step=0;step<std::abs(exponent);++step)for(int&v:want)v=exponent<0?inverse[v]:p[v];CHECK(cp::permutation_power(p,exponent)==want);}
        }while(std::next_permutation(p.begin(),p.end()));
    }
    CHECK(cp::permutation_power({1,2,0},INT64_MIN)==std::vector<int>({1,2,0}));
    CHECK(!cp::is_permutation({0,0})&&!cp::is_permutation({-1}));
    throws<std::invalid_argument>([]{cp::permutation_inverse({0,0});});
    throws<std::invalid_argument>([]{cp::permutation_compose({0},{});});
}
void searches() {
    for(int n=1;n<=180;++n)for(int l=0;l<n;++l){
        const int r=std::min(n-1,l+int(rng()%5));std::vector<int>a(n);
        for(int i=l-1;i>=0;--i)a[i]=a[i+1]+1+rng()%6;
        for(int i=r+1;i<n;++i)a[i]=a[i-1]+1+rng()%6;
        std::set<int>evaluated;
        auto result=cp::fibonacci_search(-100,n-100,[&](std::int64_t x){CHECK(-100<=x&&x<n-100);CHECK(evaluated.insert(int(x)).second);return a[x+100];});
        CHECK(result.position==l-100&&result.value==0&&result.evaluations==evaluated.size());
        std::uint64_t f=1,g=2;std::size_t k=3;while(g<std::uint64_t(n+1)){auto next=f+g;f=g;g=next;++k;}
        CHECK(result.evaluations<=std::max<std::size_t>(1,k-2));
    }
    for(auto target:{INT64_MIN,INT64_MIN+1,INT64_C(0),INT64_MAX-1}){
        auto result=cp::fibonacci_search(INT64_MIN,INT64_MAX,[&](__int128_t x){return x>target?x-target:target-x;});
        CHECK(result.position==target&&result.value==0&&result.evaluations<100);
    }
    for(int center=-12;center<=12;++center){int calls=0;auto result=cp::golden_section_search(-10,10,[&](long double x){++calls;return std::abs(x-center);},100);
        CHECK(std::abs(result.position-std::max(-10,std::min(10,center)))<1e-12L&&calls==int(result.evaluations)&&calls<=104);
        CHECK(result.left<=result.right);
    }
    CHECK(cp::golden_section_search(2,2,[](long double x){return x*x;}).evaluations==1);
    CHECK(cp::golden_section_search(-3,4,[](long double){return 1;}).position==-3);
    throws<std::invalid_argument>([]{cp::fibonacci_search(0,0,[](auto x){return x;});});
    throws<std::domain_error>([]{cp::golden_section_search(0,1,[](auto){return std::numeric_limits<double>::infinity();});});
    throws<std::invalid_argument>([]{cp::golden_section_search(2,1,[](auto x){return x;});});
}
void hashes() {
    for(std::uint64_t seed:{0,1,19}){
        cp::SetHash<int>set(seed);cp::MultisetHash<int>multi(seed);std::map<int,int>counts;std::set<int>s;
        for(int i=0;i<1500;++i){int key=int(rng()%20)-10;if(rng()%2){CHECK(set.insert(key)==s.insert(key).second);CHECK(multi.insert(key));++counts[key];}
            else {CHECK(set.erase(key)==bool(s.erase(key)));CHECK(multi.erase(key)==(counts[key]>0));if(counts[key])--counts[key];}
            cp::SetHash<int>rebuilt_set(seed);cp::MultisetHash<int>rebuilt_multi(seed);
            for(auto it=s.rbegin();it!=s.rend();++it)rebuilt_set.insert(*it);
            for(auto it=counts.rbegin();it!=counts.rend();++it)for(int k=0;k<it->second;++k)rebuilt_multi.insert(it->first);
            CHECK(set.fingerprint()==rebuilt_set.fingerprint()&&multi.fingerprint()==rebuilt_multi.fingerprint());
            CHECK(set.count(key)==s.count(key)&&multi.count(key)==static_cast<std::uint64_t>(counts[key]));
        }
        set.clear();multi.clear();CHECK(set.size()==0&&multi.size()==0&&set.fingerprint().first==0&&multi.fingerprint().second==0);
    }
    cp::MultisetHash<int>a(0),b(0);a.insert(1);a.insert(1);b.insert(2);b.insert(2);CHECK(a.fingerprint()!=b.fingerprint());
    cp::SetHash<std::string>words(42);words.insert("abc");words.insert("def");CHECK(words.count("abc")==1);
}
using Interval=std::pair<int,int>;
void check_tree(const std::vector<int>& p) {
    cp::PermutationTree tree(p);const auto&nodes=tree.nodes();const int n=int(p.size());
    std::set<Interval>common,strong,represented,actual;
    for(int l=0;l<n;++l)for(int r=l+1;r<=n;++r){auto mm=std::minmax_element(p.begin()+l,p.begin()+r);if(*mm.second-*mm.first==r-l-1)common.emplace(l,r);}
    for(auto a:common){bool overlap=false;for(auto b:common)if((a.first<b.first&&b.first<a.second&&a.second<b.second)||(b.first<a.first&&a.first<b.second&&b.second<a.second))overlap=true;if(!overlap)strong.insert(a);}
    CHECK(tree.common_interval_count()==common.size());CHECK(nodes.size()<=(n?2*p.size()-1:0));
    if(!n){CHECK(tree.root()==-1&&nodes.empty());return;}
    CHECK(nodes[tree.root()].left==0&&nodes[tree.root()].right==n&&nodes[tree.root()].parent==-1);
    for(int id=0;id<int(nodes.size());++id){const auto&v=nodes[id];CHECK(actual.insert({v.left,v.right}).second);represented.emplace(v.left,v.right);
        auto mm=std::minmax_element(p.begin()+v.left,p.begin()+v.right);CHECK(v.low==*mm.first&&v.high==*mm.second+1);
        if(v.type==cp::PermutationTree::Type::Leaf){CHECK(v.right-v.left==1&&v.children.empty());continue;}
        CHECK(v.children.size()>=2);int end=v.left;
        for(auto child:v.children){CHECK(nodes[child].parent==id&&nodes[child].left==end);end=nodes[child].right;}CHECK(end==v.right);
        bool increasing=true,decreasing=true;
        for(std::size_t i=1;i<v.children.size();++i){increasing&=nodes[v.children[i-1]].high==nodes[v.children[i]].low;decreasing&=nodes[v.children[i]].high==nodes[v.children[i-1]].low;}
        CHECK((v.type==cp::PermutationTree::Type::Increasing)==increasing);CHECK((v.type==cp::PermutationTree::Type::Decreasing)==decreasing);
        if(increasing||decreasing)for(std::size_t l=0;l<v.children.size();++l)for(std::size_t r=l;r<v.children.size();++r)represented.emplace(nodes[v.children[l]].left,nodes[v.children[r]].right);
    }
    CHECK(actual==strong&&represented==common);
}
void trees() {
    for(int n=0;n<=8;++n){std::vector<int>p(n);std::iota(p.begin(),p.end(),0);do{check_tree(p);}while(std::next_permutation(p.begin(),p.end()));}
    for(int t=0;t<200;++t){std::vector<int>p(10+rng()%30);std::iota(p.begin(),p.end(),0);std::shuffle(p.begin(),p.end(),rng);check_tree(p);}
    std::vector<int>large(200000);std::iota(large.begin(),large.end(),0);cp::PermutationTree increasing(large);CHECK(increasing.common_interval_count()==20000100000ULL);
    std::reverse(large.begin(),large.end());cp::PermutationTree decreasing(large);CHECK(decreasing.common_interval_count()==increasing.common_interval_count());
    std::shuffle(large.begin(),large.end(),rng);cp::PermutationTree random(large);CHECK(random.nodes().size()<=large.size()*2-1);
    throws<std::invalid_argument>([]{cp::PermutationTree t({1,1});});
}
void brackets() {
    for(int n=0;n<=9;++n){std::vector<std::string>expected;
        for(std::uint64_t mask=0;mask<(UINT64_C(1)<<(2*n));++mask){int depth=0;bool valid=true;std::string text;for(int i=0;i<2*n;++i){bool open=mask>>i&1;depth+=open?1:-1;text+=open?'(':')';if(depth<0)valid=false;}if(valid&&depth==0)expected.push_back(text);}
        std::sort(expected.begin(),expected.end());cp::ParenthesesTable table(n);CHECK(table.count()==cp::BigInt(expected.size()));
        for(std::size_t i=0;i<expected.size();++i){const auto&s=expected[i];CHECK(cp::is_balanced_parentheses(s)&&table.unrank(cp::BigInt(i))==s&&table.rank(s)==cp::BigInt(i));auto tree=cp::parentheses_to_tree(s);CHECK(tree.size()==std::size_t(n+1)&&cp::tree_to_parentheses(tree)==s);}
        throws<std::out_of_range>([&]{table.unrank(table.count());});throws<std::out_of_range>([&]{table.unrank(-1);});
    }
    for(auto s:{"(",")(","(a)","(()","())","\n"}){CHECK(!cp::is_balanced_parentheses(s));throws<std::invalid_argument>([&]{cp::parentheses_to_tree(s);});}
    for(const auto&tree:std::vector<std::vector<std::vector<int>>>{{},{{0}},{{1},{0}},{{1,1},{}},{{},{}},{{-1}}})throws<std::invalid_argument>([&]{cp::tree_to_parentheses(tree);});
    cp::ParenthesesTable large(100);auto last=large.unrank(large.count()-1);CHECK(large.rank(last)==large.count()-1&&last.substr(0,6)=="()()()");
    std::string deep(100000,'(');deep+=std::string(100000,')');CHECK(cp::tree_to_parentheses(cp::parentheses_to_tree(deep))==deep);
}
void partitions() {
    for(int n=0;n<=16;++n){std::vector<std::vector<int>>expected,actual;std::vector<int>path;
        auto generate=[&](auto&&self,int remaining,int cap)->void{if(!remaining){expected.push_back(path);return;}for(int p=std::min(remaining,cap);p>=1;--p){path.push_back(p);self(self,remaining-p,p);path.pop_back();}};
        generate(generate,n,n);CHECK(cp::enumerate_integer_partitions(n,[&](const auto&p){actual.push_back(p);return true;}));CHECK(expected==actual&&cp::integer_partition_count(n)==cp::BigInt(expected.size()));
    }
    for(int n=0;n<=8;++n){std::set<std::vector<int>>expected;std::vector<int>labels(n);
        auto generate=[&](auto&&self,int i,int blocks)->void{if(i==n){expected.insert(labels);return;}for(int block=0;block<=blocks;++block){labels[i]=block;self(self,i+1,std::max(blocks,block+1));}};
        generate(generate,0,0);std::vector<std::vector<int>>actual;
        CHECK(cp::enumerate_set_partitions(n,[&](const auto&rgs){actual.push_back(rgs);auto blocks=cp::partition_blocks(rgs);std::vector<int>back(n,-1);for(std::size_t b=0;b<blocks.size();++b)for(int i:blocks[b])back[i]=int(b);CHECK(back==rgs);return true;}));
        CHECK(actual==std::vector<std::vector<int>>(expected.begin(),expected.end())&&cp::set_partition_count(n)==cp::BigInt(expected.size()));
    }
    int calls=0;CHECK(!cp::enumerate_integer_partitions(100000,[&](const auto&){return ++calls<3;}));CHECK(calls==3);
    calls=0;CHECK(!cp::enumerate_set_partitions(100000,[&](const auto&){return ++calls<3;}));CHECK(calls==3);
    throws<std::invalid_argument>([]{cp::partition_blocks({0,2});});throws<std::invalid_argument>([]{cp::partition_blocks({1});});
    throws<std::invalid_argument>([]{cp::integer_partition_count(-1);});throws<std::invalid_argument>([]{cp::set_partition_count(-1);});
    CHECK(cp::integer_partition_count(100)==190569292&&cp::set_partition_count(10)==115975);
}
void persistence() {
    cp::PartiallyPersistentArray<int>a(15,0);std::vector<std::vector<int>>history(1,std::vector<int>(15));
    for(int t=1;t<=1000;++t){int i=rng()%15;auto next=history.back();next[i]+=rng()%5;CHECK(a.set(i,next[i])==t);history.push_back(next);
        for(int q=0;q<10;++q){int time=rng()%(t+1),index=rng()%15;CHECK(a.get(time,index)==history[time][index]);}}
    for(int i=0;i<15;++i)for(int threshold=0;threshold<200;++threshold){std::optional<std::int64_t>want;for(int t=0;t<=1000;++t)if(history[t][i]>=threshold){want=t;break;}
        auto got=cp::first_true_version(0,a.time()+1,[&](auto t){return a.get(t,i)>=threshold;});CHECK(got==want);}
    int calls=0;CHECK(!cp::first_true_version(5,5,[&](auto){++calls;return true;})&&calls==0);
    CHECK(cp::first_true_version(0,INT64_MAX,[](auto t){return t>=INT64_MAX-1;})==INT64_MAX-1);
    throws<std::out_of_range>([&]{a.get(1001,0);});throws<std::out_of_range>([&]{a.set(15,0);});CHECK(a.time()==1000);
    throws<std::invalid_argument>([]{cp::first_true_version(-1,1,[](auto){return true;});});
    cp::PartiallyPersistentDisjointSetUnion dsu(12);std::vector<std::vector<int>>versions(1,std::vector<int>(12));std::iota(versions[0].begin(),versions[0].end(),0);
    for(int i=0;i<80;++i){int u=rng()%12,v=rng()%12;auto next=versions.back();int before=next[v],after=next[u];for(auto&x:next)if(x==before)x=after;dsu.merge(u,v);versions.push_back(next);}
    for(int u=0;u<12;++u)for(int v=0;v<12;++v){std::optional<int>want;for(int t=0;t<int(versions.size());++t)if(versions[t][u]==versions[t][v]){want=t;break;}CHECK(cp::first_connected_version(dsu,u,v)==want);}
    cp::PartiallyPersistentDisjointSetUnion separate(2);CHECK(!cp::first_connected_version(separate,0,1)&&cp::first_connected_version(separate,0,0)==0);
}
int main(){windows();kadane();mex();subsets();permutations();searches();hashes();trees();brackets();partitions();persistence();std::cout<<"utility search: 12 tasks, exhaustive and randomized oracles passed\n";}
