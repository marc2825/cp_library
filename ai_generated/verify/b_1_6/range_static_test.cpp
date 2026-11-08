#include "common.hpp"
#include "range_query/all.hpp"
#include "static_query/all.hpp"
#include <limits>

using A=cp::AffineSumAction<long long>;
using Persistent=cp::PersistentLazySegtree<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id>;
struct CountedKey {
    int value; static long long comparisons;
    CountedKey()=delete;
    explicit CountedKey(int x) : value(x) {}
    friend bool operator<(const CountedKey& a,const CountedKey& b) { ++comparisons; return a.value<b.value; }
};
long long CountedKey::comparisons=0;
static long long operations_count=0;
long long counted_sum(long long a,long long b) { ++operations_count; return a+b; }
int main() {
    for(int trial=0;trial<180;++trial) {
        int n=pick(90); std::vector<std::string> a(n); for(auto& x:a) x=char('a'+pick(5));
        cp::RollbackSegtree<std::string,cat,empty_string> rollback(a);
        auto original=a; auto state=rollback.snapshot();
        for(int t=0;t<250;++t) {
            if(n && pick(2)) { int p=pick(n); a[p]=char('a'+pick(5)); rollback.set(p,a[p]); }
            int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r); std::string expected; for(int i=l;i<r;++i) expected+=a[i]; CHECK(rollback.prod(l,r)==expected);
            if(t==120) { rollback.rollback(state); a=original; }
        }
        std::vector<long long> values(n); for(auto& x:values) x=pick(11)-5;
        Persistent tree(A::values(values)); std::vector<int> roots{tree.initial_root()}; std::vector<std::vector<long long>> versions{values};
        for(int t=0;t<200;++t) {
            int v=pick(int(roots.size())),l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r);
            A::F f{pick(3)-1,pick(9)-4}; roots.push_back(tree.apply(roots[v],l,r,f)); auto next=versions[v]; for(int i=l;i<r;++i) next[i]=f.a*next[i]+f.b; versions.push_back(next);
            v=pick(int(roots.size())); auto before=tree.node_count(); long long want=std::accumulate(versions[v].begin()+l,versions[v].begin()+r,0LL); CHECK(tree.prod(roots[v],l,r).sum==want); CHECK(tree.node_count()==before);
        }
        std::vector<int> keys(n); std::iota(keys.begin(),keys.end(),0); std::shuffle(keys.begin(),keys.end(),rng);
        std::vector<std::pair<int,std::string>> items; for(int k:keys) items.emplace_back(k,std::to_string(k)+",");
        cp::SortableSegtree<std::string,cat,empty_string> sortable(items,std::max(1,3*n));
        for(int t=0;t<200;++t) {
            int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r);
            if(n && t%3==0) {
                int p=pick(n),key=pick(3*n);
                while(std::any_of(items.begin(),items.end(),[&](const auto& item){return item.first==key && item.first!=items[p].first;})) key=pick(3*n);
                items[p]={key,std::to_string(p)+":"}; sortable.set(p,key,items[p].second);
            }
            else if(pick(2)) { bool desc=pick(2); sortable.sort(l,r,desc); std::sort(items.begin()+l,items.begin()+r,[=](const auto& x,const auto& y){return desc?x.first>y.first:x.first<y.first;}); }
            std::string want; for(int i=l;i<r;++i) want+=items[i].second; CHECK(sortable.prod(l,r)==want);
            if(n) { int p=pick(n); CHECK(sortable.get(p)==items[p]); }
        }
    }
    for(int n=0;n<180;++n) {
        std::vector<int> a(n); std::vector<long long> weights(n); std::vector<std::string> text(n);
        for(int i=0;i<n;++i) { a[i]=pick(11)-5; weights[i]=pick(21)-10; text[i]=char('a'+pick(5)); }
        cp::LinearRMQ<int> rmq(a); cp::SqrtTree<std::string,cat,empty_string> sqrt(text);
        cp::WaveletTree<int> wavelet(a); cp::SegtreeFractionalCascading<int> fc(a); cp::WeightedWaveletMatrix<int> wm(a,weights);
        for(int t=0;t<600;++t) {
            int l=pick(n+1),r=pick(n+1); if(l>r) std::swap(l,r); int low=pick(16)-8,high=pick(16)-8; if(low>high) std::swap(low,high);
            int count=0,less=0; long long sum=0,sumless=0; std::string s; std::vector<int> ids;
            for(int i=l;i<r;++i) { count+=low<=a[i] && a[i]<high; less+=a[i]<low; if(a[i]<low) sumless+=weights[i]; if(low<=a[i] && a[i]<high) sum+=weights[i]; s+=text[i]; ids.push_back(i); }
            CHECK(wavelet.range_freq(l,r,low,high)==count && fc.range_freq(l,r,low,high)==count && wavelet.count_less(l,r,low)==less);
            CHECK(wm.range_sum(l,r,low,high)==sum && wm.sum_less(l,r,low)==sumless && sqrt.prod(l,r)==s);
            int expected=l==r?-1:int(std::min_element(a.begin()+l,a.begin()+r)-a.begin()); CHECK(rmq.argmin(l,r)==expected);
            std::stable_sort(ids.begin(),ids.end(),[&](int x,int y){return a[x]<a[y];}); int k=pick(r-l+1); long long small=0; for(int i=0;i<k;++i) small+=weights[ids[i]]; CHECK(wm.sum_smallest(l,r,k)==small);
            if(l<r) { int rank=pick(r-l); CHECK(wavelet.kth_smallest(l,r,rank)==a[ids[rank]]); } else CHECK(!wavelet.kth_smallest(l,r,0));
        }
    }
    for(int n:{0,1,63,64,65,127,128,129,1000}) {
        cp::DynamicBitset bits(n); cp::SuccinctBitVector succinct(n); std::vector<bool> a(n);
        for(int i=0;i<n;++i) { a[i]=pick(2); bits.set(i,a[i]); succinct.set(i,a[i]); } succinct.build();
        for(int shift:{0,1,63,64,65,129,1001}) {
            auto l=bits<<shift,r=bits>>shift;
            for(int i=0;i<n;++i) { CHECK(l.test(i)==(i>=shift && a[i-shift])); CHECK(r.test(i)==(i+shift<n && a[i+shift])); }
        }
        int ones=0; for(int i=0;i<=n;++i) { CHECK(succinct.rank(i)==ones); if(i<n) ones+=a[i]; } CHECK(bits.count()==unsigned(ones));
        for(bool b:{false,true}) { int k=0; for(int i=0;i<n;++i) if(a[i]==b) CHECK(succinct.select(k++,b)==i); CHECK(succinct.select(k,b)==-1); }
        for(int i=0;i<=n;++i) { int j=i; while(j<n && !a[j]) ++j; CHECK(bits.find_next(i)==unsigned(j)); }
        CHECK((bits&~bits).count()==0); CHECK((bits|~bits).count()==unsigned(n)); CHECK((bits^bits).count()==0);
    }
    cp::PersistentBinaryTrie trie; std::vector<int> roots{0}; std::vector<std::vector<std::uint64_t>> versions(1);
    for(int t=0;t<4000;++t) {
        int v=pick(int(roots.size())); auto a=versions[v]; int root;
        if(!a.empty() && pick(3)==0) { int p=pick(int(a.size())); root=*trie.erase(roots[v],a[p]); a.erase(a.begin()+p); }
        else { auto x=rng(); root=trie.insert(roots[v],x); a.push_back(x); }
        roots.push_back(root); versions.push_back(a); v=pick(int(roots.size())); auto mask=rng(); a=versions[v]; for(auto& x:a) x^=mask; std::sort(a.begin(),a.end()); CHECK(trie.size(roots[v])==a.size());
        for(int k=0;k<int(a.size());++k) CHECK(trie.kth(roots[v],k,mask)==a[k]);
        CHECK(!trie.kth(roots[v],a.size(),mask));
    }
    cp::PrioritySum<long long> priority(4); std::multiset<long long> all;
    for(int t=0;t<20000;++t) {
        long long x=pick(100)-50; if(pick(2)) { priority.insert(x); all.insert(x); } else { auto it=all.find(x); CHECK(priority.erase(x)==(it!=all.end())); if(it!=all.end()) all.erase(it); }
        int k=pick(30); priority.set_k(k); long long sum=0; int j=0; for(auto value:all) { if(j++==k) break; sum+=value; } CHECK(priority.sum()==sum && priority.size()==all.size());
    }
    cp::SparseDynamicSegtree<long long,sum_op,sum_e> sparse(1LL<<50); sparse.set((1LL<<50)-1,7); CHECK(sparse.prod(0,1LL<<50)==7 && sparse.node_count()<60);
    cp::DualSegtree2D<long long,sum_op,sum_e> dual(8,9); long long grid[8][9]={};
    for(int t=0;t<3000;++t) { int a=pick(9),b=pick(9),c=pick(10),d=pick(10); if(a>b) std::swap(a,b); if(c>d) std::swap(c,d); long long x=pick(7)-3; dual.apply(a,b,c,d,x); for(int i=a;i<b;++i) for(int j=c;j<d;++j) grid[i][j]+=x; int i=pick(8),j=pick(9); CHECK(dual.get(i,j)==grid[i][j]); }
    std::vector<CountedKey> generic{CountedKey(2),CountedKey(1),CountedKey(2)};
    cp::WaveletTree<CountedKey> generic_tree(generic); CHECK(generic_tree.kth_smallest(0,3,1)->value==2);
    cp::WeightedWaveletMatrix<CountedKey> generic_matrix(generic,{5,7,9}); CHECK(generic_matrix.range_sum(0,3,CountedKey(1),CountedKey(2))==7);
    cp::SegtreeFractionalCascading<CountedKey> generic_fc(generic); CHECK(generic_fc.range_freq(0,3,CountedKey(2),CountedKey(3))==2);
    const int large_n=200000; std::vector<CountedKey> large; large.reserve(large_n);
    for(int i=0;i<large_n;++i) large.emplace_back((i*53)%large_n);
    CountedKey::comparisons=0; cp::LinearRMQ<CountedKey> large_rmq(large); CHECK(CountedKey::comparisons<20LL*large_n);
    CountedKey::comparisons=0; for(int i=0;i<1000;++i) { int l=pick(large_n),r=l+1+pick(large_n-l); int p=large_rmq.argmin(l,r); CHECK(l<=p && p<r); } CHECK(CountedKey::comparisons<=6000);
    std::vector<long long> ones(131072,1); operations_count=0; cp::SqrtTree<long long,counted_sum,sum_e> large_sqrt(ones); CHECK(operations_count<20LL*int(ones.size()));
    operations_count=0; for(int i=0;i<1000;++i) { int l=pick(int(ones.size())+1),r=pick(int(ones.size())+1); if(l>r) std::swap(l,r); CHECK(large_sqrt.prod(l,r)==r-l); } CHECK(operations_count<=2000);
    std::cout<<"range/static: branching histories, 108000 static queries, packed boundary bits, 4000 trie versions and 20000 priority operations passed\n";
}
