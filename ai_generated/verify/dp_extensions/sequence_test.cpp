#include "common.hpp"
#include "dp_optimization/bit_parallel_lcs.hpp"
#include "dp_optimization/myers_edit_distance.hpp"
#include "dp_optimization/lcs.hpp"
#include "dp_optimization/edit_distance.hpp"
#include "dp_optimization/knapsack_reconstruction.hpp"
#include "dp_optimization/optimal_solution_count.hpp"
#include "dp_optimization/small_weight_knapsack.hpp"
#include "dp_optimization/zero_one_knapsack.hpp"
#include "dp_optimization/bounded_knapsack.hpp"
#include "basic_utility/static_modint.hpp"
using namespace cp;
void strings(const std::string& a,const std::string& b) {
    CHECK(bit_parallel_lcs_length(a,b)==std::size_t(lcs_length(a,b)));
    CHECK(myers_edit_distance(a,b)==std::size_t(edit_distance(a,b)));
}
int main() {
    std::mt19937 rng(6201);
    std::vector<std::string> binary;
    for(int n=0;n<=6;++n)for(int mask=0;mask<(1<<n);++mask){std::string s;for(int j=0;j<n;++j)s+=char('0'+(mask>>j&1));binary.push_back(s);}
    for(const auto& a:binary)for(const auto& b:binary)strings(a,b);
    for(int trial=0;trial<1500;++trial){std::string a(rng()%330,' '),b(rng()%330,' ');for(auto& c:a)c=char(rng()%(trial%2?256:3));for(auto& c:b)c=char(rng()%(trial%2?256:3));strings(a,b);}
    for(int n:{0,1,63,64,65,127,128,129,255,256,257})for(int m:{0,1,63,64,65,127,128,129}) {
        strings(std::string(n,'a'),std::string(m,'a'));strings(std::string(n,'a'),std::string(m,'b'));
    }
    CHECK(bit_parallel_lcs_length(std::string(100000,'a'),std::string(300,'a'))==300);
    CHECK(myers_edit_distance(std::string(100000,'a'),std::string(300,'a'))==99700);
    for(int trial=0;trial<800;++trial) {
        int n=int(rng()%11);std::vector<int>a(n);for(auto&x:a)x=int(rng()%5);
        for(bool strict:{false,true}) {
            int best=0,ways=0;
            for(int mask=0;mask<(1<<n);++mask){bool good=true;std::optional<int> prev;for(int i=0;i<n;++i)if(mask>>i&1){if(prev && (strict?*prev>=a[i]:*prev>a[i]))good=false;prev=a[i];}if(good){int len=__builtin_popcount(unsigned(mask));if(len>best){best=len;ways=0;}if(len==best)++ways;}}
            auto result=lis_count(a,strict);CHECK(result.length==best && result.count==ways);
        }
        std::vector<int>b(rng()%8);for(auto&x:b)x=int(rng()%3);
        int best=0,ways=0;
        for(int x=0;x<(1<<n);++x)for(int y=0;y<(1<<int(b.size()));++y)if(__builtin_popcount(unsigned(x))==__builtin_popcount(unsigned(y))) {
            std::vector<int>u,v;for(int i=0;i<n;++i)if(x>>i&1)u.push_back(a[i]);for(int j=0;j<int(b.size());++j)if(y>>j&1)v.push_back(b[j]);
            if(u==v){int len=int(u.size());if(len>best){best=len;ways=0;}if(len==best)++ways;}
        }
        auto result=lcs_alignment_count(a,b);CHECK(result.length==best && result.count==ways);
        CHECK(lcs_alignment_count<StaticModInt<12>>(a,b).count.val()==unsigned(ways%12));
    }
    for(int trial=0;trial<1000;++trial) {
        int n=int(rng()%11),capacity=int(rng()%31);std::vector<KnapsackItem>items;
        for(int i=0;i<n;++i)items.push_back({int(rng()%9),int(rng()%17)-8});
        std::vector<std::optional<long long>>best(capacity+1);std::vector<int>ways(capacity+1);
        for(int mask=0;mask<(1<<n);++mask){int w=0;long long value=0;for(int i=0;i<n;++i)if(mask>>i&1){w+=items[i].weight;value+=items[i].value;}if(w>capacity)continue;if(!best[w]||*best[w]<value){best[w]=value;ways[w]=0;}if(*best[w]==value)++ways[w];}
        auto result=zero_one_knapsack_reconstruct(capacity,items);auto counted=zero_one_knapsack_count(capacity,items);
        CHECK(result.values.dp==best);CHECK(counted.values.dp==best);CHECK(small_weight_knapsack(capacity,items).dp==best);
        BitsetSubsetSumReconstruction subset(capacity);for(auto item:items)subset.add(item.weight);
        for(int w=0;w<=capacity;++w){CHECK(counted.counts[w]==ways[w]);auto counts=result.restore(w);CHECK(bool(counts)==bool(best[w]));if(counts){int weight=0;long long value=0;for(int i=0;i<n;++i){CHECK((*counts)[i]>=0&&(*counts)[i]<=1);weight+=int((*counts)[i])*items[i].weight;value+=(*counts)[i]*items[i].value;}CHECK(weight==w&&value==*best[w]);}auto ids=subset.restore(w);CHECK(bool(ids)==bool(best[w]));if(ids){CHECK(std::set<int>(ids->begin(),ids->end()).size()==ids->size());int weight=0;for(auto id:*ids)weight+=items[id].weight;CHECK(weight==w);}}
        long long value=0;int count=0;for(int w=0;w<=capacity;++w)if(best[w])value=std::max(value,*best[w]);for(int w=0;w<=capacity;++w)if(best[w]&&*best[w]==value)count+=ways[w];CHECK(counted.best().first==value&&counted.best().second==count);
    }
    for(int trial=0;trial<500;++trial) {
        int capacity=int(rng()%35);std::vector<BoundedKnapsackItem>items;
        for(int i=0;i<5;++i)items.push_back({int(rng()%8),int(rng()%13)-6,int(rng()%4)});
        std::vector<std::optional<long long>>best(capacity+1);
        auto dfs=[&](auto&&self,int i,int w,long long value)->void{if(i==int(items.size())){if(!best[w]||value>*best[w])best[w]=value;return;}for(int k=0;k<=items[i].count&&w+k*items[i].weight<=capacity;++k)self(self,i+1,w+k*items[i].weight,value+k*items[i].value);};dfs(dfs,0,0,0);
        auto result=bounded_knapsack_reconstruct(capacity,items);CHECK(result.values.dp==best);
        for(int w=0;w<=capacity;++w)if(best[w]){auto c=*result.restore(w);int weight=0;long long value=0;for(int i=0;i<int(items.size());++i){CHECK(c[i]>=0&&c[i]<=items[i].count);weight+=int(c[i])*items[i].weight;value+=c[i]*items[i].value;}CHECK(weight==w&&value==*best[w]);}
        std::vector<KnapsackItem>unbounded{{1,2},{3,7},{4,-1}};auto u=unbounded_knapsack_reconstruct(capacity,unbounded);for(int w=0;w<=capacity;++w){long long value=LLONG_MIN;for(int b=0;3*b<=w;++b)for(int c=0;3*b+4*c<=w;++c)value=std::max(value,2LL*(w-3*b-4*c)+7*b-c);CHECK(u.values.dp[w]==value);auto counts=*u.restore(w);CHECK(counts[0]+3*counts[1]+4*counts[2]==w&&2*counts[0]+7*counts[1]-counts[2]==value);}
    }
    for(int t=0;t<300;++t){std::vector<KnapsackItem>a;for(int j=0;j<100;++j)a.push_back({int(rng()%25),int(rng()%100)-50});CHECK(small_weight_knapsack(500,a).dp==zero_one_knapsack(500,a).dp);}
    std::vector<KnapsackItem>many(100000,{2,1});CHECK(small_weight_knapsack(20000,many).best_value()==10000);
    BitsetSubsetSumReconstruction big(200000);for(int i=1;i<=1000;++i)big.add(i);auto restored=big.restore(200000);CHECK(restored);int sum=0;for(auto id:*restored)sum+=id+1;CHECK(sum==200000);
    CHECK(lcs_alignment_count(std::string(70,'a'),std::string(35,'a')).count>BigInt(ULLONG_MAX));
    throws<std::overflow_error>([]{lcs_alignment_count<unsigned long long>(std::string(70,'a'),std::string(35,'a'));});
    CHECK(zero_one_knapsack_count(0,std::vector<KnapsackItem>(100,{0,0})).counts[0]==BigInt("1267650600228229401496703205376"));
    CHECK(bounded_knapsack_reconstruct(0,{{0,1,LLONG_MAX}}).restore(0)->at(0)==LLONG_MAX);
    throws<std::invalid_argument>([]{unbounded_knapsack_reconstruct(1,{{0,1}});});
    throws<std::invalid_argument>([]{small_weight_knapsack(-1,{});});
    throws<std::overflow_error>([]{zero_one_knapsack_reconstruct(0,{{0,LLONG_MAX},{0,1}});});
    std::cout<<"sequences: exhaustive binary pairs, 1500 multiword cases, 800 count enumerations, 1500 reconstruction oracles, 300 grouped knapsacks, large inputs passed\n";
}
