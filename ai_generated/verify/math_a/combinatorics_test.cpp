#include <algorithm>
#include <climits>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <set>
#include <stdexcept>
#include <vector>
#include "combinatorics/all.hpp"
void check(bool ok){if(!ok)throw std::runtime_error("combinatorics mismatch");}
template<class E,class F>void throws(F f){bool caught=false;try{f();}catch(const E&){caught=true;}check(caught);}
long long partitions(int n,int minimum){if(!n)return 1;long long answer=0;for(int p=minimum;p<=n;++p)answer+=partitions(n-p,p);return answer;}
int main(){
    for(long long mod:{1LL,2LL,8LL,97LL,998244353LL,LLONG_MAX}){
        auto first=cp::stirling_first(9,mod),second=cp::stirling_second(9,mod),signed_first=cp::stirling_first(9,mod,true);
        auto bell=cp::bell_numbers(9,mod),derange=cp::derangements(9,mod),parts=cp::partition_numbers(22,mod);
        for(int n=0;n<=9;++n){
            std::vector<int> permutation(n);std::iota(permutation.begin(),permutation.end(),0);std::vector<long long> cycles(n+1);long long derangements=0;
            do{std::vector<bool> used(n);int count=0;bool deranged=true;for(int i=0;i<n;++i){deranged&=permutation[i]!=i;if(!used[i]){++count;int v=i;do{used[v]=true;v=permutation[v];}while(v!=i);}}++cycles[count];derangements+=deranged;}while(std::next_permutation(permutation.begin(),permutation.end()));
            check(derange[n]==derangements%mod);
            std::vector<long long> blocks(n+1);auto enumerate=[&](auto&& self,int pos,int max_block)->void{if(pos==n){++blocks[max_block+1];return;}for(int b=0;b<=max_block+1;++b)self(self,pos+1,std::max(max_block,b));};enumerate(enumerate,0,-1);
            long long sets=0;for(int k=0;k<=n;++k){check(first[n][k]==cycles[k]%mod&&second[n][k]==blocks[k]%mod);long long sign=(n-k)%2&&cycles[k]%mod?mod-cycles[k]%mod:cycles[k]%mod;check(signed_first[n][k]==sign);sets+=blocks[k];}check(bell[n]==sets%mod);
        }
        for(int n=0;n<=22;++n)check(parts[n]==partitions(n,1)%mod);
    }
    for(int n=1;n<=7;++n)for(int dihedral=0;dihedral<=1;++dihedral){
        std::vector<std::vector<int>> group;for(int flip=0;flip<=dihedral;++flip)for(int shift=0;shift<n;++shift){std::vector<int> p(n);for(int i=0;i<n;++i)p[i]=(shift+(flip?n-i:i))%n;group.push_back(p);}
        cp::CycleIndex index(group);
        for(int colors=0;colors<=3;++colors){
            std::set<std::vector<int>> representatives;std::vector<std::uint64_t> fixed(group.size());long long weighted=0;int total=1;for(int i=0;i<n;++i)total*=colors;
            for(int encoding=0;encoding<total;++encoding){int v=encoding;std::vector<int> a(n);for(int& x:a){x=v%colors;v/=colors;}auto best=a;
                for(std::size_t id=0;id<group.size();++id){std::vector<int> b(n);for(int i=0;i<n;++i)b[group[id][i]]=a[i];fixed[id]+=a==b;best=std::min(best,b);}
                if(representatives.insert(best).second){long long weight=1;for(int x:a)weight*=x+1;weighted+=weight;}
            }
            check(index.colorings(colors)==static_cast<long long>(representatives.size()));check(cp::burnside_orbits_exact(fixed)==representatives.size());
            check(cp::burnside_orbits(int(group.size()),[&](int i){return fixed[i];})==static_cast<long long>(representatives.size()));
            std::vector<long long> weights(n+1);for(int k=1;k<=n;++k)for(int c=1;c<=colors;++c)weights[k]+=cp::mod_pow(c,k,998244353);
            check(index.evaluate(weights)==weighted%998244353);
        }
    }
    cp::CycleIndex empty(std::vector<std::vector<int>>{{}});check(empty.colorings(0)==1&&empty.evaluate({0})==1);
    throws<std::invalid_argument>([]{cp::stirling_first(-1);});throws<std::invalid_argument>([]{cp::stirling_second(0,0);});throws<std::invalid_argument>([]{cp::bell_numbers(-1);});throws<std::invalid_argument>([]{cp::partition_numbers(-1);});throws<std::invalid_argument>([]{cp::derangements(-1);});
    throws<std::invalid_argument>([]{cp::CycleIndex x({});});throws<std::invalid_argument>([]{cp::CycleIndex x({{0,0}});});throws<std::invalid_argument>([]{cp::CycleIndex x({{2,0}});});
    throws<std::domain_error>([]{cp::CycleIndex({{0,1},{1,0}}).colorings(2,8);});check(cp::CycleIndex({{0,1},{1,0}}).colorings(2,1)==0);
    throws<std::overflow_error>([]{cp::burnside_orbits_exact({UINT64_MAX,1});});throws<std::domain_error>([]{cp::burnside_orbits_exact({0,1});});
    std::cout<<"permutation cycles, set/integer partitions, derangements, coloring orbits and weighted Polya passed\n";
}
