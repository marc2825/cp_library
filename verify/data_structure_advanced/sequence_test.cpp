#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "../../ordered_structure/rbst_segment_tree.hpp"
#include "../../ordered_structure/persistent_lazy_rbst.hpp"
#include "../../ordered_structure/finger_tree.hpp"
#include "../../ordered_structure/dynamic_bit_vector.hpp"
#include "../../range_query/dynamic_wavelet_matrix.hpp"
#define CHECK(x) do {if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
std::mt19937_64 rng(355632);
std::string concat(std::string a,std::string b){return a+b;}
std::string empty_string(){return {};}
struct Sum{long long sum;int length;};
Sum sum_op(Sum a,Sum b){return {(a.sum+b.sum)%10007,a.length+b.length};}
Sum sum_e(){return {0,0};}
struct Affine{long long a,b;};
Sum mapping(Affine f,Sum x){return {(f.a*x.sum+f.b*x.length)%10007,x.length};}
Affine composition(Affine f,Affine g){return {f.a*g.a%10007,(f.a*g.b+f.b)%10007};}
Affine identity(){return {1,0};}
void persistent(){
    using Tree=cp::PersistentLazyRBST<Sum,sum_op,sum_e,Affine,mapping,composition,identity>;
    std::vector<Tree> versions(1);std::vector<std::vector<long long>> expected(1);
    for(int step=0;step<7000;++step){int source=rng()%versions.size();auto tree=versions[source];auto a=expected[source];int n=int(a.size());int l=rng()%(n+1),r=rng()%(n+1);if(l>r)std::swap(l,r);
        switch(rng()%7){case 0:{long long x=rng()%10007;tree.insert(l,{x,1});a.insert(a.begin()+l,x);break;}
        case 1:if(n){int p=rng()%n;tree.erase(p);a.erase(a.begin()+p);}break;
        case 2:if(n){int p=rng()%n;long long x=rng()%10007;tree.set(p,{x,1});a[p]=x;}break;
        case 3:{Affine f{static_cast<long long>(rng()%10007),static_cast<long long>(rng()%10007)};tree.apply(l,r,f);for(int i=l;i<r;++i)a[i]=(f.a*a[i]+f.b)%10007;break;}
        case 4:tree.reverse(l,r);std::reverse(a.begin()+l,a.begin()+r);break;
        case 5:{int p=rng()%(n-(r-l)+1);tree.move(l,r,p);std::vector<long long> part(a.begin()+l,a.begin()+r);a.erase(a.begin()+l,a.begin()+r);a.insert(a.begin()+p,part.begin(),part.end());break;}
        case 6:{auto [x,y]=tree.split(l);tree=y.concat(x);std::rotate(a.begin(),a.begin()+l,a.end());break;}}
        CHECK(tree.size()==int(a.size()));auto actual=tree.to_vector();long long sum=0;for(std::size_t i=0;i<a.size();++i){CHECK(actual[i].sum==a[i] && actual[i].length==1);sum=(sum+a[i])%10007;}CHECK(tree.all_prod().sum==sum);
        for(int q=0;q<3;++q){int i=rng()%(a.size()+1),j=rng()%(a.size()+1);if(i>j)std::swap(i,j);CHECK(tree.prod(i,j).sum==std::accumulate(a.begin()+i,a.begin()+j,0LL)%10007);}
        versions.push_back(tree);expected.push_back(a);CHECK(versions[source].size()==int(expected[source].size()));
    }
    for(std::size_t v=0;v<versions.size();++v){auto actual=versions[v].to_vector();CHECK(actual.size()==expected[v].size());for(std::size_t i=0;i<actual.size();++i)CHECK(actual[i].sum==expected[v][i]);}
    cp::RBSTSegmentTree<std::string,concat,empty_string> words({"a","b","c","d"});auto old=words;words.reverse(0,4);CHECK(words.all_prod()=="dcba");CHECK(old.prod(1,4)=="bcd");auto halves=words.split(2);CHECK(halves.second.concat(halves.first).all_prod()=="badc");
}
void fingers(){
    using Tree=cp::FingerTree<std::string,concat,empty_string>;Tree tree;std::vector<std::string>a;std::vector<std::pair<Tree,std::vector<std::string>>> saved;
    for(int step=0;step<10000;++step){if(step%200==0)saved.push_back({tree,a});std::string x(1,char('a'+rng()%26));
        switch(rng()%7){case 0:tree.push_front(x);a.insert(a.begin(),x);break;case 1:tree.push_back(x);a.push_back(x);break;
        case 2:if(!a.empty()){CHECK(tree.pop_front()==a.front());a.erase(a.begin());}break;case 3:if(!a.empty()){CHECK(tree.pop_back()==a.back());a.pop_back();}break;
        case 4:{int p=rng()%(a.size()+1);auto [left,right]=tree.split(p);CHECK(left.to_vector()==std::vector<std::string>(a.begin(),a.begin()+p));CHECK(right.to_vector()==std::vector<std::string>(a.begin()+p,a.end()));tree=right.concat(left);std::rotate(a.begin(),a.begin()+p,a.end());break;}
        case 5:if(a.size()<500){Tree extra({x,x,x});tree=tree.concat(extra);a.insert(a.end(),3,x);}break;
        case 6:if(!saved.empty()){auto& entry=saved[rng()%saved.size()];CHECK(entry.first.to_vector()==entry.second);}break;}
        CHECK(tree.size()==int(a.size()));CHECK(tree.to_vector()==a);CHECK(tree.all_prod()==std::accumulate(a.begin(),a.end(),std::string()));if(!a.empty()){CHECK(tree.front()==a.front());CHECK(tree.back()==a.back());int p=rng()%a.size();CHECK(tree.get(p)==a[p]);}
        int l=rng()%(a.size()+1),r=rng()%(a.size()+1);if(l>r)std::swap(l,r);CHECK(tree.prod(l,r)==std::accumulate(a.begin()+l,a.begin()+r,std::string()));
    }
    auto doubled=tree.concat(tree);auto expected=a;expected.insert(expected.end(),a.begin(),a.end());CHECK(doubled.to_vector()==expected);
    cp::FingerTree<Sum,sum_op,sum_e> large;for(int i=0;i<30000;++i)large.push_back({i%10007,1});CHECK(large.size()==30000);for(int p:{0,1,17,19999,30000}){auto halves=large.split(p);CHECK(halves.first.size()==p);CHECK(halves.second.size()==30000-p);CHECK(halves.first.concat(halves.second).all_prod().sum==large.all_prod().sum);}
}
void bits(){
    cp::DynamicBitVector bits;std::vector<bool>a;
    for(int step=0;step<20000;++step){int p=rng()%(a.size()+1);bool value=rng()%2;
        if(a.empty() || (a.size()<200 && rng()%3==0)){bits.insert(p,value);a.insert(a.begin()+p,value);}
        else if(rng()%2){p=rng()%a.size();CHECK(bits.erase(p)==a[p]);a.erase(a.begin()+p);}
        else {p=rng()%a.size();bits.set(p,value);a[p]=value;}
        CHECK(bits.size()==int(a.size()));for(bool b:{false,true}){int count=0;for(int i=0;i<int(a.size());++i){CHECK(bits.rank(b,i)==count);CHECK(bits.access(i)==a[i]);if(a[i]==b){CHECK(bits.select(b,count)==i);++count;}}CHECK(bits.rank(b,int(a.size()))==count);CHECK(bits.select(b,count)==-1);CHECK(bits.select(b,-1)==-1);}
    }
}
void wavelets(){
    for(int width:{1,7,32,64}){cp::DynamicWaveletMatrix matrix(width);std::vector<std::uint64_t>a;std::uint64_t mask=width==64?~std::uint64_t(0):(std::uint64_t(1)<<width)-1;
        for(int step=0;step<5000;++step){auto x=rng()&mask;int p=rng()%(a.size()+1);
            if(a.empty() || (a.size()<100 && rng()%3==0)){matrix.insert(p,x);a.insert(a.begin()+p,x);}else if(rng()%2){p=rng()%a.size();CHECK(matrix.erase(p)==a[p]);a.erase(a.begin()+p);}else {p=rng()%a.size();matrix.set(p,x);a[p]=x;}
            CHECK(matrix.size()==int(a.size()));for(int i=0;i<int(a.size());++i)CHECK(matrix.access(i)==a[i]);int l=rng()%(a.size()+1),r=rng()%(a.size()+1);if(l>r)std::swap(l,r);std::vector<std::uint64_t> sorted(a.begin()+l,a.begin()+r);std::sort(sorted.begin(),sorted.end());for(int k=0;k<r-l;++k)CHECK(matrix.kth_smallest(l,r,k)==sorted[k]);CHECK(!matrix.kth_smallest(l,r,r-l));CHECK(!matrix.kth_smallest(l,r,-1));
            if(!a.empty())x=a[rng()%a.size()];
            CHECK(matrix.count(l,r,x)==std::count(a.begin()+l,a.begin()+r,x));CHECK(matrix.count_less(l,r,x)==std::count_if(a.begin()+l,a.begin()+r,[&](auto v){return v<x;}));int occurrence=0;for(int i=0;i<int(a.size());++i)if(a[i]==x)CHECK(matrix.select(x,occurrence++)==i);CHECK(matrix.select(x,occurrence)==-1);
            auto low=std::lower_bound(sorted.begin(),sorted.end(),x);auto predecessor=matrix.predecessor(l,r,x),successor=matrix.successor(l,r,x);if(low==sorted.begin())CHECK(!predecessor);else CHECK(predecessor==*std::prev(low));if(low==sorted.end())CHECK(!successor);else CHECK(successor==*low);
        }
        if(width<64){CHECK(matrix.count(0,matrix.size(),mask+1)==0);CHECK(matrix.count_less(0,matrix.size(),mask+1)==matrix.size());bool threw=false;try{matrix.insert(0,mask+1);}catch(const std::out_of_range&){threw=true;}CHECK(threw);}
    }
}
void large_wavelet(){
    std::vector<std::uint64_t> values(5000);for(int i=0;i<5000;++i)values[i]=(i*1117)%65536;
    cp::DynamicWaveletMatrix matrix(values,16);
    for(int i=0;i<5000;i+=17)CHECK(matrix.access(i)==values[i]);
    for(int i=0;i<2500;++i){CHECK(matrix.erase(0)==values[i]);matrix.insert(matrix.size(),values[i]);}
    std::rotate(values.begin(),values.begin()+2500,values.end());
    for(int i=0;i<5000;i+=17)CHECK(matrix.access(i)==values[i]);
    std::sort(values.begin(),values.end());for(int k=0;k<5000;k+=17)CHECK(matrix.kth_smallest(0,5000,k)==values[k]);
}
int main(){large_wavelet();persistent();fingers();bits();wavelets();std::cout<<"advanced sequences OK\n";}
