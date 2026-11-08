#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "string_algorithm/all.hpp"
namespace {
void require(bool b,const char* message){if(!b)throw std::runtime_error(message);}
template<class E,class F>void rejects(F f){bool caught=false;try{f();}catch(const E&){caught=true;}require(caught,"missing exception");}
std::vector<int> naive_z(const std::string&s){std::vector<int>a(s.size());for(std::size_t i=0;i<s.size();++i)while(a[i]<static_cast<int>(s.size()-i)&&s[a[i]]==s[i+a[i]])++a[i];return a;}
std::vector<int> naive_prefix(const std::string&s){std::vector<int>a(s.size());for(std::size_t i=0;i<s.size();++i)for(std::size_t k=1;k<=i;++k)if(s.compare(0,k,s,i+1-k,k)==0)a[i]=static_cast<int>(k);return a;}
std::vector<int> naive_search(const std::string&s,const std::string&p){std::vector<int>a;for(std::size_t i=0;i<=s.size();++i)if(p.size()<=s.size()-i&&s.compare(i,p.size(),p)==0)a.push_back(static_cast<int>(i));return a;}
bool palindrome(const std::string&s,int l,int r){while(l<r)if(s[l++]!=s[--r])return false;return true;}
cp::RollingHash::Hash hash_direct(std::string_view s,std::uint32_t base){cp::RollingHash::Hash h{};for(std::size_t j=0;j<2;++j){std::uint64_t power=1;for(std::size_t i=s.size();i-- >0;){h[j]=static_cast<std::uint32_t>((h[j]+power*(static_cast<unsigned char>(s[i])+1))%cp::RollingHash::moduli[j]);power=power*base%cp::RollingHash::moduli[j];}}return h;}
void check(const std::string&s){
    require(cp::z_algorithm(s)==naive_z(s),"Z by direct prefix comparisons");require(cp::prefix_function(s)==naive_prefix(s),"all border lengths");
    auto sa=cp::suffix_array(s);std::vector<int>sorted(s.size());std::iota(sorted.begin(),sorted.end(),0);std::sort(sorted.begin(),sorted.end(),[&](int i,int j){return std::lexicographical_compare(s.begin()+i,s.end(),s.begin()+j,s.end(),[](char a,char b){return static_cast<unsigned char>(a)<static_cast<unsigned char>(b);});});require(sa==sorted,"suffix lexicographic order by unsigned bytes");
    auto lcp=cp::lcp_array(s,sa);for(std::size_t i=0;i<lcp.size();++i){int common=0;while(common<static_cast<int>(s.size())-sa[i]&&common<static_cast<int>(s.size())-sa[i+1]&&s[sa[i]+common]==s[sa[i+1]+common])++common;require(lcp[i]==common,"adjacent direct LCP");}
    auto r=cp::manacher(s);cp::RollingHash hash(s);for(int l=0;l<=static_cast<int>(s.size());++l)for(int rr=l;rr<=static_cast<int>(s.size());++rr){require(r.is_palindrome(l,rr)==palindrome(s,l,rr),"all interval palindromes");require(hash.get(l,rr)==hash_direct(std::string_view(s).substr(l,rr-l),hash.base()),"substring hash by sum of powers");}
    for(int i=0;i<static_cast<int>(s.size());++i){int odd=1,even=0;while(odd<=i&&odd<static_cast<int>(s.size())-i&&s[i-odd]==s[i+odd])++odd;while(even<i&&even<static_cast<int>(s.size())-i&&s[i-even-1]==s[i+even])++even;require(r.odd[i]==odd&&r.even[i]==even,"both center radii");}
    for(int a=0;a<=static_cast<int>(s.size());++a)for(int b=0;b<=static_cast<int>(s.size());++b){int n=0;while(n<static_cast<int>(s.size())-std::max(a,b)&&s[a+n]==s[b+n])++n;require(hash.lcp(hash,a,b)==n,"hash suffix LCP by direct comparison");}
}
struct Symbol {int value;static std::size_t comparisons;friend bool operator==(Symbol a,Symbol b){++comparisons;return a.value==b.value;}friend bool operator<(Symbol a,Symbol b){return a.value<b.value;}};std::size_t Symbol::comparisons=0;
void generic_and_large(){
    const std::vector<long long>a{std::numeric_limits<long long>::max(),std::numeric_limits<long long>::lowest(),0,std::numeric_limits<long long>::lowest()};auto sa=cp::suffix_array(a);require(sa==std::vector<int>({3,1,2,0}),"generic signed extremes ordering");require(cp::lcp_array(a,sa)==std::vector<int>({1,0,0}),"generic LCP");require(cp::kmp_search(a,std::vector<long long>{std::numeric_limits<long long>::lowest()})==std::vector<int>({1,3}),"generic KMP");
    std::vector<Symbol>symbols(200000,Symbol{1});Symbol::comparisons=0;cp::z_algorithm(symbols);require(Symbol::comparisons<=4*symbols.size(),"Z linear comparisons");Symbol::comparisons=0;cp::prefix_function(symbols);require(Symbol::comparisons<=4*symbols.size(),"prefix linear comparisons");Symbol::comparisons=0;auto radii=cp::manacher(symbols);require(Symbol::comparisons<=8*symbols.size(),"Manacher linear comparisons");Symbol::comparisons=0;auto matches=cp::kmp_search(symbols,std::vector<Symbol>(1000,Symbol{1}));require(matches.size()==199001&&Symbol::comparisons<=6*(symbols.size()+1000),"overlap-heavy KMP linear work");
    const int n=200000;std::string repeated(n,'a');auto ordered=cp::suffix_array(repeated);auto lcp=cp::lcp_array(repeated,ordered);auto z=cp::z_algorithm(repeated);auto prefix=cp::prefix_function(repeated);for(int i=0;i<n;++i){require(ordered[i]==n-i-1&&z[i]==n-i&&prefix[i]==i,"large repeated all outputs");if(i<n-1)require(lcp[i]==i+1,"large repeated LCP");require(radii.odd[i]==std::min(i+1,n-i)&&radii.even[i]==std::min(i,n-i),"large repeated radii");}
    std::string alternating(n,'a');for(int i=1;i<n;i+=2)alternating[i]='b';auto r=cp::manacher(alternating);for(int i=0;i<n;++i)require(r.odd[i]==std::min(i+1,n-i)&&r.even[i]==0,"large alternating radii");
    cp::Trie trie;trie.insert(repeated);require(trie.node_count()==static_cast<std::size_t>(n+1)&&trie.count(repeated)==1&&trie.count_prefix(repeated.substr(0,n-1))==1,"deep trie no recursion");require(trie.erase(repeated)&&trie.empty(),"deep trie erase");trie.clear();require(trie.node_count()==1,"trie clear frees history");
    cp::RollingHash hash(repeated,1000000006);require(hash.lcp(hash,0,n-1)==1&&hash.get(0,n)==hash_direct(repeated,hash.base()),"long hash extreme base");
    std::cout<<"generic signed extremes; 200000-byte repeated/alternating/deep inputs and linear comparison counts passed\n";
}
void hash_boundaries(std::mt19937_64&rng){
    for(auto base:{257U,911382323U,1000000006U})for(int trial=0;trial<100;++trial){std::string a(rng()%30,'\0'),b(rng()%30,'\0');for(auto&x:a)x=static_cast<char>(rng()%256);for(auto&x:b)x=static_cast<char>(rng()%256);cp::RollingHash ha(a,base),hb(b,base);require(ha.hash_string(b)==hash_direct(b,base),"direct hash of foreign bytes");for(int q=0;q<20;++q){int l=rng()%(a.size()+1),r=rng()%(a.size()+1),bl=rng()%(b.size()+1),br=rng()%(b.size()+1);if(l>r)std::swap(l,r);if(bl>br)std::swap(bl,br);int common=0;while(common<std::min(r-l,br-bl)&&a[l+common]==b[bl+common])++common;require(ha.lcp(hb,l,r,bl,br)==common,"bounded cross-object LCP");require(ha.equal(hb,l,r,bl,br)==(a.substr(l,r-l)==b.substr(bl,br-bl)),"cross-object equal on corpus");require(ha.concat(ha.get(0,l),ha.get(l,r),r-l)==ha.get(0,r),"hash concatenation");}}
    cp::RollingHash a("abc"),b("abc",257);rejects<std::invalid_argument>([&]{a.equal(b,0,0,0,0);});rejects<std::invalid_argument>([&]{a.lcp(b,0,0);});
    for(auto base:{0U,256U,1000000007U,std::numeric_limits<std::uint32_t>::max()})rejects<std::invalid_argument>([&]{cp::RollingHash bad("a",base);});
    rejects<std::out_of_range>([&]{a.get(-1,2);});rejects<std::out_of_range>([&]{a.get(2,1);});rejects<std::out_of_range>([&]{a.get(0,4);});rejects<std::out_of_range>([&]{a.concat({0,0},{0,0},4);});rejects<std::invalid_argument>([&]{a.concat({1000000007,0},{0,0},0);});
    auto saved=a;auto moved=std::move(a);require(moved.get(0,3)==saved.get(0,3)&&a.size()==0&&a.get(0,0)==cp::RollingHash::Hash({0,0})&&a.concat({0,0},{0,0},0)==cp::RollingHash::Hash({0,0}),"hash move source usable as empty");
    // 2法の積のbase257表現。全係数差がbyteの範囲内なので二重衝突を実際に作れる。
    const unsigned char digits[]{13,129,147,110,252,31,115,34};
    const std::string collision(reinterpret_cast<const char*>(digits),sizeof(digits)),zeros(sizeof(digits),'\0');
    cp::RollingHash hc(collision,257),hz(zeros,257);
    require(collision!=zeros&&hc.equal(hz,0,8,0,8),"known double collision does not imply exact equality");
    require(cp::RollingHash(std::string(1,'\0')).get(0,1)!=cp::RollingHash("").get(0,0),"NUL byte is encoded as one");
    std::cout<<"300 cross-object byte hashes / 6000 bounded queries; bases, concatenation, ranges and move passed\n";
}
void trie_histories(std::mt19937_64&rng){
    std::size_t operations=0;
    for(int trial=0;trial<120;++trial){cp::Trie trie;std::map<std::string,int>bag;std::set<std::string>prefixes{""};for(int step=0;step<400;++step){std::string word(rng()%7,'\0');for(auto&c:word)c=static_cast<char>(rng()%5==0?rng()%256:'a'+rng()%3);const int op=rng()%5;if(op<2){trie.insert(word);++bag[word];for(std::size_t k=1;k<=word.size();++k)prefixes.insert(word.substr(0,k));}else if(op==2){const bool existed=bag[word]>0;require(trie.erase(word)==existed,"trie erase result");if(existed)--bag[word];}else if(op==3&&step%13==0){trie.clear();bag.clear();prefixes={""};}
            std::size_t size=0;for(const auto&entry:bag)size+=entry.second;require(trie.size()==size&&trie.count("")==static_cast<std::size_t>(bag[""])&&trie.node_count()==prefixes.size(),"trie size/root/historical nodes");
            std::size_t expected=0;for(const auto&entry:bag)if(entry.first.compare(0,word.size(),word)==0)expected+=entry.second;require(trie.count(word)==static_cast<std::size_t>(bag[word])&&trie.count_prefix(word)==expected&&trie.contains(word)==(bag[word]>0),"trie exact/prefix by dictionary scan");++operations;
        }auto copy=trie;copy.insert("extra");require(copy.size()==trie.size()+1,"trie independent copy");auto moved=std::move(copy);require(copy.empty()&&copy.node_count()==1&&moved.size()==trie.size()+1,"trie move resets root");copy=std::move(moved);copy=std::move(copy);require(copy.size()==trie.size()+1,"trie move/self assignment");}
    std::cout<<"120 trie histories / "<<operations<<" operations against dictionary and historical prefix sets passed\n";
}
void rejection(){
    rejects<std::invalid_argument>([]{cp::lcp_array("abc",{0,1});});rejects<std::invalid_argument>([]{cp::lcp_array("abc",{0,1,1});});rejects<std::invalid_argument>([]{cp::lcp_array("abc",{-1,0,1});});rejects<std::invalid_argument>([]{cp::lcp_array("abc",{0,1,3});});
    auto radii=cp::manacher("a");rejects<std::out_of_range>([&]{radii.is_palindrome(-1,0);});rejects<std::out_of_range>([&]{radii.is_palindrome(1,0);});rejects<std::out_of_range>([&]{radii.is_palindrome(0,2);});
    rejects<std::length_error>([]{cp::string_algorithm_detail::checked_size(static_cast<std::size_t>(std::numeric_limits<int>::max())+1);});
}
}
int main(){std::mt19937_64 rng(321021);std::size_t count=0;for(int n=0,limit=1;n<=8;++n,limit*=3)for(int code=0;code<limit;++code){std::string s(n,'a');int value=code;for(auto&x:s){x=static_cast<char>('a'+value%3);value/=3;}check(s);++count;}for(int trial=0;trial<180;++trial){std::string s(rng()%70,'\0');for(auto&x:s)x=static_cast<char>(rng()%256);check(s);}std::cout<<count<<" ternary strings + 180 byte strings: Z/prefix/suffix/LCP/all palindromes/hash definitions passed\n";std::size_t pairs=0;for(int n=0;n<=6;++n)for(int a=0;a<(1<<n);++a){std::string s(n,'a');for(int i=0;i<n;++i)if(a>>i&1)s[i]='b';for(int m=0;m<=4;++m)for(int b=0;b<(1<<m);++b){std::string p(m,'a');for(int i=0;i<m;++i)if(b>>i&1)p[i]='b';require(cp::kmp_search(s,p)==naive_search(s,p),"all small text/pattern pairs");++pairs;}}std::cout<<pairs<<" binary KMP pairs including empty patterns and overlaps passed\n";generic_and_large();hash_boundaries(rng);trie_histories(rng);rejection();}
