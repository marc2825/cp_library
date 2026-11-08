#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "string_algorithm/all.hpp"
std::string read_bytes(){std::string h,s;std::cin>>h;if(h=="-")return s;for(std::size_t i=0;i<h.size();i+=2)s.push_back(static_cast<char>(std::stoul(h.substr(i,2),nullptr,16)));return s;}
void print(const std::vector<int>&a){std::cout<<a.size();for(auto x:a)std::cout<<' '<<x;std::cout<<' ';}
template<class S>void arrays(const S&s){auto sa=cp::suffix_array(s);auto r=cp::manacher(s);print(cp::z_algorithm(s));print(cp::prefix_function(s));print(sa);print(cp::lcp_array(s,sa));print(r.odd);print(r.even);std::cout<<'\n';}
int main(){char op;while(std::cin>>op){try{if(op=='A')arrays(read_bytes());else if(op=='V'){std::size_t n;std::cin>>n;std::vector<long long>a(n);for(auto&x:a)std::cin>>x;arrays(a);}else if(op=='K'){auto s=read_bytes(),p=read_bytes();print(cp::kmp_search(s,p));std::cout<<'\n';}else if(op=='H'){std::uint32_t base;std::cin>>base;auto s=read_bytes(),t=read_bytes();int l,r,bl,br;std::cin>>l>>r>>bl>>br;cp::RollingHash a(s,base),b(t,base);auto h=a.get(l,r),j=b.get(bl,br);auto common=a.lcp(b,l,r,bl,br);auto same=a.equal(b,l,r,bl,br);auto joined=a.concat(a.get(0,l),a.get(l,r),r-l);std::cout<<h[0]<<' '<<h[1]<<' '<<j[0]<<' '<<j[1]<<' '<<common<<' '<<same<<' '<<joined[0]<<' '<<joined[1]<<'\n';}else if(op=='T'){int n;std::cin>>n;cp::Trie trie;while(n--){char action;std::cin>>action;auto s=read_bytes();std::size_t result=0;if(action=='I'){trie.insert(s);result=trie.count(s);}else if(action=='E')result=trie.erase(s);else if(action=='C')result=trie.count(s);else if(action=='P')result=trie.count_prefix(s);else if(action=='R')trie.clear();std::cout<<result<<' '<<trie.size()<<' '<<trie.node_count()<<'\n';}}}catch(const std::out_of_range&){std::cout<<"RANGE\n";}catch(const std::invalid_argument&){std::cout<<"INVALID\n";}}}
