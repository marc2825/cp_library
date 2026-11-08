#define PROBLEM "https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_C"
#include <iostream>
#include <string>
#include <vector>
#include "string_algorithm/rolling_hash_2d.hpp"
int main(){int h,w,n,m;std::cin>>h>>w;std::vector<std::string>a(h);for(auto&row:a)std::cin>>row;std::cin>>n>>m;std::vector<std::string>b(n);for(auto&row:b)std::cin>>row;const cp::RollingHash2D hash(a),pattern(b);for(int i=0;i+n<=h;++i)for(int j=0;j+m<=w;++j)if(hash.equal(pattern,i,i+n,j,j+m,0,n,0,m)){bool exact=true;for(int k=0;k<n;++k)if(a[i+k].compare(j,m,b[k])!=0)exact=false;if(exact)std::cout<<i<<' '<<j<<'\n';}}
