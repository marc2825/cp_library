// Local judge-style wrapper with its own protocol (0 insert, 1 erase, 2 reverse, 3 sum);
// no Library Checker problem uses it. Run by verify/run_advanced.py on a sample input.
#include <iostream>
#include "ordered_structure/implicit_treap.hpp"
long long op(long long a,long long b){return a+b;}long long e(){return 0;}
int main(){int n,q;std::cin>>n>>q;std::vector<long long>a(n);for(auto &x:a)std::cin>>x;cp::ImplicitTreap<long long,op,e>s(a);while(q--){int t;std::cin>>t;if(t==0){int p;long long x;std::cin>>p>>x;s.insert(p,x);}else if(t==1){int p;std::cin>>p;s.erase(p);}else{int l,r;std::cin>>l>>r;if(t==2)s.reverse(l,r);else std::cout<<s.prod(l,r)<<'\n';}}}
