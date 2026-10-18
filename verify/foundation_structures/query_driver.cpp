#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include "basic_utility/fast_io.hpp"
#include "basic_utility/seeded_hash.hpp"
#include "basic_utility/static_modint.hpp"
#include "graph/csr_graph.hpp"
#include "range_query/polynomial_add_sum.hpp"
int main(){
    cp::FastScanner in;cp::FastPrinter out;std::string kind;
    while(in.read(kind)){
        if(kind=="G"){
            int n=0,m=0,directed=0;in.read(n,m,directed);std::vector<cp::Edge<long long>> edges(m);
            for(auto& e:edges)in.read(e.from,e.to,e.cost);
            cp::CSRGraph<long long> graph(n,edges,directed?cp::GraphKind::Directed:cp::GraphKind::Undirected);
            out.write(graph.arc_count());for(auto x:graph.offsets()){out.write(' ');out.write(x);}
            for(const auto& a:graph.arcs()){out.write(' ');out.write(a.from,a.to,a.cost,a.idx,a.rev);}out.writeln();
        }else if(kind=="P"){
            using Mint=cp::ModInt998244353;using A=cp::PolynomialAddSumAction<Mint,2>;
            int n=0,q=0;in.read(n,q);std::vector<Mint>a(n);long long x=0;
            for(auto&v:a){in.read(x);v=Mint(x);}cp::ActedMonoidLazySegtree<A> tree(A::values(a));
            for(int i=0;i<q;i++){
                int op=0,l=0,r=0;in.read(op,l);
                if(op==0){long long c0=0,c1=0,c2=0;in.read(r,c0,c1,c2);tree.apply(l,r,{Mint(c0),Mint(c1),Mint(c2)});}
                else if(op==1){in.read(r);out.writeln(tree.prod(l,r).sum.val());}
                else {in.read(x);tree.set(l,A::leaf(Mint(x),Mint(l)));}
            }
        }else{
            std::uint64_t seed=0;in.read(seed);cp::SeededHash hash(seed);
            if(kind=="I"){std::int64_t x=0;in.read(x);out.writeln(hash.hash64(x));}
            else if(kind=="W"){__uint128_t x=0;in.read(x);out.writeln(hash.hash64(x));}
            else if(kind=="T"){std::int64_t x=0,y=0;in.read(x,y);out.writeln(hash.hash64(std::make_pair(x,y)));}
            else if(kind=="V"){int n=0;in.read(n);std::vector<std::int64_t>a(n);for(auto&x:a)in.read(x);out.writeln(hash.hash64(a));}
            else if(kind=="S"){int n=0;in.read(n);std::string s;for(int i=0;i<n;i++){unsigned x=0;in.read(x);s.push_back(static_cast<char>(x));}out.writeln(hash.hash64(s));}
            else throw std::runtime_error("unknown query");
        }
    }
    out.flush();
}
