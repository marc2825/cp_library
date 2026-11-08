#include "common.hpp"
#include "graph_connectivity/spqr_tree.hpp"
#include "graph_connectivity/tree_decomposition.hpp"
#include "graph_connectivity/cograph_canonical_form.hpp"
#include "graph_combinatorics/graph_isomorphism.hpp"
#include "graph_combinatorics/series_parallel_decomposition.hpp"
using namespace cp;
bool connected(const Graph<int>& g,int a=-1,int b=-1) {
    int start=-1;for(int v=0;v<g.size();++v)if(v!=a && v!=b)start=v;
    if(start<0)return true;
    std::vector<bool> seen(g.size());std::vector<int> q{start};seen[start]=true;
    for(std::size_t i=0;i<q.size();++i)for(auto e:g[q[i]])if(e.to!=a && e.to!=b && !seen[e.to]){seen[e.to]=true;q.push_back(e.to);}
    return int(q.size())==g.size()-(a>=0)-(b>=0 && b!=a);
}
void spqr(const Graph<int>& g) {
    bool valid=g.size()>=2 && connected(g); for(int v=0;v<g.size();++v)valid=valid && connected(g,v);
    auto r=spqr_tree(g); CHECK(bool(r)==valid); if(!r)return;
    CHECK(r->tree.edge_count()==r->tree.size()-1 && connected(r->tree));
    std::vector<int> occur(g.edge_count());
    for(int i=0;i<int(r->nodes.size());++i) {
        auto node=r->nodes[i];std::set<int> vertices;
        for(auto e:node.edges){vertices.insert(e.from);vertices.insert(e.to);}
        std::vector<int> original(vertices.begin(),vertices.end()); Graph<int> skeleton(int(original.size()));
        for(int j=0;j<int(node.edges.size());++j) {
            auto e=node.edges[j];int u=int(std::lower_bound(original.begin(),original.end(),e.from)-original.begin()),v=int(std::lower_bound(original.begin(),original.end(),e.to)-original.begin());skeleton.add_edge(u,v);
            if(e.original_edge>=0){++occur[e.original_edge];auto x=g.edges()[e.original_edge];CHECK(x.from==e.from && x.to==e.to);CHECK(r->original_location[e.original_edge]==std::make_pair(i,j));}
            else {CHECK(e.twin_node>=0);auto twin=r->nodes[e.twin_node].edges[e.twin_edge];CHECK(twin.twin_node==i && twin.twin_edge==j);CHECK(twin.virtual_id==e.virtual_id);CHECK(std::minmax(e.from,e.to)==std::minmax(twin.from,twin.to));CHECK(node.kind!=r->nodes[e.twin_node].kind || node.kind==SPQRTree::Kind::R);}
        }
        if(node.kind==SPQRTree::Kind::S){CHECK(skeleton.size()>=3);for(int v=0;v<skeleton.size();++v)CHECK(skeleton[v].size()==2);}
        if(node.kind==SPQRTree::Kind::P){CHECK(skeleton.size()==2);CHECK(skeleton.edge_count()>=2);}
        if(node.kind==SPQRTree::Kind::Q)CHECK(g.edge_count()==1);
        if(node.kind==SPQRTree::Kind::R){CHECK(skeleton.size()>=4);std::set<std::pair<int,int>> unique;for(auto e:skeleton.edges())CHECK(unique.insert(std::minmax(e.from,e.to)).second);for(int a=0;a<skeleton.size();++a)for(int b=a+1;b<skeleton.size();++b)CHECK(connected(skeleton,a,b));}
    }
    for(int c:occur)CHECK(c==1);
    // Every virtual edge is an actual separation of the original real edges.
    for(auto link:r->tree.edges()) {
        std::vector<bool> side(r->tree.size());std::vector<int> queue{link.from};side[link.from]=true;
        for(std::size_t i=0;i<queue.size();++i)for(auto e:r->tree[queue[i]])if(e.idx!=link.idx && !side[e.to]){side[e.to]=true;queue.push_back(e.to);}
        std::set<int> left,right;for(int i=0;i<int(r->nodes.size());++i)for(auto e:r->nodes[i].edges)if(e.original_edge>=0){auto& s=side[i]?left:right;s.insert(e.from);s.insert(e.to);}
        std::vector<int> intersection;std::set_intersection(left.begin(),left.end(),right.begin(),right.end(),std::back_inserter(intersection));CHECK(intersection.size()==2);
    }
}
int order_width(const Graph<int>& g,const std::vector<int>& order) {
    int n=g.size(),width=-1;std::vector<std::set<int>> a(n);for(auto e:g.edges())if(e.from!=e.to){a[e.from].insert(e.to);a[e.to].insert(e.from);}
    for(int v:order){width=std::max(width,int(a[v].size()));auto row=a[v];for(int u:row){a[u].erase(v);for(int w:row)if(u!=w)a[u].insert(w);}a[v].clear();}return width;
}
void decomposition(const Graph<int>& g,const TreeDecomposition& r) {
    int n=g.size();CHECK(int(r.bags.size())==n);CHECK(order_width(g,r.elimination_order)==r.width);
    for(auto e:g.edges()){bool found=false;for(auto bag:r.bags)if(std::find(bag.begin(),bag.end(),e.from)!=bag.end() && std::find(bag.begin(),bag.end(),e.to)!=bag.end())found=true;CHECK(found);}
    for(int v=0;v<n;++v){std::vector<bool> contains(n);int count=0,start=-1;for(int i=0;i<n;++i)if(std::find(r.bags[i].begin(),r.bags[i].end(),v)!=r.bags[i].end()){contains[i]=true;++count;start=i;}CHECK(count>0);std::vector<int> queue{start};contains[start]=false;for(std::size_t i=0;i<queue.size();++i)for(auto e:r.forest[queue[i]])if(contains[e.to]){contains[e.to]=false;queue.push_back(e.to);}CHECK(int(queue.size())==count);}
}
bool brute_isomorphic(const Graph<int>& a,const Graph<int>& b) {
    if(a.size()!=b.size() || a.edge_count()!=b.edge_count())return false;
    int n=a.size();std::vector<std::vector<int>> A(n,std::vector<int>(n)),B=A;
    for(auto e:a.edges()){++A[e.from][e.to];if(!a.directed() && e.from!=e.to)++A[e.to][e.from];}
    for(auto e:b.edges()){++B[e.from][e.to];if(!b.directed() && e.from!=e.to)++B[e.to][e.from];}
    std::vector<int> p(n);std::iota(p.begin(),p.end(),0);do{bool ok=true;for(int u=0;u<n;++u)for(int v=0;v<n;++v)if(A[u][v]!=B[p[u]][p[v]])ok=false;if(ok)return true;}while(std::next_permutation(p.begin(),p.end()));return false;
}
bool sp_oracle(const Graph<int>& g,int s,int t) {
    int m=g.edge_count(),size=1<<m;std::vector<int> vertices(size);std::vector<std::set<std::pair<int,int>>> dp(size);
    for(auto e:g.edges()){if(e.from==e.to)return false;vertices[1<<e.idx]=(1<<e.from)|(1<<e.to);dp[1<<e.idx].insert({e.from,e.to});}
    for(int mask=1;mask<size;++mask){int bit=mask&-mask;vertices[mask]=vertices[bit]|vertices[mask^bit];for(int a=(mask-1)&mask;a;a=(a-1)&mask){int b=mask^a;if(!b)continue;int common=vertices[a]&vertices[b];for(auto x:dp[a])for(auto y:dp[b]){if(x.second==y.first && common==(1<<x.second))dp[mask].insert({x.first,y.second});if(x==y && common==((1<<x.first)|(1<<x.second)))dp[mask].insert(x);}}}
    return vertices.back()==(1<<g.size())-1 && dp.back().count({s,t});
}
int main() {
    std::mt19937 rng(638);
    for(int n=0;n<=5;++n) {
        int m=n*(n-1)/2;
        for(int mask=0;mask<(1<<m);++mask) {
            Graph<int> g(n);int id=0;for(int u=0;u<n;++u)for(int v=u+1;v<n;++v,++id)if(mask>>id&1)g.add_edge(u,v);
            if(n>=2)spqr(g);
            auto r=minimum_tree_decomposition(g);decomposition(g,r);CHECK(r.optimal);auto greedy=greedy_tree_decomposition(g);decomposition(g,greedy);CHECK(!greedy.optimal && greedy.width>=r.width);
            std::vector<int> order(n);std::iota(order.begin(),order.end(),0);int best=n;do{best=std::min(best,order_width(g,order));}while(std::next_permutation(order.begin(),order.end()));CHECK(r.width==best);
            bool p4=false;for(int a=0;a<n;++a)for(int b=a+1;b<n;++b)for(int c=b+1;c<n;++c)for(int d=c+1;d<n;++d){std::set<int> chosen{a,b,c,d};std::vector<int> degree;Graph<int> induced(4);std::vector<int> v(chosen.begin(),chosen.end());for(int i=0;i<4;++i){int deg=0;for(auto e:g[v[i]])if(chosen.count(e.to)){++deg;int j=int(std::find(v.begin(),v.end(),e.to)-v.begin());if(i<j)induced.add_edge(i,j);}degree.push_back(deg);}std::sort(degree.begin(),degree.end());if(degree==std::vector<int>({1,1,2,2}) && connected(induced))p4=true;}
            auto canon=cograph_canonical_form(g);CHECK(bool(canon)==!p4);std::shuffle(order.begin(),order.end(),rng);Graph<int> h(n);for(auto e:g.edges())h.add_edge(order[e.from],order[e.to]);auto other=cograph_canonical_form(h);if(canon)CHECK(canon->code==other->code);
        }
    }
    for(int trial=0;trial<250;++trial){int n=2+int(rng()%9);Graph<int> g(n);for(int v=0;v<n;++v)g.add_edge(v,(v+1)%n);for(int i=0;i<15;++i){int u=int(rng()%n),v=int(rng()%n);if(u!=v)g.add_edge(u,v);}spqr(g);}
    for(int trial=0;trial<350;++trial){int n=int(rng()%8);Graph<int> a(n,trial%2?GraphKind::Directed:GraphKind::Undirected),b(n,a.kind());if(n)for(int i=0;i<10;++i){a.add_edge(int(rng()%n),int(rng()%n));b.add_edge(int(rng()%n),int(rng()%n));}if(trial%2==0){b=Graph<int>(n,a.kind());std::vector<int> p(n);std::iota(p.begin(),p.end(),0);std::shuffle(p.begin(),p.end(),rng);for(auto e:a.edges())b.add_edge(p[e.from],p[e.to]);}CHECK(bool(graph_isomorphism(a,b))==brute_isomorphic(a,b));}
    for(int trial=0;trial<250;++trial){int n=2+int(rng()%4);Graph<int> g(n,GraphKind::Directed);for(int i=0;i<1+int(rng()%8);++i)g.add_edge(int(rng()%n),int(rng()%n));auto result=series_parallel_decomposition(g,0,n-1);CHECK(bool(result)==sp_oracle(g,0,n-1));if(result){std::vector<int> count(g.edge_count()),stack{result->root};while(!stack.empty()){auto x=result->nodes[stack.back()];stack.pop_back();if(x.kind==SeriesParallelDecomposition::Kind::edge)++count[x.original_edge];else{auto a=result->nodes[x.left],b=result->nodes[x.right];CHECK(x.from==a.from && x.to==b.to);if(x.kind==SeriesParallelDecomposition::Kind::series)CHECK(a.to==b.from);else CHECK(a.from==b.from && a.to==b.to);stack.push_back(x.left);stack.push_back(x.right);}}for(int c:count)CHECK(c==1);}}
    Graph<int> clique(16);for(int u=0;u<16;++u)for(int v=u+1;v<16;++v)clique.add_edge(u,v);
    CHECK(minimum_tree_decomposition(clique).width==15);CHECK(cograph_canonical_form(clique));
    auto rigid=spqr_tree(clique);CHECK(rigid && rigid->nodes.size()==1 && rigid->nodes[0].kind==SPQRTree::Kind::R);
    Graph<int> chain(20000,GraphKind::Directed);for(int v=1;v<20000;++v)chain.add_edge(v-1,v);
    auto expression=series_parallel_decomposition(chain,0,19999);CHECK(expression && expression->nodes.size()==39997);
    Graph<int> colors(2);CHECK(!graph_isomorphism(colors,colors,{1,2},{1,1}));CHECK(cograph_canonical_form(colors,{1,2})->code==cograph_canonical_form(colors,{2,1})->code);
    throws<std::length_error>([]{minimum_tree_decomposition(Graph<int>(23));});
    Graph<int> six_cycle(6),triangles(6);
    for(int i=0;i<6;++i) { six_cycle.add_edge(i,(i+1)%6);triangles.add_edge(i,i/3*3+(i+1)%3); }
    CHECK(!graph_isomorphism(six_cycle,triangles));
    throws<std::invalid_argument>([]{Graph<int> g(1);g.add_edge(0,0);spqr_tree(g);});
    std::cout<<"structure: all simple graphs through n=5 (SPQR, exact/greedy treewidth, cograph), 250 multigraph SPQR, 350 exact isomorphisms, 250 SP expression oracles passed\n";
}
