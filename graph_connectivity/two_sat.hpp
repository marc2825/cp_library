#pragma once
#include <cassert>
#include <optional>
#include <vector>
#include "scc.hpp"

namespace cp {
// 節 (x_i=f) OR (x_j=g)。solveは毎回再計算、未充足ならnullopt。
class TwoSAT {
    Graph<int> graph_;
    static int vertices(int n) { assert(0<=n && n<=100000000); return 2*n; }
    void check(int v) const { assert(0<=v && v<size()); (void)v; }
public:
    explicit TwoSAT(int n):graph_(vertices(n),GraphKind::Directed) {}
    int size() const { return graph_.size()/2; }
    void add_clause(int i,bool f,int j,bool g) {
        check(i); check(j); int a=2*i+int(f),b=2*j+int(g);
        graph_.add_edge(a^1,b); graph_.add_edge(b^1,a);
    }
    void force(int i,bool value) { add_clause(i,value,i,value); }
    void imply(int i,bool f,int j,bool g) { add_clause(i,!f,j,g); }
    std::optional<std::vector<bool>> solve() const {
        auto s=strongly_connected_components(graph_); std::vector<bool> answer(size());
        for(int v=0;v<size();++v) {
            if(s.component[2*v]==s.component[2*v+1]) return std::nullopt;
            answer[v]=s.component[2*v]<s.component[2*v+1];
        }
        return answer;
    }
};
} // namespace cp
