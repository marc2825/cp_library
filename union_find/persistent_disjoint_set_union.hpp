#pragma once
#include <cassert>
#include <utility>
#include "../ordered_structure/persistent_segment_tree.hpp"

namespace cp {
namespace detail {
// -1を単位元とする右優先モノイド。一点取得で単位元と結合しても値を保つ。
inline int persistent_dsu_op(int a,int b) { return b==-1?a:b; }
inline int persistent_dsu_e() { return -1; }
}
// 同一オブジェクト由来のrootハンドルを使う。経路圧縮なし、union by size。
class PersistentDisjointSetUnion {
    PersistentSegmentTree<int,detail::persistent_dsu_op,detail::persistent_dsu_e> data_;
public:
    explicit PersistentDisjointSetUnion(int n):data_(n) {}
    int empty_root() const { return data_.empty_root(); }
    int vertex_count() const { return data_.size(); }
    int leader(int root,int v) const {
        assert(0<=v && v<vertex_count());
        for (;;) { int p=data_.get(root,v); if (p<0) return v; v=p; }
    }
    int size(int root,int v) const { return -data_.get(root,leader(root,v)); }
    bool same(int root,int u,int v) const { return leader(root,u)==leader(root,v); }
    int merge(int root,int u,int v) {
        u=leader(root,u); v=leader(root,v);
        if (u==v) return root;
        int a=data_.get(root,u), b=data_.get(root,v);
        if (a>b) { std::swap(u,v); std::swap(a,b); }
        return data_.set(data_.set(root,u,a+b),v,u);
    }
};
} // namespace cp
