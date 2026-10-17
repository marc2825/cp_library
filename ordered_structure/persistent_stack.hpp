#pragma once
#include <cassert>
#include <optional>
#include <vector>

namespace cp {
// root=0は空。全ハンドルは生成元オブジェクト専用。値はプールが所有。
template<class T> class PersistentStack {
    struct Node { std::optional<T> value; int previous=0,size=0; };
    std::vector<Node> nodes_{Node{}};
    void check(int root) const { assert(0<=root && root<int(nodes_.size())); (void)root; }
public:
    int empty_root() const { return 0; }
    int size(int root) const { check(root); return nodes_[root].size; }
    bool empty(int root) const { return size(root)==0; }
    T top(int root) const { check(root); assert(root); return *nodes_[root].value; }
    int push(int root,const T& x) {
        check(root); int n=nodes_[root].size+1; nodes_.push_back({x,root,n}); return int(nodes_.size())-1;
    }
    int pop(int root) const { check(root); assert(root); return nodes_[root].previous; }
};
} // namespace cp
