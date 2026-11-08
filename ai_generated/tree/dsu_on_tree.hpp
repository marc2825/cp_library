#pragma once
#include <vector>
#include "rooted_tree.hpp"

namespace cp {
// add(v)/remove(v)で可逆な集約を管理し、answer(v)時はvの部分木だけが入る。
// 開始・終了時は空。非再帰、各頂点O(log n)回追加/削除。森に対応。
template<class T,class Add,class Remove,class Answer>
void dsu_on_tree(const RootedTree<T>& tree,Add add,Remove remove,Answer answer) {
    int n=tree.size(); std::vector<std::vector<int>> children(n); std::vector<int> heavy(n,-1);
    for(int v:tree.preorder) if(tree.parent[v]!=-1) {
        int p=tree.parent[v]; children[p].push_back(v);
        if(heavy[p]==-1 || tree.subtree_size[v]>tree.subtree_size[heavy[p]]) heavy[p]=v;
    }
    struct Event { int v,stage; bool keep; };
    std::vector<Event> stack;
    for(auto it=tree.roots.rbegin();it!=tree.roots.rend();++it) stack.push_back({*it,0,false});
    while(!stack.empty()) {
        auto event=stack.back(); stack.pop_back(); int v=event.v;
        if(event.stage==0) {
            if(!event.keep) stack.push_back({v,3,false});
            stack.push_back({v,2,true}); stack.push_back({v,1,true});
            if(heavy[v]!=-1) stack.push_back({heavy[v],0,true});
            for(auto it=children[v].rbegin();it!=children[v].rend();++it) if(*it!=heavy[v]) stack.push_back({*it,0,false});
        } else if(event.stage==1) {
            for(int c:children[v]) if(c!=heavy[v]) for(int i=tree.tin[c];i<tree.tout[c];++i) add(tree.preorder[i]);
            add(v);
        } else if(event.stage==2) answer(v);
        else for(int i=tree.tin[v];i<tree.tout[v];++i) remove(tree.preorder[i]);
    }
}
} // namespace cp
