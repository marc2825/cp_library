---
title: 辺リストの入力補助
documentation_of: //graph_basic/input.hpp
---

# read_graph

`cp::read_graph<T=long long>(istream&,n,m,GraphInputOptions={})` は m 本の辺を読み、`Graph<T>` を返します。共有の本体は [graph/input.hpp](../../graph/input.hpp) です。

GraphInputOptions の既定値は `kind=Undirected`、`weighted=false`、`index_base=1`。重み付きなら u v cost、それ以外なら u v を読みます。入力頂点から index_base を引いて 0 始まりへ直します。m≥0、有効な変換後頂点を前提とします。

```cpp
#include <cassert>
#include <sstream>
#include "graph_basic/input.hpp"
int main() {
    std::istringstream input("1 2 7\n2 3 -1\n");
    cp::GraphInputOptions options;
    options.kind=cp::GraphKind::Directed;
    options.weighted=true;
    auto g=cp::read_graph<long long>(input,3,2,options);
    assert(g.directed() && g.edges()[0].from==0 && g.edges()[1].cost==-1);
}
```

読み込み不足は `std::runtime_error`、頂点等の使用条件は assert で確認します。時間・メモリ O(V+E)。重みの条件は使用するアルゴリズムに合わせてください。

[Graph の契約](graph.md) / [利用するアルゴリズム・問題例・図解](README.md)。入力補助は既存グラフ・木の検証、使用例、単独 include で確認します。[参考資料](references.md)。
