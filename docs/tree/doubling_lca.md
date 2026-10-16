---
title: LCA・木上の距離と移動
documentation_of: //tree/doubling_lca.hpp
---

# LCA・木上の距離と移動

根付き木で最も近い共通祖先を求めます。重み付き距離、祖先への移動、u→v のパス上の k 番目も同じ前計算で処理します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/tree/index.html#lca) / [実装](../../tree/doubling_lca.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "tree/doubling_lca.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 3); g.add_edge(1, 3, 5);
    cp::DoublingLCA<> lca(g);
    std::cout << lca.lca(2, 3) << ' ' << lca.distance(2, 3) << ' ' << lca.jump(2, 3, 1) << '\n';
}
```

[動く使用例](../../examples/tree/doubling_lca.cpp)。リポジトリのルートを include パスに指定します。

## API・使用条件

| API | 戻り値・条件 |
|---|---|
| `DoublingLCA<T>(g,root=0)` | 森を根付きにして祖先表を構築 |
| `DoublingLCA<T>(rooted)` | RootedTree を値で受け取る。move で配列を引き継げる |
| `lca(u,v)` | LCA。別成分は `-1` |
| `kth_ancestor(v,k)` | k 個上。k は非負の long long。範囲外は `-1` |
| `distance(u,v)` | 重み和 T。同一成分が条件 |
| `edge_distance(u,v)` | 辺数 int。同一成分が条件 |
| `jump(u,v,k)` | パスの0始まりのk番目。範囲外・別成分は `-1` |
| `is_on_path(s,t,v)` | v が s→t 上にあるか。端点を含む。別成分は false |
| `vertex_path(u,v)` | u→v の頂点列。別成分は空 |
| `tree` / `up[b][v]` | RootedTree / 2^b 個上の祖先。不存在は `-1` |

自分自身の LCA は自分、0個上も自分です。空の森は構築できますが、頂点を指定する問い合わせはできません。[RootedTree の入力・算術条件](rooted_tree.md)を引き継ぎます。元の Graph の変更後は再構築します。

## 原理・計算量・注意点

構築・空間 O(N log(N+1))。lca・distance・edge_distance・jump・kth_ancestor・is_on_path は O(log(N+1))。vertex_path は O(log(N+1)+L)、L は復元パスの頂点数。

深さを揃えてから、2^b 個上が異なる最大の b で両方を上げます。祖先が同じ段は飛ばし、最後の親を答えにします。距離は LCA から両端への根距離の差を加算します。差・和の全中間値が型に収まることが必要で、超過は overflow_error。

負辺と0重みに対応します。is_on_path は距離の重み和ではなく辺数で判定するため、重み0でも正しく動きます。

## 使用する問題・検証

[Lowest Common Ancestor](https://judge.yosupo.jp/problem/lca)、[Jump on Tree](https://judge.yosupo.jp/problem/jump_on_tree)。

全頂点対を BFS のパス列・親の素朴走査と比較。全祖先・経路の全位置・別成分・最大 long long・負辺・0重み・符号なし型・距離超過・20万頂点。

共通の [検証入口](../../verify/run_tree.py) は C++17・警告エラー・debug/release・単独 include・複数翻訳単位・使用例・独立オラクル・ジャッジ入出力を確認します。`--sanitize` で ASan/UBSan。**オンライン AC は未確認**です。
