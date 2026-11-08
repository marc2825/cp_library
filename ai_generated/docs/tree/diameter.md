---
title: 木の直径・パス復元
documentation_of: //tree/diameter.hpp
---

# 木の直径・パス復元

非負重みの連結な木について、距離が最大の2頂点とその間のパスを O(N) で求めます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/tree/index.html#diameter) / [実装](../../tree/diameter.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "tree/diameter.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 3); g.add_edge(1, 3, 5);
    auto diameter = cp::tree_diameter(g);
    std::cout << diameter.length << ' ' << diameter.vertices.size() << '\n';
}
```

[動く使用例](../../examples/tree/diameter.cpp)。リポジトリのルートを include パスに指定します。

## API・使用条件

| API / 結果 | 意味 |
|---|---|
| `tree_diameter(g)` | `TreeDiameterResult<T>` を返す |
| `length` | 直径の重み和 T |
| `endpoints` | `{s,t}`。復元するパスの向き |
| `vertices` | s→t の頂点列 |
| `edges` | 同じ向きの元辺ID列。edges[i] は vertices[i] と vertices[i+1] の辺 |

非空では無向・連結・閉路なしが条件。非連結／閉路は invalid_argument。辺重みは非負が前提（assert で検査）。負辺の最大重みパスには使いません。同点の端点の辞書順は保証しません。

空の Graph は長さ0・端点 `{-1,-1}`・空列。一頂点なら長さ0・端点 `{0,0}`・頂点列 `{0}`。0重みも許容し、直径0では同一頂点が両端になる場合があります。

## 原理・計算量・注意点

時間 O(N)、空間 O(N)。入力を変更しません。

任意の頂点から最遠の s を選び、s から再走査して最遠の t を選びます。木ではパスが一つなので、重み付きでも DFS の距離和で十分です。2回目の親・親辺を t から逆走査してパスを復元します。

[RootedTree の距離算術](rooted_tree.md)を使います。重み・距離の全中間値が型の範囲内かつ最大値未満であることが必要で、超過は overflow_error。

## 使用する問題・検証

[Tree Diameter](https://judge.yosupo.jp/problem/tree_diameter)。

小さな木の全頂点対距離の最大値と比較。復元頂点・元辺ID・端点・距離を検査。0重み・一頂点・空・非連結・浮動小数点・20万頂点。

共通の [検証入口](../../verify/run_tree.py) は C++17・警告エラー・debug/release・単独 include・複数翻訳単位・使用例・独立オラクル・ジャッジ入出力を確認します。`--sanitize` で ASan/UBSan。**オンライン AC は未確認**です。
