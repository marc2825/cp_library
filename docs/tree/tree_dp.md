---
title: Tree DP の基本テンプレート
documentation_of: //tree/tree_dp.hpp
---

# Tree DP の基本テンプレート

根を一つ選び、子の結果を帰りがけ順に集める汎用テンプレートです。集約・頂点の追加・辺を越える変換をコールバックで定義します。森では各成分の根について同じ処理をします。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/tree/index.html#dp) / [実装](../../tree/tree_dp.hpp) / [参考資料](references.md)

## usage

```cpp
#include <array>
#include <iostream>
#include "tree/tree_dp.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    // {根を白にする場合の数, 根を黒にする場合の数}。隣り合う黒を禁止する。
    using S = std::array<long long, 2>;
    auto dp = cp::tree_dp(g, S{1, 1},
        [](S a, S b) { return S{a[0] * b[0], a[1] * b[1]}; },
        [](S a, int) { return a; },
        [](S a, const cp::Edge<long long>&) { return S{a[0] + a[1], a[0]}; });
    std::cout << dp[0][0] + dp[0][1] << '\n';
}
```

[動く使用例](../../examples/tree/tree_dp.cpp)。リポジトリのルートを include パスに指定します。

## API・使用条件

```text
tree_dp(g, identity, merge, add_vertex, lift, root=0) -> vector<R>
```

| コールバック | 契約 |
|---|---|
| `merge(S a,S b) -> S` | 寄与を左から右へ集約。identity は単位元 |
| `add_vertex(S aggregate,int v) -> R` | v 自身を加えて頂点の状態へ変換 |
| `lift(const R& value,const Edge<T>& edge) -> S` | 送信元側の状態を受信先から見た寄与へ変換 |

R は add_vertex の戻り値から推論し、S と異なる型にもできます。R はデフォルト構築・コピー・代入可能、S はコピー・代入可能が必要です。関数は値で受け取ります。move-only の関数オブジェクトは rvalue または std::ref 等で渡します。

lift へ渡す edge は **情報の送信元→受信先** です。子 c の結果を親 v が受け取るなら `edge.from=c`, `edge.to=v`。cost と idx は元の無向辺のものを保ちます。

`dp[v] = add_vertex(merge(… lift(dp[c], c→v) …),v)`。子の走査は g[v] の順で、親辺だけ除きます。葉では `add_vertex(identity,v)`。戻り値の dp[v] は選んだ根に対する部分木の状態で、森全体の答えが必要なら roots の結果を問題に応じてまとめます。

[RootedTree の入力・距離算術条件](rooted_tree.md)を引き継ぎます。辺の重みを DP で無視しても根距離の前計算を行うため、この範囲条件が必要です。空の森は空列で、コールバックは呼びません。コールバックで入力グラフや渡された状態を変更しないでください。

## 原理・計算量・注意点

各コールバック・状態のコピー／構築を O(1) とすると時間 O(N+E)、空間 O(N)。実際には状態サイズと演算の費用を掛けて評価します。文字列連結や可変長ベクトルの DP が自動的に線形時間になるわけではありません。

基本例は独立集合の数です。状態は `{白,黒}`、子の寄与は `{子の白+黒,子の白}`、merge は成分ごとの積、単位元は `{1,1}`。実際の大きな問題では法を取り、中間の積が型に収まるようにします。DP の算術は利用者のコールバックに任せ、ライブラリによる自動オーバーフロー検出は行いません。

複雑な木DPでは、RootedTree.postorder を直接ループし、子の状態を問題固有の方法で結合する書き方も選べます。

## 使用する問題・検証

[EDPC P - Independent Set](https://atcoder.jp/contests/dp/tasks/dp_p)。[ジャッジ形式](../../verify/tree/independent_set.test.cpp)は mod 10^9+7。

全根の部分木距離和、S と R が異なる型、文字列による非可換集約、送受信方向・元辺ID、空ではコールバックなし、独立集合の全頂点部分集合、20万頂点。

共通の [検証入口](../../verify/run_tree.py) は C++17・警告エラー・debug/release・単独 include・複数翻訳単位・使用例・独立オラクル・ジャッジ入出力を確認します。`--sanitize` で ASan/UBSan。**オンライン AC は未確認**です。
