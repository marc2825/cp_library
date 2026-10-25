---
title: Cographの標準形
documentation_of: //graph_connectivity/cograph_canonical_form.hpp
---

# Cographの標準形

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#cograph_canonical_form) / [実装](../../graph_connectivity/cograph_canonical_form.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

特殊グラフをcograph（誘導P4を持たない単純無向グラフ）と定め、認識・cotree・衝突のない標準形を返します。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/cograph_canonical_form.hpp"
int main() {
    cp::Graph<int> g(3); g.add_edge(0,1); g.add_edge(1,2);
    auto r=cp::cograph_canonical_form(g);
    std::cout << r->code << '\n';
}
```

出力:

```text
J(L0;U(L0;L0;))
```

## API・使用条件

`cograph_canonical_form(g,colors={})`→optional<Result>。`code`、`vertex_order`（標準順→元頂点）、`cotree`、`root`。Nodeのkindはleaf/disjoint_union/join、leafのvertexとchildren。色はint、空指定なら全0。

非cographはnullopt。有向/ループ/平行辺/色数不一致はinvalid_argument。空グラフのcodeはE、root=−1。同じcodeと同じ順位置の頂点を対応させると色付き同型を得ます。codeはハッシュではなく構造文字列なので衝突しません。

## 仕組み

非連結ならdisjoint union、補グラフが非連結ならjoinとして再帰します。どちらでもなければ非cograph。子のcodeをソートし、色付き葉を区切り文字付きで符号化します。

## 計算量

保守的にO(n³+n² log(n+1))時間、O(n²)領域（int色の文字長を定数とする）。再帰深さO(n)。標準形文字列自身はO(n)。

## 使用する問題・検証

cographの同型判定、系列並列な結合操作で作られたグラフの重複排除。一般同型は別API。

5頂点以下全グラフで誘導P4の列挙と認識を比較し、頂点置換後のcode一致と色を検査。 [実行できる使用例](../../examples/graph_connectivity/cograph_canonical_form.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
