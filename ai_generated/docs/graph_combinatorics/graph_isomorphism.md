---
title: 一般グラフの厳密同型判定
documentation_of: //graph_combinatorics/graph_isomorphism.hpp
---

# 一般グラフの厳密同型判定

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#graph_isomorphism) / [実装](../../graph_combinatorics/graph_isomorphism.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

未確定を返す1-WL filterを補い、一般グラフの同型を厳密に判定して対応を復元します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/graph_isomorphism.hpp"
int main() {
    cp::Graph<int> a(3),b(3);
    a.add_edge(0,1); a.add_edge(1,2);
    b.add_edge(2,0); b.add_edge(0,1);
    auto r=cp::graph_isomorphism(a,b);
    std::cout << bool(r) << ' ' << r->size() << '\n';
}
```

出力:

```text
1 3
```

## API・使用条件

`graph_isomorphism(a,b,colors_a={},colors_b={})`→optional<vector<int>>。値があれば `mapping[a_vertex]=b_vertex`、なければ非同型。

有向/無向、ループ、多重辺の多重度、任意int頂点色を区別。辺重みは無視。空色配列は全0。色数不一致はinvalid_argument、サイズ/方向/辺数不一致はnullopt。空同士は空mapping。同型が複数ある場合の一意な対応は保証しません。

## 仕組み

両グラフを共通の辞書で同時色精緻化し、色クラス数が合わなければ棄却。非単色クラスの頂点を個別化して候補へ対応付け、再び精緻化します。全頂点が一意になったとき隣接多重度行列を最終確認します。

## 計算量

最悪O(n! n³ log(n+1))時間の保守的上界、再帰の作業を含めO(n²)領域。精緻化だけで区別できない正則グラフなどは指数時間。サイズ上限や確率的打切りは置きません。

## 使用する問題・検証

小中規模の構造照合、色付き有向/多重グラフの厳密比較。cographなら専用の標準形を利用できます。

350例の全頂点置換と比較。有向、ループ、多重辺、置換同型、色の不一致を含む。 [実行できる使用例](../../examples/graph_combinatorics/graph_isomorphism.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
