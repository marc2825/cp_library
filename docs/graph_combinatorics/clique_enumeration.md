---
title: 全クリーク・極大クリーク列挙
documentation_of: //graph_combinatorics/clique_enumeration.hpp
---

# 全クリーク・極大クリーク列挙

互いに全て隣接する頂点集合を列挙します。一頂点もクリークです。頂点を追加できない「極大」と、頂点数が全体で一番大きい「最大」は異なります。

[全列挙・P/X/pivot の図解](../../visualizations/graph_combinatorics/index.html#cliques)

## usage

```cpp
#include <cassert>
#include "graph_combinatorics/clique_enumeration.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1); g.add_edge(0,2); g.add_edge(1,2); g.add_edge(2,3);
    int all=0, maximal=0;
    cp::enumerate_cliques(g,[&](const auto&) { ++all; return true; });
    cp::enumerate_maximal_cliques(g,[&](const auto&) { ++maximal; return true; });
    assert(all==9 && maximal==2); // 極大: {0,1,2} と {2,3}
    int calls=0;
    bool completed=cp::enumerate_cliques(g,[&](const auto&) {
        ++calls; return false;
    },true); // 空集合の一件目で止める
    assert(!completed && calls==1);
}
```

## API と原理

- `bool enumerate_cliques(g,emit,include_empty=false)`：全クリーク。頂点番号が大きい隣接先をソートし、現在の候補と追加頂点の隣接先の共通部分へ進みます。各集合は最小頂点を根にして昇順に作るため、一度だけ現れます。頂点数の64制限はありません。
- `bool enumerate_maximal_cliques(g,emit,include_empty=false)`：極大クリーク。64頂点までのビット集合を使う Bron–Kerbosch 法です。P は追加可能、X は既に処理した候補。P と X がともに空の場合だけ出力します。P∪X から pivot を選び、pivot に隣接しない P の頂点だけで分岐します。

emit は `bool(const std::vector<int>& vertices)`。頂点は元の番号の昇順。false で停止し、列挙関数も false を返します。true の戻り値は列挙完了です。コールバックに渡す vector は一時的な参照なので、保存するならコピーしてください。グラフはコールバック中も変更しません。

単純無向グラフが条件です。重みは無視し、空・非連結も扱います。全列挙の include_empty=true は空集合を最初に一度出します。極大列挙では空集合が極大になるのは頂点数0のときだけです。既定では空グラフから何も出力しません。極大列挙の65頂点以上は release でも length_error です。

## 計算量・使い分け

全列挙の出力数 C、番号の大きい隣接先の最大数 D とすると、準備と探索は O(N+M log(M+1)+(D+1)C)、作業領域 O(N+M+D²)。結果を全保存する領域や利用者のコールバック時間は別です。探索の再帰深さは最大クリークのサイズです。

極大列挙は最悪指数時間（保守的上界 O(N²2^N)）、作業領域 O(N²+N+M)。極大クリーク数自体も指数的になります。最大の一つが必要なら [最大クリーク](maximum_clique.md) を使う方が目的に合います。

## 問題・検証

[Library Checker Enumerate Cliques](https://judge.yosupo.jp/problem/enumerate_cliques) の [ラッパー](../../verify/graph_combinatorics/enumerate_cliques.test.cpp) は全ての空でないクリークを集計します。[動く例](../../examples/graph_combinatorics/enumeration.cpp)。

全部分集合との一致、重複なし、極大性、空集合の扱い、打切り、64頂点完全グラフの極大一件、20万頂点の星の全クリークを確認します。[カテゴリの検証](README.md#動く例問題検証) / [参考資料](references.md)
