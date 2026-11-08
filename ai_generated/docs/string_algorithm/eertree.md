---
title: Eertree（回文木）
documentation_of: //string_algorithm/eertree.hpp
---

# Eertree（回文木）

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#eertree) / [実装](../../string_algorithm/eertree.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/eertree.hpp"
int main() {
    cp::Eertree t("ababa"); auto counts = t.occurrence_counts();
    std::cout << t.distinct_count() << ' ' << t.nodes()[t.longest_suffix_node()].length << ' ' << counts.back() << '\n';
}
```

[実行できる使用例](../../examples/string_algorithm/eertree.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`Eertree()` または `(string_view)`、`append(unsigned char)->node_id`、`size/distinct_count/longest_suffix_node`、`nodes() const`、`occurrence_counts() const`。nodesの0/1は長さ−1/0の根、実回文は2以降。Nodeは `length/link/first_end/next/suffix_count`。first_endは最初の出現の末尾位置（0始まり）、suffix_countはそのノードが最長回文suffixだった回数です。

occurrence_countsはnode ID順の総出現回数、2個の根は0、非破壊で追記後も呼べます。任意byte・NUL可、空入力は異なる回文0個、最長suffixは空根。ノード参照はappendによるvector再確保で無効になる場合があります。削除・巻戻しは提供しません。

## 計算量・原理

長さn、σ≤256として構築ならしO(n log(σ+1))、1回appendは最悪O(n)、個数集約O(n)。格納O(n)。各appendで新しい異なる回文は高々1個。

長さ−1と0の2根を用意し、現在の最長回文suffixからsuffix linkをたどって新文字で挟める回文を探します。新しい回文のsuffix linkを設定し、総出現回数は長い（後から作った）ノードからlinkへ伝播します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/enumerate_palindromes)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
