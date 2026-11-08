---
title: Aho–Corasick
documentation_of: //string_algorithm/aho_corasick.hpp
---

# Aho–Corasick

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#aho) / [実装](../../string_algorithm/aho_corasick.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/aho_corasick.hpp"
int main() {
    cp::AhoCorasick ac; ac.insert("he"); ac.insert("she"); ac.insert(""); ac.build();
    for (auto x : ac.count_occurrences("shehe")) std::cout << x << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/string_algorithm/aho_corasick.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`AhoCorasick`、`insert(string_view)->pattern_id`、`build()`、`pattern_count/node_count`、`count_occurrences(text)->vector<size_t>`、`scan(text,callback)`。callbackは `(pattern_id,end_exclusive)`。位置順に出力しますが、同じ終端位置内のID順は保証しません。空patternは0..nの全文字境界、重複patternは別IDで各々出力。任意unsigned byte、NULも可。

insert後にはbuildが必要で、未buildの照合はlogic_error。再build・build後のinsertも可。文字列長とノード数はintの範囲。countの出力は重複込みID順。入力は変更しません。

## 計算量・原理

ノード数V、文字種σ≤256、最大pattern長L。疎な辺をmapで保存してO(V+pattern数)領域。挿入O(総pattern長 log(σ+1))、buildはfailureをたどるため保守的上界O(V L log(σ+1))（密な遷移表の線形構築ではない）。照合はならしO(text長 log(σ+1))、countは追加O(V+pattern数)、scanは出現件数を加算。countの補助O(V+pattern数)。

Trieのfailure linkを最長の接尾辞状態へ張ります。scanは出力を持つ接尾辞へのoutput linkで報告し、countは訪問回数をfailure木の逆BFS順で親へ集約します。空patternのためrootには最初の文字境界の1回も数えます。

## 使用する問題・検証

[関連問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_D)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
