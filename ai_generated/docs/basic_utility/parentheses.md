---
title: 括弧列・順序木・辞書順k番目
documentation_of: //basic_utility/parentheses.hpp
---

# 括弧列・順序木・辞書順k番目（U-09）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#parentheses)

未使用の開き括弧数と閉じ括弧数から補完方法数をDPします。k番目を求めるときは開き括弧で始まる補完数とkを比較し、辞書順のブロックを選びます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/parentheses.hpp"
int main() {
    cp::ParenthesesTable table(3);
    auto text = table.unrank(3);
    auto tree = cp::parentheses_to_tree(text);
    std::cout << table.count() << ' ' << text << ' ' << table.rank(text) << '\n';
    std::cout << tree.size() << ' ' << cp::tree_to_parentheses(tree) << '\n';
}
```

出力:

```text
5 ()(()) 3
4 ()(())
```

## API・使用条件

`is_balanced_parentheses(string_view)` は `(`,`)` だけからなる正規括弧列かを判定します。空列は正規、それ以外の文字はfalse。

`parentheses_to_tree(text)` は子リスト `vector<vector<int>>`。**括弧で囲まれない仮想根0**を置き、各組を親から子への辺とします。n組→n+1頂点、頂点IDは先行順、兄弟は出現順です。`tree_to_parentheses(children)` は根0から子の格納順に辿って逆変換します。任意の頂点ID配置を許しますが、戻した木のIDは先行順へ正規化されます。根のない空配列・閉路・共有された子・範囲外頂点・非連結は `invalid_argument`。変換は非再帰です。

`ParenthesesTable(n)` の `count()` はBigIntのCatalan数、`unrank(BigInt k)` は0始まり辞書順k番目（`(` < `)`）、`rank(text)` はその逆です。n=0は空列1個。負のnなどは `invalid_argument`、範囲外kは `out_of_range`、長さ違い・不正列は `invalid_argument`。BigIntのため64bitを超える順位にも対応します。

## 計算量

判定・木との変換はO(n)時間（判定はO(1)、変換はO(n)記憶）。Table構築はO(n²)個のBigInt加算・記憶、rank/unrankはO(n)回のBigInt演算。値はO(n)bitなのでbit計算量は加算ベースで概ねO(n³)構築です。

## 使用する問題・検証

正規括弧列のk番目、順序付き根付き木の符号化。小サイズの全2^(2n)文字列から正規列だけを選ぶ独立解と比較し、100組の大きな順位と深さ10万の木の往復も確認します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
