---
title: Suffix Tree（静的）
documentation_of: //string_algorithm/suffix_tree.hpp
---

# Suffix Tree（静的）

SAとLCPから圧縮suffix trieを構築します。入力の全byteを使い、特別な終端文字は要求しません。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#suffix_tree) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`SuffixTree(text={})`、`size/text/nodes`、`contains(pattern)`、`occurrences(pattern)`。Nodeはparent,depth,begin,end,suffix_start,children。

静的な実装で、Ukkonenによるオンライン構築ではありません。edgeラベルは保存したtextの[begin,end)。内部ノードでsuffixが終わる場合もsuffix_startを保持。根は空suffix Nを保持。空patternの出現は0..N、出現位置は昇順。長さ≤(INT_MAX-1)/2。nodes()の参照はオブジェクトの寿命内で有効。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../string_algorithm/suffix_tree.hpp"

int main() {
    cp::SuffixTree tree("banana");
    auto positions = tree.occurrences("ana");
    std::cout << positions[0] << " " << positions[1] << "\n";
}
```

出力:

```text
1 3
```

[コンパイルして動く使用例](../../examples/string_algorithm/suffix_tree_b.cpp)。

## 計算量

構築O(N log N)、保存O(N)。検索O(|pattern| log 256)、列挙は検索に加えO(K log K+K)の保守的費用（圧縮部分木の走査と整列）。

## 使用する問題・検証

[Suffix Array](https://judge.yosupo.jp/problem/suffixarray)、複数の部分文字列の出現位置列挙。

全二値列の全substringとKMP、任意byteのランダムsubstring、圧縮辺・親深さ・空suffixを比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
