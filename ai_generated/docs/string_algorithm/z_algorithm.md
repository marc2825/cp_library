---
title: Z Algorithm
documentation_of: //string_algorithm/z_algorithm.hpp
---

# Z Algorithm

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#z) / [実装](../../string_algorithm/z_algorithm.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/z_algorithm.hpp"
int main() {
    for (int x : cp::z_algorithm("aabcaabxaaaz")) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/string_algorithm/z_algorithm.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`z_algorithm(string_view s) -> vector<int>`、`z_algorithm(const vector<T>& s)`。

z[i]はs全体とs[i..n)の最長共通接頭辞の長さです。**z[0]=n**。空入力は空、1要素は{1}です。vector版は要素の==だけを使います。

## 原理・計算量

先頭と一致している最も右の区間[left,right)を保持します。iがその内側ならmin(right−i,z[i−left])を再利用し、それより右だけを比較して延ばします。右端が進む比較と、各iで止まる比較だけなのでO(n)時間・O(n)領域です。

## 使用する問題・検証範囲

[Library Checker Z Algorithm](https://judge.yosupo.jp/problem/zalgorithm)の[ラッパ](../../verify/string_algorithm/z_algorithm.test.cpp)。各suffixと先頭を直接比較した値を独立オラクルにします。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。
