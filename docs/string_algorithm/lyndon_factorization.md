---
title: Lyndon Factorization
documentation_of: //string_algorithm/lyndon_factorization.hpp
---

# Lyndon Factorization

文字列を、辞書順が非増加となるLyndon文字列の列へ一意に分解します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#lyndon) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`lyndon_factorization(string_view s) -> vector<pair<int,int>>`。元の文字列の半開区間[l,r)を左から返します。空は空、同じ因子の繰返しは別の区間として残します。辞書順はunsigned byte順で、NUL・高位byteも可。入力不変。長さはINT_MAX以下で、超過はlength_error。

Lyndon文字列は非空で、全ての非自明な巡回シフトより真に小さい文字列です。非増加順は因子列の辞書順であり、因子の長さ順ではありません。

## Usage

```cpp
#include <iostream>
#include "string_algorithm/lyndon_factorization.hpp"
int main() {
    for (auto [l, r] : cp::lyndon_factorization("banana")) std::cout << l << ' ' << r << '\n';
}
```

出力:

```text
0 1
1 3
3 5
5 6
```

[コンパイルできる使用例](../../examples/string_algorithm/lyndon_factorization.cpp)。

## 原理

Duval法で比較中の位置jと、繰返しの先頭側kを持ちます。より大きな文字に出会うとkを候補先頭へ戻し、同じならkを進めます。小さくなった位置で因子長j−kを確定し、同長の繰返しをまとめて出力します。

## 計算量

O(n)時間・O(n)出力領域、作業領域O(1)。byteの比較をO(1)として数えます。

## 使用する問題と検証

[Library Checker Lyndon Factorization](https://judge.yosupo.jp/problem/lyndon_factorization)。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
