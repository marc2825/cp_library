---
title: LCS・最長共通部分列
documentation_of: //dp_optimization/lcs.hpp
---

# LCS・最長共通部分列

二つの列に共通し、順序を保つ最長の部分列を求めます。長さだけをO(M)空間で求める入口と、対応する添字を復元する入口があります。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#lcs) / [実装](../../dp_optimization/lcs.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <string>
#include "dp_optimization/lcs.hpp"
int main() {
    std::string a = "abcde", b = "ace";
    auto result = cp::lcs(a, b);
    std::cout << result.length << '\n';
    for (auto [i, j] : result.matches) { (void)j; std::cout << a[i]; }
    std::cout << '\n';
}
```

[動く使用例](../../examples/dp_optimization/lcs.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`lcs(a,b,equal=equal_to<>{}) -> LCSResult`、`lcs_length(a,b,equal) -> int`。

- `length`: 長さ。
- `matches`: `{a側の添字,b側の添字}` の列。両方で昇順。一致した値は a[i] などで取得します。
- 両列は size()、添字アクセスを持ち、長さは INT_MAX 未満。長さのみの関数では a の範囲forも使います。
- std::string、vector<int> などに対応。equal は二つの要素を比較する等値判定です。
- 片側が空なら0・空の対応列。同点解の辞書順保証はありません。

## 原理・計算量・注意点

時間 O(NM+N+M)。lcs は O((N+1)(M+1)) 空間、lcs_length は O(M+1) 空間です。

接頭辞 a[0..i),b[0..j) を状態にします。上・左の値、一致した場合の対角+1を比較します。復元は一致を採用できれば対角へ、そうでなければ上／左へ移動します。

部分文字列と違い、間の要素を飛ばせます。std::string はバイトの列です。Unicodeの文字単位で使う場合は、利用者がコードポイントの列へ変換してください。比較器の費用は計算量に掛かります。

## 使用する問題・検証範囲

[AOJ ALDS1_10_C: Longest Common Subsequence](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_10_C)。

二値文字列3,969組を全ての部分列と比較し、復元添字の順序・一致・長さを検査。空、整数列、大小文字を同一視する判定、長さ2,000の行圧縮も確認。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
