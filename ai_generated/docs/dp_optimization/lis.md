---
title: LIS・最長増加部分列
documentation_of: //dp_optimization/lis.hpp
---

# LIS・最長増加部分列

最長増加部分列の長さと、その一つの解を元の添字で返します。広義単調増加、任意の厳密弱順序の比較器にも対応し、入力を変更しません。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#lis) / [実装](../../dp_optimization/lis.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "dp_optimization/lis.hpp"
int main() {
    std::vector<int> a{3, 1, 2, 2, 4};
    auto result = cp::lis(a); // false を渡すと広義単調増加。
    std::cout << result.length << '\n';
    for (int i : result.indices) std::cout << a[i] << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/dp_optimization/lis.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`lis(const vector<T>& a, bool strict=true, Compare compare=less<T>{}) -> LISResult`

- `length`: 最長部分列の長さ。
- `indices`: 復元した元の添字。昇順、0始まり。
- `length_ending[i]`: a[i] で終わる最長部分列の長さ。
- strict=true は比較器の狭義順序、false は同値も許す順序。降順なら `std::greater<T>{}`。
- T は負数へ変換できる必要はありません。文字列・符号なし型・LLONG_MIN も比較器で扱えます。入力長は int に収まることが条件です。

空では長さ0・空列。同点解の辞書順最小や最初の添字を選ぶ保証はありません。

## 原理・計算量・注意点

比較をO(1)として O(N log(N+1)) 時間、O(N) 空間。各長さの最良の末尾を tails に残し、狭義では lower_bound、広義では upper_bound で挿入位置を求めます。tails は必ずしも入力の部分列ではないので、各位置の前駆添字を保存して逆にたどります。

旧 `DP/LIS.hpp` は保持しています。新APIは入力を変更せず、減少列でも値を符号反転しません。旧関数が持っていた減少時の入力変更に依存しないでください。

## 使用する問題・検証範囲

[Library Checker: Longest Increasing Subsequence](https://judge.yosupo.jp/problem/longest_increasing_subsequence)。

三値配列9,841個、狭義／広義・昇降順、全 length_ending と復元列。独立したO(N²)のDPと比較。文字列・整数極値・空・20万要素。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
