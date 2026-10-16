---
title: SOS DP（部分集合の集約）
documentation_of: //set_function/sos_dp.hpp
---

# SOS DP（部分集合の集約）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/set_function/index.html#sos) / [実装](../../set_function/sos_dp.hpp) / [一次資料](references.md)

## usage

```cpp
#include <algorithm>
#include <iostream>
#include <vector>
#include "set_function/sos_dp.hpp"
int main() {
    std::vector<int> sum{3, 1, 4, 2}, best = sum;
    cp::sos_dp(sum);
    cp::sos_dp(best, [](int a, int b) { return std::max(a, b); });
    for (int x : sum) std::cout << x << ' ';
    std::cout << '\n';
    for (int x : best) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/set_function/sos_dp.cpp)。ライブラリのルートをincludeパスに追加します。

## API・使用条件

`sos_dp(vector<T>& f) -> void`

部分集合の和をin-placeで計算します。subset_zeta_transformと同じAPI契約・整数超過検査です。

`sos_dp(vector<T>& f,Merge merge) -> void`

全ての部分集合T⊆Sについて元のf[T]を集約し、f[S]へ上書きします。mergeは**結合的・可換**で、引数を書き換えずに集約値を返す関数です。Tはその結果を代入できること。min/max、bitwise OR、値と添字を持つ比較結果などにも使えます。各S自身を必ず含むため、単位元の引数やT(0)・デフォルト構築は不要です。

長さは正の2冪で、空を含む違反はinvalid_argument。形の検査はmergeを呼ぶ前に行います。長さ1はそのまま。和以外のSOSへメビウス変換を適用することはできません。独自merge内の算術・例外は利用者が管理します。和の整数超過検査が必要なら引数1個の版を使ってください。

C++17・namespace cp。[共通仕様と旧API](README.md#共通仕様)も参照してください。

## 原理・計算量

ビットiごとに、iを持たない集合の集約値を、iを持つ集合へ合成します。処理したビットだけ自由に変えられる部分集合を集約している、という不変条件を保ちます。2つの入力が表す集合の族は互いに重ならないため、同じ項を二重に数えません。ゼータはこのmergeを加算に固定した場合です。

N=2^nとしてO(nN)時間、追加空間O(1)。mergeは正確にnN/2回（n=0は0回）呼びます。独自mergeの演算費用を掛けて考えます。

## 使用する問題・検証範囲

[CSES SOS Bit Problem](https://cses.fi/problemset/task/1654)の[和のラッパ](../../verify/set_function/sos_dp.test.cpp)、[Codeforces 165E Compatible Numbers](https://codeforces.com/problemset/problem/165/E)の[最大値のラッパ](../../verify/set_function/sos_max.test.cpp)。

[検証入口](../../verify/run_set_function.py)は9ヘッダの単独／逆順／混在includeと複数翻訳単位、7使用例、4ジャッジ形式・256ケースを確認します。2,047基準mask・177,146組、6,654三値表・180ランダム表、Python任意精度2,428クエリを、定義から直接集計した和・交互和・集約と比較します。空集合・0ビット・63/64ビット、法12の型、複素数、単位元不要の型、262,144個のint8演算、符号付き／符号なし極値、途中超過と失敗前の状態、2^20項の全出力と正確な更新回数を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行し、移動した3.12 DP APIも回帰検証します。図解は2,853入力・50,838スナップショット、15拒否条件とブラウザ操作を確認します。オンラインACは未確認です。
