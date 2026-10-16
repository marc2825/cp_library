---
title: 行列累乗による漸化式
documentation_of: //linear_recurrence/matrix_recurrence.hpp
---

# 行列累乗による漸化式

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_recurrence/index.html#matrix) / [実装](../../linear_recurrence/matrix_recurrence.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "linear_recurrence/matrix_recurrence.hpp"
int main() {
    std::cout << cp::linear_recurrence_matrix({0, 1}, {1, 1}, 10) << '\n';
    // x <- x+3 を定数1を含む状態で表す。合成数法も使える。
    const auto s = cp::matrix_power_apply<12>({{1, 3}, {0, 1}}, {2, 1}, 5);
    std::cout << s[0] << ' ' << s[1] << '\n';
}
```

[動く使用例](../../examples/linear_recurrence/matrix_recurrence.cpp)。ライブラリのルートをincludeパスに指定します。

## API・使用条件

`linear_recurrence_matrix<Mod=998244353>(initial,c,uint64_t n) -> int`

`initial={a[0],…,a[d−1]}` と `c` は同じ非空長。a[t]=Σ_(j=0..d−1)c[j]*a[t−1−j] (t≥d) のa[n]を返します。長さ違い・空はinvalid_argument。n<dでは該当する初期項を正規化して返します。係数の末尾0も次数dに含めます。

`matrix_power_mod<Mod>(A,uint64_t k) -> vector<vector<int>>`

正方行列Aのk乗です。k=0は単位行列、0×0行列は空。不正な形は指数0でもinvalid_argumentです。入力係数は負数も正規化します。

`matrix_power_apply<Mod>(A,initial,uint64_t k) -> vector<int>`

**列ベクトル**のs(k)=A^k*s(0)を返します。Aは正方行列で、initialの長さが行数と一致する必要があります。0×0と空の状態は成功した空列。k=0でも形を検査します。状態が複数の数列からなる場合、定数1を状態に足す非斉次漸化式などに使えます。

法は1〜2×10^9の任意の整数で、合成数法・法1も可。法1の単位行列も全成分0です。

全てC++17・namespace cp、入力は不変。負の係数も[0,Mod)へ正規化し、INT_MIN/MAXを扱います。項番号・回数は0始まりのuint64_t全域で、負数を渡さないでください。[共通仕様](README.md#共通仕様)も参照してください。

## 原理・計算量

状態を[a[t],a[t−1],…,a[t−d+1]]ᵀにすると、遷移行列の先頭行がc、下の行は1つ前の値をコピーする副対角成分1になります。初期状態は[a[d−1],…,a[0]]ᵀで、n−(d−1)回だけ遷移を適用します。指数は先に引き算するためuint64_t最大値でも超過しません。

指数の1ビットごとに状態へ適用し、遷移を二乗します。任意行列の積は可換ではないため、行列は列ベクトルの左から掛ける規約を固定しています。旧 `matrix/matrix_basic.hpp` のグローバル関数・誤綴りの `matrirx_pow` は保持し、新APIはcp名前空間で提供します。

次数／行列サイズdに対してO(d³ log(k+1)+d²)時間、O(d²)補助領域。状態に適用する版は採用ビットでO(d²)、遷移の二乗でO(d³)。漸化式のn<dはO(1)です。

剰余の加減乗算はO(1)、中間の積はlong longに収め、加算ごとに剰余を取ります。

## 使用する問題・検証範囲

[Library Checker: kth_term_of_linearly_recurrent_sequence](https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence) / [ジャッジ形式](../../verify/linear_recurrence/matrix_recurrence.test.cpp)。行列全体の累乗には[Pow of Matrix](https://judge.yosupo.jp/problem/pow_of_matrix)の[ラッパ](../../verify/linear_recurrence/matrix_power_mod.test.cpp)もあります。次数の大きな漸化式の提出ではKitamasaまたはBostan–Moriを選びます。

[検証入口](../../verify/run_linear_recurrence.py)で6ヘッダの単独・逆順・混在include、複数翻訳単位、4使用例、5ジャッジ形式・320ケースを確認します。BMは9,233有限列を独立した連立方程式の最小可解次数と比較。600漸化式・160任意行列、Python任意精度3,504クエリで各重み、初期項の順、合成数法・法1、非NTT素数、負数・整数極値、uint64最大値、空・次数1・遅い不一致・係数の末尾0を検証します。

192次Kitamasa、2,048観測／最短次数1,024のBM、48×48密行列、16,384次Bostan–Moriと2^63番目の項も確認します。通常／ASan・UBSan・LeakSanitizer、debug/release。図解は3,446入力・55,104スナップショットとブラウザ操作を検証します。オンラインACは未確認です。
