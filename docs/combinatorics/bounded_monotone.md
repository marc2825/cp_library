---
title: 上下境界付き単調列・格子路
documentation_of: //combinatorics/bounded_monotone.hpp
---

# 上下境界付き単調列・格子路（C-02）

[ヘッダ](../../combinatorics/bounded_monotone.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#bounded_monotone)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/bounded_monotone.hpp"
int main() {
    std::cout << cp::count_bounded_monotone_sequences({0, 1, 1, 2}, {2, 3, 4, 4}) << '\n';
}
```

出力:

```text
45
```

## API・条件

`count_bounded_monotone_sequences<Mod=998244353,Root=3>(lower,upper,strict=false,max_extent=1000000)` は `lower[i]≤a[i]≤upper[i]` の整数非減少列を数えます。両端を含むsigned64配列。strict=trueなら狭義増加。空列は1、不可能な境界は0。入力の境界自体は単調でなくても構いません。

strictではa[i]−iへ変換し、下限のprefix maxと上限のsuffix minを取ります。変換後の `S=N+max(upper)−min(lower)+2` がmax_extent以下、S≤Mod、S≤`fps_max_size<Mod,Root>()`であることが必要です。これは座標幅に依存し、巨大な幅を座標圧縮するAPIではありません。サイズ不一致は `invalid_argument`、予算/NTT容量超過は `length_error`、標数条件違反は `domain_error`。途中の座標差はBigIntで計算し、signed64端でもoverflowしません。BigInt依存のためBoostヘッダが必要です。

## 仕組み

N本の右向き辺の高さをa[0]..a[N−1]と見れば、(0,min lower)から(N,max upper)への右/上格子路と一対一です。列iの許可頂点は、下限がlower[i−1]、上限がupper[i]（両端は隣接境界）です。右辺を渡った直後に下限、渡る直前に上限が効きます。

長い辺で長方形を二分し、境界と交わる部分を再帰します。全域が許可されたw×h長方形では、左端入力W[j]・下端入力B[i]から右端・上端へ4つの畳み込みで伝播します。左端から右端へのカーネルは `C(w−1+t,t)`。左端から上端のi番目へは

`sum_j W[j] * (h−1−j+i)! / ((h−1−j)! i!)`

です。逆階乗で重みを付けたWと階乗列との畳み込みから取り出せます。左/下の外部から最初に入る辺が異なるため二重計上しません。領域外は0、256セル以下は直接DP。2本の単調な境界が交わるブロックだけを細分化します。

## 計算量

O(S log² S)体演算、O(S)追加空間。各再帰段の境界長の合計O(S)、長方形転送O((w+h)log(w+h))、再帰深さO(log S)。座標変換はO(N)回の高々64+log N bit整数演算。

## 使用する問題

二つの列の間の増加列、上下の階段境界から出ない格子路、Catalan型の数え上げ。高さ座標は連続した整数格子で数えます。

## 検証

小境界の全列挙、ランダムな非単調境界を素朴prefix DPと17,162比較。幅/高さの偏った長方形、2万要素の無制約二項係数・Catalan・一致境界、signed64両端、strictを検証。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
