---
title: 区間DP・二分割の最小化
documentation_of: //dp_optimization/interval_dp.hpp
---

# 区間DP・二分割の最小化

半開区間を二つの短い区間へ分割する形の区間DPです。隣接結合や行列連鎖積のように、結合費用を(l,k,r)から定義できます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#interval) / [実装](../../dp_optimization/interval_dp.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "dp_optimization/interval_dp.hpp"
int main() {
    std::vector<long long> a{10,20,30}, prefix(4);
    for (int i=0;i<3;++i) prefix[i+1]=prefix[i]+a[i];
    auto result = cp::interval_dp(3, [&](int l,int,int r) { return prefix[r]-prefix[l]; });
    std::cout << result.value() << '\n';
    for (auto [l,k,r] : result.merge_order()) std::cout << l << ' ' << k << ' ' << r << '\n';
}
```

[動く使用例](../../examples/dp_optimization/interval_dp.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`interval_dp(int n, Cost cost) -> IntervalDPResult`。

- nは非負。`cost(l,k,r)` はlong longの結合費用。`l<k<r`。
- `dp[l][r]` は区間[l,r)の最小費用。長さ0・1は0。
- `split[l][r]` は選んだk。長さ0・1は−1。同点は最小のk。
- `value()` はdp[0][n]。
- `merge_order()` は `{l,k,r}` の列。子の結合を親より先に返す。
- 空では値0・空の結合順。n<2ではcostを呼びません。

このAPIは上記の二分割の形に限定します。両端を取り除くDP、任意の初期値や多状態を持つ区間DPでは、同じ長さ順の走査を参考に専用の表を作ってください。

## 原理・計算量・注意点

costをO(1)として時間O(N³)、空間O(N²)。merge_orderはO(N)時間・空間で、明示的なスタックを使います。

長さ順に表を埋め、左右の最適値+costを比較します。負の費用も可。加算の中間値がlong longを超える場合はoverflow_error。cost自身の計算・戻り値変換の範囲は利用者の責任です。

隣接結合ではcostは区間和、行列連鎖積なら行列次元の積など、kに依存する費用にもできます。最適分割点の単調性は仮定せず、Knuthや分割統治の最適化は含みません。

## 使用する問題・検証範囲

[EDPC N - Slimes](https://atcoder.jp/contests/dp/tasks/dp_n)。

200個の小さな入力を「現在隣り合う塊を全順序で結合する」独立探索と比較。分割点に依存する費用、負値、空・1要素、復元順、最大値と演算超過、160要素。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
