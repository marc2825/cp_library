---
title: Wiedemann Algorithm
documentation_of: //linear_algebra/wiedemann.hpp
---

# Wiedemann Algorithm

スカラー射影u^T A^i vにBMを適用し、多項式と検証済み候補解を得ます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#wiedemann) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`wiedemann_polynomial<Mod>(n,apply,left,right)`、`wiedemann_solve<Mod>(n,apply,b,attempts=8,seed=20261018)`→optional<vector<int>>。素数法。

射影多項式は行列全体の最小多項式とは限りません。2n項を生成するため固定の線形作用という前提下ではスカラー列の最小多項式になります。ランダム射影は固定seedで再現可能。求解は全候補の残差を検証し、成功した値だけ返します。試行を使い切ったnulloptは非可解性の証明ではなく、特異行列や射影失敗も含みます。零右辺は零解。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_algebra/wiedemann.hpp"

int main() {
    auto apply = [](const std::vector<int>& x) { return x; };
    auto x = cp::wiedemann_solve(2, apply, std::vector<int>{5, 8});
    if (x) std::cout << (*x)[0] << " " << (*x)[1] << "\n";
}
```

出力:

```text
5 8
```

[コンパイルして動く例](../../examples/linear_algebra/wiedemann_b.cpp)。

## 計算量

matvec費用Tとして射影1回はO(nT+n²+n log Mod)時間、O(n)領域。求解は試行数を掛ける。

## 使用する問題・検証範囲

疎な有限体連立方程式。行列全体の正確な最小多項式には[決定的版](minimal_polynomial.md)。

射影列の全後続項、密行列の残差、可逆な小行列、法3/17/1000000007で比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
