---
title: Knuth Optimization
documentation_of: //dp_optimization/knuth_optimization.hpp
---

# Knuth Optimization

分割点の二方向の単調性を使い、区間DPを二乗時間にします。

[実装](../../dp_optimization/knuth_optimization.hpp) / [操作できる図解](../../visualizations/math_a/index.html#knuth_optimization) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`knuth_optimization(n,cost)` → 既存の`IntervalDPResult`。`dp[l][r]=min_{l<k<r}(dp[l][k]+dp[k][r])+cost(l,r)`。長さ0/1は0。`split`は最左の分割点、`value()`は全体の費用、`merge_order()`は子から親への`{l,k,r}`列です。

最左optが`opt[l][r-1]<=opt[l][r]<=opt[l+1][r]`を満たすこと。costが区間包含について単調かつ四辺形不等式を満たすことが十分条件です。costはkに依存しません。非負のSlimesの区間和が典型例。負の次元はinvalid_argument、DP加算の超過はoverflow_error。前提は自動検査しません。

## 原理と計算量

長さ順に区間を埋め、前の長さで得た左右の分割点の間だけを走査します。探索幅は区間ごとではなく全体で望遠和になり二乗時間です。結果型はinterval_dpと共有します。

costがO(1)ならO(N^2)時間/O(N^2)領域。復元O(N)。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "dp_optimization/knuth_optimization.hpp"

int main() {
    std::vector<long long> prefix{0,10,30,60};
    auto result=cp::knuth_optimization(3,[&](int l,int r) {return prefix[r]-prefix[l];});
    std::cout << result.value() << " " << result.merge_order().size() << "\n";
}
```

出力:

```text
90 2
```

[動く使用例](../../examples/dp_optimization/knuth_optimization.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [AtCoder DP N - Slimes](https://atcoder.jp/contests/dp/tasks/dp_n)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [KACTL KnuthDP](https://github.com/kth-competitive-programming/kactl/blob/main/content/various/KnuthDP.h)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
