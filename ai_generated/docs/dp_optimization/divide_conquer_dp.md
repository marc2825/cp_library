---
title: Divide and Conquer DP Optimization
documentation_of: //dp_optimization/divide_conquer_dp.hpp
---

# Divide and Conquer DP Optimization

連続区間への分割DPを段ごとに更新し、選んだ区間を復元します。

[実装](../../dp_optimization/divide_conquer_dp.hpp) / [操作できる図解](../../visualizations/math_a/index.html#divide_conquer_dp) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`divide_conquer_dp_stage(previous,cost)` は `DPStageResult {values,parent}`。`previous`は長さN+1の`vector<optional<long long>>`で、到達不能をnulloptにします。`next[r]=min_{l<r}(previous[l]+cost(l,r))`、next[0]は到達不能です。

`divide_conquer_dp(n,groups,cost)` は0個の区間で空列だけ費用0から開始し、`dp[k][r]`・`parent[k][r]`を保持します。`partitions(k,r)`は半開区間の列、到達不能ならnulloptです。

各段で最左の最適なlがrとともに非減少であることが必要です。Monge性は十分条件です。到達不能だけの先頭行では仮のargminを0と定義します。costは有効な非空区間の有限long long値を返す純粋な関数。前提の単調性は検査しません。負の次元・空のpreviousはinvalid_argument、復元範囲外はout_of_range、DP和の超過はoverflow_error。groups>Nは到達不能、N=groups=0は空の復元です。

## 原理と計算量

既存のMonotone Minimaを共有します。中央の行の最適列を見つけると、上下の行で探索すべき列が狭まります。無効なl>=rや到達不能値は無限大として比較し、costを呼びません。

costをO(1)とすると一段O(N log(N+1))時間/O(N)領域、全段O(KN log(N+1))時間/O(KN+K+N)領域。区間復元はO(K)。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "dp_optimization/divide_conquer_dp.hpp"

int main() {
    std::vector<long long> prefix{0,1,3,6};
    auto result=cp::divide_conquer_dp(3,2,[&](int l,int r) {
        auto sum=prefix[r]-prefix[l]; return sum*sum;
    });
    std::cout << *result.dp[2][3];
    auto parts=result.partitions(2,3);
    for (auto [l,r]:*parts) std::cout << " " << l << ":" << r;
    std::cout << "\n";
}
```

出力:

```text
18 0:2 2:3
```

[動く使用例](../../examples/dp_optimization/divide_conquer_dp.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: 非負列をK個に分ける区間和の二乗和最小化（図解と差分検証）。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [KACTL Knuth/単調DPの条件](https://github.com/kth-competitive-programming/kactl/blob/main/content/various/KnuthDP.h)、[共有するMonotone Minima](monotone_minima.md)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
