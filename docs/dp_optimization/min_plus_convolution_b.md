---
title: Min-Plus Convolution（凸・凹）
documentation_of: //dp_optimization/min_plus_convolution.hpp
---

# Min-Plus Convolution（凸・凹）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#min_plus_convolution) / [実装](../../dp_optimization/min_plus_convolution.hpp) / [一次資料と設計](../b_12_15/references.md)

既存の凸列版に加え、凹列同士と任意列×凹列のMin-Plus、および凹列同士のMax-Plusを提供します。

## usage

```cpp
#include <iostream>
#include "dp_optimization/min_plus_convolution.hpp"
int main() {
    auto c=cp::min_plus_convolution_arbitrary_concave(std::vector<long long>{3,1,4},std::vector<long long>{0,3,4});
    std::cout << c[2] << '\n';
}
```

出力:

```text
4
```

## API・使用条件

`min_plus_convolution_convex_convex(a,b)`、`_arbitrary_convex`、`_convex_arbitrary`、`_concave_concave`、`_arbitrary_concave`、`_concave_arbitrary`。`max_plus_convolution_concave_concave`も同梱。入力vector<T>、返値c[k]=min/max_{i+j=k}(a[i]+b[j])。空があれば空。凸は差分が非減少、凹は非増加。

## 仕組み・境界条件

凸×凸は小さい傾きをmerge。凹×凹の最小値は各反対角線の端点にあります。任意×凹は任意列を長さM以下に分割し、前半／反転後半それぞれで凹カーネルの部分包絡をdequeへ保持します。新候補が勝つprefixの境界を二分探索します。形状条件の検査は行わず利用者の前提。標準整数の加算超過はoverflow_error、符号反転を使わないためLLONG_MINも和が入れば扱えます。既存[詳細](min_plus_convolution.md)も保持。

## 計算量

凸×凸・凹×凹はO(N+M)、任意×凸はO((N+M)log(N+M))、任意×凹はO(N log(M+1)+M)。出力を含む領域O(N+M)。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Min Plus Convolution (Convex and Arbitrary)](https://judge.yosupo.jp/problem/min_plus_convolution_convex_arbitrary)、[凹と任意](https://judge.yosupo.jp/problem/min_plus_convolution_concave_arbitrary)。 [動く小問題](../../examples/dp_optimization/min_plus_convolution_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
