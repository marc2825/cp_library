---
title: 商列挙（数論入口）
documentation_of: //number_theory_basic/quotient_enumeration.hpp
---

# 商列挙（数論入口）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#quotient_enumeration) / [実装](../../number_theory_basic/quotient_enumeration.hpp) / [一次資料と設計](../b_12_15/references.md)

floor(N/i)が一定となる最大区間を、既存ユーティリティから共有します。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/quotient_enumeration.hpp"
int main() {
    cp::for_each_quotient(10,[](auto l,auto r,auto q){if(q==2) std::cout << l << ' ' << r << ' ' << q << '\n';});
}
```

出力:

```text
4 5 2
```

## API・使用条件

`for_each_quotient(uint64_t n,callback(l,r,q))`。l,rは両端を含む区間で1<=l<=r<=n、q=n/l=n/r。n=0はcallbackなし。

## 仕組み・境界条件

本APIは例外的に閉区間[l,r]を返します。r=Nのときr+1を作らず終了するためUINT64_MAXでも折り返しません。基本ユーティリティの[詳細](../basic_utility/quotient_enumeration.md)を共有。

## 計算量

O(sqrt N)回のcallback、追加領域O(1)。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

約数和やDu Jiaoで同じ商を持つ添字をまとめる問題。 [動く小問題](../../examples/number_theory_basic/quotient_enumeration_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
