---
title: Four Russians法（Boolean行列積）
documentation_of: //dp_optimization/four_russians.hpp
---

# Four Russians法（Boolean行列積）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#four_russians) / [実装](../../dp_optimization/four_russians.hpp) / [一次資料と設計](../b_12_15/references.md)

内側の次元を小さいbitブロックに分け、Bの行のOR組合せを表引きするM4RMを実装します。

## usage

```cpp
#include <iostream>
#include "dp_optimization/four_russians.hpp"
int main() {
    cp::BooleanMatrix a{{1,0,1},{0,1,0}},b{{0,1},{1,0},{1,1}};
    auto c=cp::boolean_matrix_product_four_russians(a,b,2);
    for(const auto& row:c) { for(bool x:row) { std::cout << x; } std::cout << '\n'; }
}
```

出力:

```text
11
10
```

## API・使用条件

`boolean_matrix_product_four_russians(BooleanMatrix a,b,unsigned block_bits=0)`。BooleanMatrix=vector<vector<bool>>。AはM×K、BはK×N。1<=block_bits<=12、0はMから自動選択。返値M×NのOR-AND積。

## 仕組み・境界条件

列数が0の空行列は表現できます。Bの行数が0の場合、Nは0と解釈します。長方形でない入力・内側次元不一致はinvalid_argument。ここで扱うのはBoolean行列積で、LCS用のブロックDPは含みません。表引きの前処理があるため、小さい行列で常に素朴法より速いとは限りません。

## 計算量

W=ceil(N/64)、ブロック幅bとしてO(MK+KN+ceil(K/b)(2^b+M)W)時間、出力を含めO(MN+KN+(M+K+2^b)W)領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

到達関係の合成やBoolean行列積。 [動く小問題](../../examples/dp_optimization/four_russians_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
