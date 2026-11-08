---
title: Nimberの体演算
documentation_of: //game_theory/nimber.hpp
---

# Nimberの体演算

既存uint64のnim積に、加減算・冪・逆元・除算・平方根を加えます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#nimber) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`Nimber(uint64 value=0)`、`+ - * / += -= *= /= == !=`、`value/inverse/power`。raw版`nimber_power/inverse/sqrt`。

64bit nimberはGF(2^64)。加減算はXOR、通常の整数積・mod素数体ではありません。a≠0の逆はa^(2^64−2)、零除算はdomain_error。0^0=1。超限nimberやゲーム値の大小は扱いません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../game_theory/nimber.hpp"

int main() {
    cp::Nimber a(3), b(5);
    std::cout << ((a * b) / b).value() << " " << (a + a).value() << "\n";
}
```

出力:

```text
3 0
```

[コンパイルして動く使用例](../../examples/game_theory/nimber_b.cpp)。

## 計算量

nim積の費用をMとして冪・逆元O(64M)、平方根63回の二乗。保存O(1)（既存積の小表を除く）。

## 使用する問題・検証

[Nim Product](https://judge.yosupo.jp/problem/nim_product)の体演算拡張。

全小値の逆と全幅ランダム値の逆・平方根・分配則、既存nim積のmex検証を共有。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
