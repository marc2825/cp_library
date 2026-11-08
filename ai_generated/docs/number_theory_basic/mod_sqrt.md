---
title: mod平方根（Tonelli–Shanks）
documentation_of: //number_theory_basic/mod_sqrt.hpp
---

# mod平方根（Tonelli–Shanks）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#mod_sqrt) / [実装](../../number_theory_basic/mod_sqrt.hpp) / [一次資料と設計](../b_12_15/references.md)

Euler規準で平方剰余を判定し、2冪部分の誤差をTonelli–Shanksで修正します。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/mod_sqrt.hpp"
int main() {
    std::cout << *cp::mod_sqrt(10,13) << '\n';
}
```

出力:

```text
6
```

## API・使用条件

`mod_sqrt(long long a,long long prime)`→optional<long long>。primeは2以上のsigned64素数。aは負数も正規化。2根のうち小さい根を返し、不存在はnullopt。合成数はinvalid_argument。

## 仕組み・境界条件

p=2とa=0を個別に処理します。非剰余は2から決定的に探すため、探索回数の一律O(log p)保証はしません。既存64bit Miller–Rabinと安全なmod乗算を共有し、montgomeryの狭い法制限を持ち込みません。

## 計算量

素数検査と非剰余探索を除きO(log²p)回のmod乗算。非剰余探索Z候補ならO(Z log p)回を追加、領域O(1)。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Sqrt Mod](https://judge.yosupo.jp/problem/sqrt_mod)。 [動く小問題](../../examples/number_theory_basic/mod_sqrt_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
