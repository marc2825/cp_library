---
title: 巨大なnの階乗 mod p
documentation_of: //combinatorics/large_factorial.hpp
---

# 巨大なnの階乗 mod p（N-07）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#large_factorial)

## API・条件

`large_factorial_mod(uint64 n,int prime,max_block=1000000)`。primeは2..2×10^9の素数、合成数はinvalid_argument。n≥primeなら0。r=min(n,prime−1−n)の√rがmax_blockを超えるとlength_error。n=0は1。固定NTT素数だけでなく実行時の任意の素数を受けます。

## 使用例

[そのままコンパイルできるソース](../../examples/combinatorics/large_factorial.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "combinatorics/large_factorial.hpp"

int main() {
    std::cout << cp::large_factorial_mod(1000, 1009) << '\n';
}
```

出力:

```text
782
```

## 原理・計算量

Wilsonの定理で短い側へ移し、長さB=⌊√r⌋の積多項式を構築、Bの倍数で多点評価してブロック積を得ます。積・剰余木と既存convolution_modの3本NTTを共有。M(B)=O(B log B)としてO(M(B) log B+√r)時間、O(B log B)領域。r≤1は定数時間です。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

線形階乗表を確保できない引数の階乗、Library Checker Factorial Mod型の問い合わせ。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
