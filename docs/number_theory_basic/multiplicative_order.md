---
title: 乗法位数・Carmichael関数
documentation_of: //number_theory_basic/multiplicative_order.hpp
---

# 乗法位数・Carmichael関数（N-02）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#multiplicative_order)

## API・条件

`carmichael_lambda(m)` と `multiplicative_order(a,m)` はuint64全域、m>0。位数は最小の正の指数でoptional<uint64>、gcd(a,m)≠1ならnullopt。λ(1)=1、法1での位数は1と定めます。m=0はinvalid_argument。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/multiplicative_order.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/multiplicative_order.hpp"

int main() {
    std::cout << cp::carmichael_lambda(360) << ' '
              << *cp::multiplicative_order(7, 40) << '\n';
}
```

出力:

```text
12 4
```

## 原理・計算量

素数冪ごとのλをLCMで合成し、λの素因数を取り除きながら冪を確認します。素因数分解を除くλ合成はO(log m)、位数判定はO(log² m) modular multiply。分解は既存Pollard–Rho/Brentで最悪時間の保証はありません。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

周期の長さ、べき乗列の繰り返し、合成数法の群構造。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
