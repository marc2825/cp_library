---
title: 任意乗法的関数の大規模総和
documentation_of: //number_theory_convolution/multiplicative_summatory.hpp
---

# 任意乗法的関数の大規模総和（N-27）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#multiplicative_summatory)

## API・条件

`multiplicative_summatory_table(prime_prefix,prime_power)` と `.total()`だけを返す `multiplicative_summatory`。fは乗法的でf(1)=1。入力表は各商でΣ_{p≤x}f(p)、callbackはf(p^e)。f(p)が多項式である必要はありませんが、その素数prefixを呼出側で与える必要があります。任意のcallbackだけから素数和を自動で高速構築するという意味ではありません。表とcallbackの一致は契約であり自動検査しません。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_convolution/multiplicative_summatory.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include "number_theory_convolution/prime_sum.hpp"
#include <iostream>
#include "number_theory_convolution/multiplicative_summatory.hpp"

int main() {
    auto prime_count = cp::prime_sum_sieve<cp::BigInt>(100,
        [](auto x) { return cp::BigInt(x); }, [](auto) { return cp::BigInt(1); });
    for (auto x : prime_count.quotients()) prime_count.set(x, 2 * prime_count.sum(x));
    std::cout << cp::multiplicative_summatory(prime_count,
        [](auto, unsigned e, auto) { return cp::BigInt(e + 1); }) << '\n';
}
```

出力:

```text
482
```

## 原理・計算量

小さい素数を逆順に再導入し、現在の表からΣ_{p′≤p}f(p′)を引いて、最小素因数がpとなる数の寄与を合成します。保守的にO(N^(3/4))環演算と同程度のcallback、O(√N)領域。素数prefixの構築時間は別。Nまでの線形配列を作る既存APIとは異なります。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

合同類に依存する素数項や独自の素数prefixを持つ乗法的関数。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
