---
title: FWHT / Walsh–Hadamard変換
documentation_of: //set_function/fwht.hpp
---

# FWHT / Walsh–Hadamard変換

ビット集合のXORに対応する高速変換です。3.16のヘッダは3.19の共通実装をincludeします。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#fwht) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`fwht<T>(vector<T>& f, bool inverse=false)`。正の2冪長Nのみ。空・非2冪は変更前にinvalid_argument。添字は自然順（Hadamard順）で、F[s]=Σ_t (-1)^popcount(s&t)f[t]、順変換は正規化なし。長さ1はそのままです。

逆変換はビット段を逆順に進め、各butterflyで2で割ります。Tは可換な加法群とT(2)による除算が必要。独自mod型は2が可逆な場合に使います（N>1の標数2は不可）。整数では各除算が割り切れることを検査し、不成立はdomain_error。標準整数の途中の加減算超過はoverflow_error、符号なしの負数も拒否。途中の例外では既に処理した値が残ります。独自型の算術・浮動小数点誤差は利用者が管理します。

## Usage

```cpp
#include <iostream>
#include "set_function/fwht.hpp"
int main() {
    std::vector<long long> a{1, 2, 3, 4};
    cp::fwht(a);
    for (auto x : a) std::cout << x << ' ';
    std::cout << '\n';
    cp::fwht(a, true);
    std::cout << a[0] << ' ' << a[3] << '\n';
}
```

出力:

```text
10 -2 -4 0
1 4
```

[コンパイルできる使用例](../../examples/set_function/fwht.cpp)。

## 原理

1ビットごとに(x,y)を(x+y,x-y)へ置き換えます。Hadamard行列HはH²=NIを満たすため、逆は同じbutterflyと正規化で得られます。整数の逆変換は一段ずつ2で割り、丸めによる誤った整数復元を防ぎます。

## 計算量

O(N log N)時間・O(1)追加領域。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

[Library Checker Bitwise XOR Convolution](https://judge.yosupo.jp/problem/bitwise_xor_convolution)。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
