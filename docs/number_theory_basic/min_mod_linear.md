---
title: 一次式modの最小値
documentation_of: //number_theory_basic/min_mod_linear.hpp
---

# 一次式modの最小値（N-03）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#min_mod_linear)

## API・条件

`min_mod_linear(left,right,a,b,m)` はBigInt引数、[left,right)上で `(a*i+b) mod m` の最小非負剰余と最初の添字を `optional<MinModLinearResult>{value,index}` で返します。left,a,bは負でも可。空区間はnullopt、逆転区間・m≤0はinvalid_argument。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/min_mod_linear.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/min_mod_linear.hpp"

int main() {
    const auto result = cp::min_mod_linear(0, 10, 7, 3, 11);
    std::cout << result->value << ' ' << result->index << '\n';
}
```

出力:

```text
0 9
```

## 原理・計算量

しきい値以下の剰余の個数を2つのfloor sumの差で判定して値を二分探索し、線形合同式から最初の添字を復元します。O(log² m)任意精度整数演算、追加領域O(1)個のBigInt。各演算のビット計算量は別です。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

巨大区間上の合同式最適化、周期的な時刻の最小待ち時間。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
