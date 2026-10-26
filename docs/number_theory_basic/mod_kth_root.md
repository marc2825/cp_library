---
title: mod k乗根
documentation_of: //number_theory_basic/mod_kth_root.hpp
---

# mod k乗根（N-20）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#mod_kth_root)

## API・条件

`mod_kth_roots(a,k,prime,max_baby_steps=2000000)` は素数法の全解を圧縮した `ModKthRoots` を返します。合成数法はinvalid_argument。uint64全域。非零解は `first*ratio^j`、0≤j<count、`zero` は0が解かを示します。解なしはcount=0かつzero=false。k=0では0を含めx^0=1と定義。`enumerate(max_roots=1000000)` は予算検査後に昇順列挙。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/mod_kth_root.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/mod_kth_root.hpp"

int main() {
    const auto roots = cp::mod_kth_roots(4, 4, 17);
    for (auto x : roots.enumerate()) std::cout << x << ' ';
    std::cout << '\n';
}
```

出力:

```text
6 7 10 11
```

## 原理・計算量

原始根gに関する離散対数を取り、k*t=log_g(a) mod(p−1)を解きます。Pollard–Rhoによる分解と原始根探索の後、BSGSは期待O(√p)時間/領域。列挙はK個に対してO(K log K)時間/O(K)領域。0やk=0ではBSGS不要。素数冪・合成数法のk乗根はこのAPIの対象外です。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

有限体でのべき乗の逆操作、全根の個数だけ必要な場合の圧縮表現。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
