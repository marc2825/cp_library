---
title: Hook-Length / Hook-Content
documentation_of: //combinatorics/hook_length.hpp
---

# Hook-Length / Hook-Content（C-22）

[ヘッダ](../../combinatorics/hook_length.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#hook_length)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/hook_length.hpp"
int main() {
    const cp::YoungShape shape{3, 2};
    std::cout << cp::standard_young_tableaux_count(shape) << ' '
              << cp::standard_young_tableaux_count_mod(shape, 4) << ' '
              << cp::semistandard_young_tableaux_count(shape, 3) << '\n';
}
```

出力:

```text
5 1 15
```

## API・条件

`young_hook_lengths(shape,max_cells=1000000)` は各セルの右+下+自身のhook長を返します。`standard_young_tableaux_count(shape,max_cells)` は標準盤数をBigIntで、`standard_young_tableaux_count_mod(shape,uint64 modulus,max_cells)` は任意の正の法で返します。

`semistandard_young_tableaux_count(shape,uint64 alphabet,max_cells)` はラベル1..alphabetの半標準盤数をBigIntで返します。空shapeは1、alphabetが行数より小さい非空shapeは0。通常の直線Young図形が対象で、skew図形は対象外です。

不正shape/法0は `invalid_argument`、セル予算超過・篩のint上限超過は `length_error`。合成数法でもhookの逆元を要求しません。法1は0。BigIntのためBoostヘッダが必要です。

## 仕組み

標準盤数は `N! / product_cells h(r,c)`。N!と各hook長をSPFで素因数分解し、指数を差し引いてからべき乗を掛けます。modの世界で不正な除算をせず、偶数や合成数の法に対応します。

半標準盤数はhook-content式 `product_cells (alphabet+c−r) / h(r,c)`（r,cは0始まり）。この版は分子/分母を多倍長整数で積み、最後に厳密除算します。巨大alphabetも固定幅で加算しません。

## 計算量

hook一覧O(N)時間・空間。標準盤数の素因数指数構築はO(N log N)を上界とする整数演算・O(N)空間、出力は素数べき乗の費用を加算。mod版は固定幅で各素数O(log N)回の安全な乗算。半標準版はO(N)回の多倍長積と1回の除算で、桁数の費用を含める必要があります。

## 使用する問題

Young図形に沿った順序付き充填、対称群の既約表現の次元、指定アルファベットの半標準盤数。

## 検証

全標準盤列挙と照合し、各nでsum_shape(f_shape²)=n!を確認。小図形の半標準盤を直接充填してhook-contentと比較。法1/2/4/7/12/uint64最大、巨大alphabetを検証。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
