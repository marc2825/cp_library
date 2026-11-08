---
title: Low-Rank Update
documentation_of: //linear_algebra/low_rank_update.hpp
---

# Low-Rank Update

既知の逆行列A^-1から、A+U V^Tの逆と行列式の比を小さい核行列で求めます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#low_rank_update) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`low_rank_inverse_update_mod<Mod>(inverse,U,V)`→optional<Matrix<int>>。`low_rank_determinant_ratio_mod<Mod>(inverse,U,V)`→int。素数法、inverseはn×n、U,Vは同じn×k。

inverseが実際のA^-1であることは利用者の前提。I+V^T A^-1 Uを作りWoodbury式を適用。更新後が特異なら逆はnullopt、比は0。k=0は変更なしで比1、n=0にも対応。次元違反invalid_argument。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_algebra/low_rank_update.hpp"

int main() {
    auto inverse = cp::Matrix<int>::identity(2);
    cp::Matrix<int> u{{1}, {0}}, v{{2}, {0}};
    std::cout << cp::low_rank_determinant_ratio_mod(inverse, u, v) << "\n";
}
```

出力:

```text
3
```

[コンパイルして動く例](../../examples/linear_algebra/low_rank_update_b.cpp)。

## 計算量

逆更新O(n²k+nk²+k³+n²)時間、O(n²+nk+k²)領域。行列式比はO(n²k+nk²+k³)、保存O(nk+k²)。

## 使用する問題・検証範囲

動的な線形方程式・行列式、1本の辺追加によるLaplacian小行列の更新。

更新した密行列を直接逆行列化し、置換和の行列式とWoodburyの値を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
