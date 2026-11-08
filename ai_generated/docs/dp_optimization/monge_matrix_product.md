---
documentation_of: //dp_optimization/monge_matrix_product.hpp
---

# 長方形Monge行列のmin-plus積

[P-20の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#monge_matrix_product) / [ヘッダ](../../dp_optimization/monge_matrix_product.hpp)

`monge_min_plus_product(rows,inner,cols,a,b)` はcallback行列の`C[i][j]=min_k(a(i,k)+b(k,j))`を返します。Aは任意の有限long long、Bは有限long longのMonge行列で、隣接する2×2について`B[k][j]+B[k+1][j+1] <= B[k][j+1]+B[k+1][j]`を満たすことが必要です。Monge条件は自動検査しません。

返値の`value[i][j]`はoptional値、`argmin[i][j]`は最小の中間添字k。同点は最左k。inner=0なら全valueがnullopt、argmin=-1です。rows=0/cols=0も明示した形を保持します。callbackは副作用なし・ランダムアクセスO(1)を想定。計算する和のlong long超過はoverflow_error。

Aの各行を固定すると、Bの転置の各列へAの値を加えたMonge行列になります。その行argminを既存SMAWKで一括計算します。時間O(rows*(inner+cols))、結果領域O(rows*cols)、追加作業領域O(inner+cols)。密な正方行列ならO(n²)。入力/出力を暗黙表現するunit-Mongeの圧縮積は対象に含めません。

用途: Monge遷移の複数初期値をまとめたDP、二層の最短遷移の合成。600長方形行列を三重ループと比較し、同点・空次元・callback評価回数を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/monge_matrix_product.hpp"
int main() {
    std::vector<std::vector<long long>> a{{0,4,2},{3,0,5}};
    auto result = cp::monge_min_plus_product(2,3,3,
        [&](int i,int k) { return a[i][k]; }, [](int k,int j) { return 1LL*(k-j)*(k-j); });
    for (const auto& row : result.value) {
        for (std::size_t j = 0; j < row.size(); ++j) std::cout << (j ? " " : "") << *row[j];
        std::cout << '\n';
    }
}
```

出力:

```text
0 1 2
1 0 1
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
