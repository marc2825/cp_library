---
title: Sparse Matrix
documentation_of: //linear_algebra/sparse_matrix.hpp
---

# Sparse Matrix

CSR形式で行列を保存し、行列・ベクトル積だけで計算するアルゴリズムへ接続します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#sparse_matrix) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`SparseMatrix<T>(rows,cols,vector<Entry>{row,column,value})`。`rows/cols/nonzeros/row_offsets/column_indices/values`、`multiply(x)`。`sparse_matrix_vector_multiply_mod<Mod>(SparseMatrix<int>,x)`。

重複座標と零は別項として保持、nonzerosは実際の格納数。添字範囲外out_of_range、ベクトル次元違反invalid_argument。generic版は標準整数の途中超過と浮動小数点の非有限値を検査。mod版は法1・合成数も可。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_algebra/sparse_matrix.hpp"

int main() {
    cp::SparseMatrix<int> a(2, 3, {{0, 1, 2}, {0, 1, 3}, {1, 2, -1}});
    auto y = a.multiply({7, 8, 9});
    std::cout << y[0] << " " << y[1] << "\n";
}
```

出力:

```text
40 -9
```

[コンパイルして動く例](../../examples/linear_algebra/sparse_matrix_b.cpp)。

## 計算量

構築O(rows+nnz)、積O(rows+cols+nnz)時間、保存O(rows+nnz)、積の出力・正規化O(rows+cols)。

## 使用する問題・検証範囲

疎グラフの状態遷移、Wiedemann、[Sparse Matrix Determinant](https://judge.yosupo.jp/problem/sparse_matrix_determinant)の疎な積部品。行列式APIをこの項目には含めません。

密行列の直接積、重複・負値・長方形・空・整数超過を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
