---
documentation_of: //linear_algebra/hermite_normal_form.hpp
---

# Hermite標準形（L-07）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/hermite_normal_form.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#hermite)

## API・条件

`hermite_normal_form(a) → {form,left,pivot_columns}`。

行側の規約H=UA。pivot列は狭義増加、pivotは正、pivotより下は0・上は0以上pivot未満。零行を末尾に残す。Uはunimodular、長方形と0次元も対応。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

列を左から見て整数Euclid法でpivotを作り、正符号に揃える。上側はfloor除算で標準剰余へ減らす。

Euclid基本操作数EとしてO(E(m+n))回の多倍長整数演算、O(mn+m²)個の整数。途中値のビット長と操作数は入力依存。

ここでEには行基本変形とpivotの商・剰余計算の回数を含めます。行列の変換・初期化・pivot候補の探索に別途O(mn+m²)が必要です。

## 使用例

[実行するソース](../../examples/linear_algebra/hermite_normal_form.cpp)。

```cpp
#include "linear_algebra/hermite_normal_form.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto h = cp::hermite_normal_form(cp::Matrix<int>{{4, 6}, {2, 8}});
    print(h.form.data());
}
```

出力:

```text
2 8 0 10
```

## 使用する問題・検証

整数行格子の標準化と同一性判定。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
