---
title: Primitive Root
documentation_of: //number_theory_basic/primitive_root.hpp
---

# Primitive Root

素数を法とした乗法群の最小の正の原始根を求めます。

[実装](../../number_theory_basic/primitive_root.hpp) / [操作できる図解](../../visualizations/math_a/index.html#primitive_root) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`primitive_root(uint64_t p)` → uint64_t。p=2では1。それ以外の素数では2以上の最小の候補を返します。

pは素数。既存Miller–Rabinで検査し、合成数はinvalid_argument。p-1を既存Pollard–Rhoで分解します。uint64_t全域が対象ですが、探索時間の最悪保証はありません。合成数法の単元群を生成する入口は対象外です。

## 原理と計算量

p-1の異なる各素因数qについてg^((p-1)/q)!=1を検査します。全部通る候補だけが位数p-1を持つため、昇順に試した最初が最小の原始根です。

p-1の素因数分解＋G個の候補、異なる素因数個数wについてO(G*w*log p)回の安全なmod乗算。mod乗算自体は最大O(log p)。追加領域は因数分解に従います。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "number_theory_basic/primitive_root.hpp"

int main() {
    std::cout << cp::primitive_root(17) << "\n";
}
```

出力:

```text
3
```

[動く使用例](../../examples/number_theory_basic/primitive_root.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [Library Checker Primitive Root](https://judge.yosupo.jp/problem/primitive_root)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [既存Miller–Rabin](../number_theory_convolution/miller_rabin.md)、[原始根の定義と判定](https://exploringnumbertheory.wordpress.com/2013/09/03/defining-primitive-root/)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
