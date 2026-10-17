---
title: Garner
documentation_of: //number_theory_basic/garner.hpp
---

# Garner

任意個の互いに素な法のCRT解を、指定した法で取得します。

[実装](../../number_theory_basic/garner.hpp) / [操作できる図解](../../visualizations/math_a/index.html#garner) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`garner(residues,moduli,target_mod)` → long long。入力法の積未満の最小非負CRT解をtarget_modで割った剰余。

各入力法とtarget_modは正、入力長は一致し、入力法はpairwise coprime。target_modは入力法と共通因子があっても可。法1・負の剰余・空入力（0）に対応。不正条件はinvalid_argument。入力法の積は64bitに収まらなくてもよく、最小解の整数全体は構築しません。

## 原理と計算量

混合基数の桁を順に確定し、後続の法ごとの定数項と係数を更新します。最後にtarget_modを置きますが、その位置では逆元を求めません。このため出力法と入力法の互いに素性は不要です。

O(K^2)回のmod乗算とO(K)回の逆元、O(K)領域。大きなmod乗算は加算倍増にO(log M)。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "number_theory_basic/garner.hpp"

int main() {
    std::cout << cp::garner({2,3,2},{3,5,7},12) << "\n";
}
```

出力:

```text
11
```

[動く使用例](../../examples/number_theory_basic/garner.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

使用問題: 任意modのCRT復元。[既存の任意mod畳み込み](../convolution/convolution_mod.md)は固定3法の高速な内部処理を保持します。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [ACL CRTの契約](https://atcoder.github.io/ac-library/production/document_ja/math.html)、[既存の固定法復元](../../convolution/convolution_mod.hpp)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
