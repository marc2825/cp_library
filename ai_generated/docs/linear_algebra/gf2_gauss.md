---
title: GF(2)上のBitset Gauss
documentation_of: //linear_algebra/gf2_gauss.hpp
---

# GF(2)上のBitset Gauss

std::bitsetのXORで複数列をまとめて消去し、二値方程式の解空間を返します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#gf2) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`gf2_gauss_jordan<Bits>(const vector<bitset<Bits>>& A, const vector<int>& b, size_t variables=Bits)`。bit jが変数jの係数です。variables<=Bits、b.size()==A.size()、bは0/1、variables以上の入力bitは0を要求し、違反はinvalid_argument。Bitsはコンパイル時の容量で、実際の変数数を別に指定できます。

結果GF2LinearSystemResult<Bits>はstatus（既存LinearSystemStatus）、rank、pivot_columns、reduced、particular、nullspace_basis、has_solution()を持ちます。reducedはbitset<Bits+1>の配列で、RHSの位置はvariables。特解と基底はbitset<Bits>です。解がない場合は特解・基底を使用しません。入力不変。0方程式は全変数が自由、0変数はRHSが全0なら一意解です。

`gf2_rank<Bits>(A, variables=Bits)`は係数行列の階数だけを求め、零空間の全基底を作りません。

## Usage

```cpp
#include <iostream>
#include "linear_algebra/gf2_gauss.hpp"
int main() {
    const std::vector<std::bitset<8>> a{std::bitset<8>(3), std::bitset<8>(6)};
    const auto r = cp::gf2_gauss_jordan(a, std::vector<int>{1, 0}, 3);
    std::cout << r.rank << ' ' << r.nullspace_basis.size() << '\n';
    for (std::size_t j = 0; j < 3; ++j) std::cout << r.particular[j];
    std::cout << '\n';
}
```

出力:

```text
2 1
100
```

[コンパイルできる使用例](../../examples/linear_algebra/gf2_gauss.cpp)。

## 原理

pivot列を昇順で選び、pivotを持つ他の行へ行全体をXORします。GF(2)では非0が1なので除算は不要です。自由列ごとにその変数を1にして、pivot列を消去行から復元すると零空間の基底を得ます。

## 計算量

m行・v変数・階数r・W=ceil((Bits+1)/word)として、入力検査O(m Bits)、pivot探索O(mv)、消去O(mrW)。全基底の構築O((v-r)(W+r))を加えます。領域O((m+v-r)W)。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

[Library Checker System of Linear Equations Mod 2](https://judge.yosupo.jp/problem/system_of_linear_equations_mod_2)。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
