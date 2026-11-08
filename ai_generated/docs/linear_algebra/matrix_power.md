---
title: 行列累乗
documentation_of: //linear_algebra/matrix_power.hpp
---

# 行列累乗

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_algebra/index.html#power) / [実装](../../linear_algebra/matrix_power.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "linear_algebra/matrix_power.hpp"
int main() {
    cp::Matrix<long long> a{{1, 1}, {1, 0}};
    auto power = cp::matrix_power(a, 10);
    std::cout << power(0, 0) << ' ' << power(0, 1) << '\n';
    std::cout << cp::matrix_power(a, 0)(1, 1) << '\n';
}
```

[動く使用例](../../examples/linear_algebra/matrix_power.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cpです。

## API・使用条件

`matrix_power(Matrix<T> A,uint64_t exponent) -> Matrix<T>` は正方行列Aの累乗を返します。入力は値としてコピーし、呼出し元の行列は変わりません。非正方は指数0でもinvalid_argument。0×0も扱い、指数0は同じ次元の単位行列です。

指数は0〜2^64−1です。負数を符号なし引数に渡さないでください。Matrixの積の条件に加え、T(1)が乗法の単位元となる必要があります。mod型なら合成数法・法1も使えます。各積・部分和の整数超過と標準浮動小数点の非有限値を検査します。指数0でも全入力の有限性を検査します。

## 原理・計算量

答えをIから始め、下位ビットから1なら現在のA^(2^bit)を右へ掛けます。次のビットへ進むときだけ基底行列を二乗します。最後の不要な二乗をしないため、1×1のLLONG_MAXの1乗もそのまま返せます。

n≥1でO(n³ log(exponent+1)+n²)時間、O(n²)領域。指数0はO(n²)。浮動小数点の行列積は厳密な結合則を満たさず、反復積と丸め結果が異なることがあります。

3.18の[matrix_power_mod / matrix_power_apply](../linear_recurrence/matrix_recurrence.md)はvector<vector<int>>を受け取る既存APIです。新しいMatrix<T>のAPIとは名前・型を分け、旧行列のグローバル関数はリポジトリ直下の `depricated/matrix/matrix_basic.hpp` に保存しています。

## 使用する問題・検証範囲

[Library Checker Pow of Matrix](https://judge.yosupo.jp/problem/pow_of_matrix)の[ラッパ](../../verify/linear_algebra/matrix_power.test.cpp)で使用します。小さい指数の反復積と、巨大指数の巡回置換を直接の添字計算で比較します。

[検証入口](../../verify/run_linear_algebra.py)は8ヘッダ、5使用例、4ジャッジ形式・256ケース、Pythonの独立検証1,361クエリを確認します。500長方形積、反復積との1,950比較、有限体5,915方程式の全解、実数160系、有理数・独自mod型・複素数、65,536個のint8積、37,449短いXOR系列・100個の64ビット系列、128変数の消去、32×32の巡回行列、100,000回の挿入を含みます。通常・ASan/UBSan/LeakSanitizer、debug/release、旧行列と3.18との混在includeも検証します。オンラインACは未確認です。

図解は5モード。2,413入力・27,172スナップショットと17拒否条件を、内積の定義・反復積・行の同値性・解の全列挙・XORの部分集合から独立に確認します。ブラウザ操作・再生・入力拒否・描画範囲・390px表示も検証します。
