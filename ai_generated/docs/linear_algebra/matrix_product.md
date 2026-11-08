---
title: 行列積・行列と列ベクトルの積
documentation_of: //linear_algebra/matrix_product.hpp
---

# 行列積・行列と列ベクトルの積

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_algebra/index.html#product) / [実装](../../linear_algebra/matrix_product.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "linear_algebra/matrix_product.hpp"
int main() {
    cp::Matrix<long long> a{{1, 2, 3}, {4, 5, 6}}, b{{7, 8}, {9, 10}, {11, 12}};
    auto c = a * b;
    for (std::size_t i = 0; i < c.rows(); ++i) {
        for (std::size_t j = 0; j < c.cols(); ++j) std::cout << c(i, j) << ' ';
        std::cout << '\n';
    }
    auto y = cp::matrix_vector_multiply(a, std::vector<long long>{1, 0, -1});
    for (auto x : y) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/linear_algebra/matrix_product.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cpです。

## API・使用条件

`matrix_multiply(const Matrix<T>& A,const Matrix<T>& B) -> Matrix<T>`、同じ処理を`A*B`でも呼べます。Aがn×k、Bがk×mのときn×mを返し、入力は変更しません。内側の次元が違えばinvalid_argument。n×0と0×mの積はn×mの零行列です。

`matrix_vector_multiply(const Matrix<T>& A,const vector<T>& x) -> vector<T>` は列ベクトルのA*xです。x.size()==A.cols()を要求し、違反はinvalid_argument。A.rows()個の結果を返します。

T(0)、T同士の*、+=が必要です。加法は可換・結合的、乗法は結合的で加法に分配し、0が積の零元であること。乗法の可換性は要求しません。順序はA(i,k)*B(k,j)。独自mod型なら合成数法・法1も使えます。bool未対応。

bool以外の標準整数は各積と各部分和がTの範囲内かを検査し、超過をoverflow_errorで拒否します。打ち消した後の最終値だけが収まる場合も拒否され得ます。標準浮動小数点は入力NaN/Infをinvalid_argument、非有限の途中結果をoverflow_errorにします。有限の丸め・桁落ち・アンダーフローは検出しません。複素数等の独自型の算術条件は利用者が管理します。例外でも入力は変わりません。

## 原理・計算量

C(i,j)=Σ_(t=0..k−1)A(i,t)B(t,j)を計算します。i→t→jの順に走査し、各要素の和はtの昇順です。積と加算はそれぞれ正確にnkm回。時間O(nkm+nk+km+nm)（入力検査・零初期化を含む）、追加領域O(nm)。ベクトル積はO(nk+n+k)時間、O(n)領域です。型の演算費用を掛けます。

行列積の順序を入れ替えると結果が変わり、形によっては逆の積を定義できません。

## 使用する問題・検証範囲

[Library Checker Matrix Product](https://judge.yosupo.jp/problem/matrix_product)の[ラッパ](../../verify/linear_algebra/matrix_product.test.cpp)でmod998244353の積を使用します。独立な行と列の内積で全出力を比較します。

[検証入口](../../verify/run_linear_algebra.py)は8ヘッダ、5使用例、4ジャッジ形式・256ケース、Pythonの独立検証1,361クエリを確認します。500長方形積、反復積との1,950比較、有限体5,915方程式の全解、実数160系、有理数・独自mod型・複素数、65,536個のint8積、37,449短いXOR系列・100個の64ビット系列、128変数の消去、32×32の巡回行列、100,000回の挿入を含みます。通常・ASan/UBSan/LeakSanitizer、debug/release、旧行列と3.18との混在includeも検証します。オンラインACは未確認です。

図解は5モード。2,413入力・27,172スナップショットと17拒否条件を、内積の定義・反復積・行の同値性・解の全列挙・XORの部分集合から独立に確認します。ブラウザ操作・再生・入力拒否・描画範囲・390px表示も検証します。
