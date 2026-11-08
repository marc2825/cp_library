---
title: 連立一次方程式（Gauss–Jordan）
documentation_of: //linear_algebra/gauss_jordan.hpp
---

# 連立一次方程式（Gauss–Jordan）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_algebra/index.html#gauss) / [実装](../../linear_algebra/gauss_jordan.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "linear_algebra/gauss_jordan.hpp"
int main() {
    cp::Matrix<double> a{{1, 1}, {2, -1}};
    auto real = cp::gauss_jordan(a, std::vector<double>{3, 0});
    for (auto x : real.particular) std::cout << x << ' ';
    std::cout << '\n';
    auto mod = cp::gauss_jordan_mod<7>(cp::Matrix<int>{{1, 2, 3}, {2, 4, 6}}, {4, 1});
    std::cout << mod.rank << ' ' << mod.nullspace_basis.size() << '\n';
    for (auto x : mod.particular) std::cout << x << ' ';
    std::cout << '\n';
    for (const auto& v : mod.nullspace_basis) {
        for (auto x : v) std::cout << x << ' ';
        std::cout << '\n';
    }
}
```

[動く使用例](../../examples/linear_algebra/gauss_jordan.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cpです。

## API・使用条件

`gauss_jordan(const Matrix<T>& A,const vector<T>& b) -> LinearSystemResult<T>` は厳密な体の型向けです。T(0),T(1),+=,-=,*,/,==が必要。非零元で除算できる可換体を要求します。有理数・素数法のmod型等を渡せます。標準整数の割り算は体にならないためコンパイル時に拒否します。

標準浮動小数点は`gauss_jordan(A,b,T epsilon=T(1e-10))`です。epsilonは正の有限値。列内の絶対値が最大の候補をpivotにし、絶対値≤epsilonを0とみなします。入力NaN/Infはinvalid_argument、非有限の途中値はoverflow_error。epsilonは絶対誤差で、スケール変更・悪条件の行列・丸めにより階数や整合性の判定が変わり得ます。特解と基底の残差を元のA,bで確認できるよう、入力を変更しません。

`gauss_jordan_mod<Mod=998244353>(const Matrix<int>& A,const vector<int>& b) -> LinearSystemResult<int>` は素数法2〜2×10^9専用です。負の係数やINT_MIN/MAXも正規化します。不適切な法はコンパイル時に拒否。合成数法の一般の連立合同式には使えません。内部のmod演算は3.18の整数算術補助を再利用します。

どの版もAがn×mならb.size()==nを要求し、違反はinvalid_argument。結果のフィールドは次のとおりです。

| フィールド | 意味 |
|---|---|
| status | LinearSystemStatus::no_solution / unique / multiple |
| has_solution() | no_solution以外ならtrue |
| rank | 係数行列Aの階数（実数ではepsilonで判定） |
| pivot_columns | pivotに使った変数の列、昇順 |
| reduced | n×(m+1)の拡大行列。係数側のpivot列を1/0にした簡約結果 |
| particular | 自由変数を0にした長さmの特解 |
| nullspace_basis | 自由列の昇順に、該当自由変数を1とした長さmのベクトル |

解があればx=particular+Σ t[j]*nullspace_basis[j]です。uniqueは基底が空。multipleは自由変数あり（有限体では解の個数も有限）。no_solutionではparticularとnullspace_basisは空で、rank/pivot_columns/reducedは返します。矛盾する右辺の列自体をpivotにして完全な拡大行列のRREFを作るAPIではありません。

0×mと空のbは特解が零ベクトル、零空間が標準基底の全空間。n×0はbが全0なら空ベクトルの一意解、そうでなければ解なしです。

## 原理・計算量

列ごとにpivotを選び、行交換→pivotを1へ正規化→全ての別の行からpivot行の倍数を引きます。自由列を飛ばし、最後の残りの行の右辺に0以外があれば矛盾です。実数では小さな消去係数も省略せず、大きなpivot行との積の寄与を残します。

r=rankとして、体の演算回数はO(n(m+1)(r+1)+m(m−r))、領域O(n(m+1)+m(m−r)+m)。解空間の基底を明示的に返すため、方程式が0本でもO(m²)の出力が必要です。独自型の除算費用は別。素数法版はpivotごとに逆元を1回計算し、O(n(m+1)(r+1)+m(m−r)+r log Mod)時間です。

## 使用する問題・検証範囲

[Library Checker System of Linear Equations](https://judge.yosupo.jp/problem/system_of_linear_equations)の[ラッパ](../../verify/linear_algebra/gauss_jordan.test.cpp)で特解と全零空間基底を返します。有限体の解を全列挙し、生成される解の集合全体を比較。別のPython検証では全小行列式の順列和から階数と整合性を判定し、各出力の残差・独立性を確認します。

[検証入口](../../verify/run_linear_algebra.py)は8ヘッダ、5使用例、4ジャッジ形式・256ケース、Pythonの独立検証1,361クエリを確認します。500長方形積、反復積との1,950比較、有限体5,915方程式の全解、実数160系、有理数・独自mod型・複素数、65,536個のint8積、37,449短いXOR系列・100個の64ビット系列、128変数の消去、32×32の巡回行列、100,000回の挿入を含みます。通常・ASan/UBSan/LeakSanitizer、debug/release、旧行列と3.18との混在includeも検証します。オンラインACは未確認です。

図解は5モード。2,413入力・27,172スナップショットと17拒否条件を、内積の定義・反復積・行の同値性・解の全列挙・XORの部分集合から独立に確認します。ブラウザ操作・再生・入力拒否・描画範囲・390px表示も検証します。
