---
title: Matrix（長方形行列）
documentation_of: //linear_algebra/matrix.hpp
---

# Matrix（長方形行列）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_algebra/index.html#matrix) / [実装](../../linear_algebra/matrix.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "linear_algebra/matrix.hpp"
int main() {
    cp::Matrix<int> a{{1, 2, 3}, {4, 5, 6}};
    auto t = a.transposed();
    std::cout << a.rows() << ' ' << a.cols() << '\n';
    std::cout << t(2, 1) << ' ' << cp::Matrix<int>::identity(2)(1, 1) << '\n';
}
```

[動く使用例](../../examples/linear_algebra/matrix.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cpです。

## API・使用条件

`Matrix<T>()` は0×0、`Matrix<T>(rows,cols,value=T(0))` は指定した形を同じ値で埋めます。`Matrix<T>{{…},{…}}`、明示的なvector<vector<T>>からも構築できます。行の長さが違えばinvalid_argument。空のvectorからは0×0になり、0×mが必要なら次元を指定してください。

`rows()` / `cols()` はsize_t、`data()` は行優先のconst vector<T>&です。`a(row,col)` は参照を返し、有効な添字であることが前提（assert）。`at(row,col)` は違反をout_of_rangeで拒否します。行数×列数がvectorの最大容量を超えればlength_error。メモリ確保失敗は通常のbad_alloc等です。負の次元や添字を符号なし引数へ渡さないでください。

`Matrix<T>::identity(n)` はn×n単位行列、`transposed()` は行と列を入れ替えた新しい行列です。`==` / `!=` は次元と全要素を比較します。コピーは独立した値。move後の元は0×0となり、自分自身へのmove代入も扱います。要素数0でも0×mとn×0を区別します。

Tはコピー・代入とT(0)が必要です。identityはT(1)、等値比較はTの==を使います。boolは未対応です。行列を保持するだけでは係数の有限性・数値範囲を検査しません。演算時の条件は各APIを参照してください。

## 原理・計算量

構築・コピー・転置はO(nm)時間・O(nm)空間、identityはO(n²)。次元・要素アクセスはO(1)です。空の行列の構築・転置は要素を走査しません。添字(r,c)の格納位置はr*cols+cです。

## 使用する問題・検証範囲

[Library Checker Matrix Product](https://judge.yosupo.jp/problem/matrix_product)の長方形データの保持に使います。API単独の例では形・転置・単位行列を確認し、積の[ラッパ](../../verify/linear_algebra/matrix_product.test.cpp)でも使用します。

[検証入口](../../verify/run_linear_algebra.py)は8ヘッダ、5使用例、4ジャッジ形式・256ケース、Pythonの独立検証1,361クエリを確認します。500長方形積、反復積との1,950比較、有限体5,915方程式の全解、実数160系、有理数・独自mod型・複素数、65,536個のint8積、37,449短いXOR系列・100個の64ビット系列、128変数の消去、32×32の巡回行列、100,000回の挿入を含みます。通常・ASan/UBSan/LeakSanitizer、debug/release、旧行列と3.18との混在includeも検証します。オンラインACは未確認です。

図解は5モード。2,413入力・27,172スナップショットと17拒否条件を、内積の定義・反復積・行の同値性・解の全列挙・XORの部分集合から独立に確認します。ブラウザ操作・再生・入力拒否・描画範囲・390px表示も検証します。
