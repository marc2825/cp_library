---
title: XOR基底
documentation_of: //linear_algebra/xor_basis.hpp
---

# XOR基底

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/linear_algebra/index.html#xor) / [実装](../../linear_algebra/xor_basis.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "linear_algebra/xor_basis.hpp"
int main() {
    cp::XorBasis<> basis;
    basis.insert(3); basis.insert(5);
    std::cout << basis.insert(6) << ' ' << basis.rank() << '\n';
    std::cout << basis.contains(6) << ' ' << basis.contains(1) << '\n';
    std::cout << basis.min_xor(1) << ' ' << basis.max_xor() << '\n';
}
```

[動く使用例](../../examples/linear_algebra/xor_basis.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cpです。

## API・使用条件

`XorBasis<UInt=uint64_t>` は符号なし標準整数のビットベクトルをGF(2)の線形空間として保持します。boolは未対応。符号付きの負数を符号なし引数へ渡さないでください。既定では最上位bit63を含む64ビット全域を扱います。

| API | 意味 |
|---|---|
| insert(value) -> bool | 独立なら挿入してtrue。0や既存の空間内ならfalseで基底不変 |
| rank() -> int | 張る空間の次元。最大64 |
| empty() / clear() | 基底が空か／初期状態へ戻す |
| contains(value) -> bool | 空のXORも許し、valueが空間に含まれるか |
| min_xor(seed=0) / max_xor(seed=0) -> UInt | seed XOR spanの値を符号なし数値順で最小／最大化 |
| merge(const XorBasis& other) -> int | 両空間の和を保持し、増えた次元を返す。otherは不変、自分自身とのmergeも可 |
| basis() -> vector<UInt> | 最上位の1の位置が大きい順の独立ベクトル |

空の基底も0を表せます。空ではmin/maxともseed自身、contains(0)はtrue。basisの値は挿入順に依存し、全pivot列が他の行で0になる標準形にはしていません。入力から何個選ぶか・個数制限・選んだ要素の復元は保持しません。

表せる異なる値は2^rank個です。rank=64のとき2^64はuint64_tに収まらないため、1ULL<<rankで個数を計算しないでください。API内では64ビットのシフトを行いません。

## 原理・計算量

最上位の1がある位置bitに既存pivotがあればXORして消し、なければその位置へ挿入します。問い合わせも大きいbitから、値が小さく／大きくなる候補を採用します。下位のpivotでは確定した上位bitを変更できません。containsはmin_xor(value)==0と同じです。

B=UIntのビット数として、挿入・所属判定・最小／最大・clear・basisはO(B)、mergeはO(B²)、rank/emptyはO(1)。内部空間はO(B)、basisの戻り値はO(rank)。

## 使用する問題・検証範囲

[AtCoder ABC141 F: Xor Sum 3](https://atcoder.jp/contests/abc141/tasks/abc141_f)の[ラッパ](../../verify/linear_algebra/xor_basis.test.cpp)で使用します。全体のXORをtotalとし、totalが0のビットだけを基底に入れると、答えはtotal+2*max_xor()になります。両側を空にしない分割を全列挙して比較します。

[検証入口](../../verify/run_linear_algebra.py)は8ヘッダ、5使用例、4ジャッジ形式・256ケース、Pythonの独立検証1,361クエリを確認します。500長方形積、反復積との1,950比較、有限体5,915方程式の全解、実数160系、有理数・独自mod型・複素数、65,536個のint8積、37,449短いXOR系列・100個の64ビット系列、128変数の消去、32×32の巡回行列、100,000回の挿入を含みます。通常・ASan/UBSan/LeakSanitizer、debug/release、旧行列と3.18との混在includeも検証します。オンラインACは未確認です。

図解は5モード。2,413入力・27,172スナップショットと17拒否条件を、内積の定義・反復積・行の同値性・解の全列挙・XORの部分集合から独立に確認します。ブラウザ操作・再生・入力拒否・描画範囲・390px表示も検証します。
