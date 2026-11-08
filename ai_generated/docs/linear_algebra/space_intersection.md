---
title: 線形空間の交差
documentation_of: //linear_algebra/space_intersection.hpp
---

# 線形空間の交差

行が生成する二つの線形空間の共通部分の基底を求めます。GF(2)のXOR基底にも対応します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#intersection) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`linear_space_intersection<T>(const Matrix<T>& U, const Matrix<T>& V) -> Matrix<T>`。各行が生成ベクトル、列数dがambient dimension。重複・従属・0行も可。結果の各行は交差空間の独立な基底で、順序や値の標準形は保証しません。空の空間も0×dで形を保持します。列数不一致はinvalid_argument。

体型専用で通常の整数除算は拒否。実数版は第3引数epsilon（既定1e-10、正・有限）と既存Gauss–Jordanの数値的な階数判定を使い、悪条件で厳密な交差を保証しません。`linear_space_intersection_mod<Mod=998244353>(U,V)`は素数法のint入出力。入力不変です。

`xor_space_intersection<UInt>(const XorBasis<UInt>& U, const XorBasis<UInt>& V) -> XorBasis<UInt>`は符号なし整数のGF(2)空間の交差。64bit全域を使えます。

## Usage

```cpp
#include <iostream>
#include "linear_algebra/space_intersection.hpp"
int main() {
    const cp::Matrix<int> u{{1, 0, 0}, {0, 1, 0}}, v{{0, 1, 0}, {0, 0, 1}};
    const auto common = cp::linear_space_intersection_mod(u, v);
    std::cout << common.rows() << '\n';
    for (auto x : common.data()) std::cout << x << ' ';
    std::cout << '\n';
    cp::XorBasis<> a, b; a.insert(3); a.insert(5); b.insert(6);
    std::cout << cp::xor_space_intersection(a, b).max_xor() << '\n';
}
```

出力:

```text
1
0 1 0
6
```

[コンパイルできる使用例](../../examples/linear_algebra/space_intersection.cpp)。

## 原理

まず両方の行を独立な基底へ減らします。[Uᵀ|-Vᵀ]の零空間のベクトル(α,β)に対し、αU=βVが共通ベクトルです。独立な基底からの係数写像は単射なので、零空間基底の像も独立です。XOR版は消去中にU側の成分だけを追跡し、V側の依存関係が0へ消えたときのU成分を採用します。

## 計算量

p行・q行・d列でO((p+q)d²+d³)時間、O((p+q)d+d²)領域。XOR版はwビット、両階数の和rとしてO(wr)時間・O(w)領域。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

二つの生成系で同時に表せるベクトルや共有できるXOR値を求める問題。専用の外部ジャッジへ提出していないため、ローカルでは小体の全ベクトル列挙との一致・独立性・次元を確認します。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
