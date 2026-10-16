---
title: 部分集合ゼータ変換
documentation_of: //set_function/subset_zeta_transform.hpp
---

# 部分集合ゼータ変換

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/set_function/index.html#subset-zeta) / [実装](../../set_function/subset_zeta_transform.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include <vector>
#include "set_function/subset_zeta_transform.hpp"
int main() {
    std::vector<long long> f{1, 2, 3, 4};
    cp::subset_zeta_transform(f);
    for (auto x : f) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/set_function/subset_zeta_transform.cpp)。ライブラリのルートをincludeパスに追加します。

## API・使用条件

`subset_zeta_transform(vector<T>& f) -> void`

f[S] = Σ_(T⊆S) 元のf[T] をin-placeで計算します。入力の長さNは**正の2冪**で、空や非2冪はinvalid_argument、変更前に拒否します。長さ1は0ビットの集合関数で、値を変更しません。空集合も全体集合も配列の要素に含みます。

Tは可換で結合的な加算+=を持つ型です。自分自身も集計に含みます。

標準整数（bool以外）は全中間値の加減算を検査し、範囲超過はoverflow_errorです。符号なしでも折り返さず、負になる減算は拒否します。失敗前に終えた更新は残ります。独自mod型や複素数などはその型の算術条件を利用者が管理します。boolは未対応です。

C++17・namespace cp。[共通仕様と旧API](README.md#共通仕様)も参照してください。

## 原理・計算量

各ビットについて、他のビットが同じ集合の組を作り、ビット0から1の集合へ値を足します。処理済みのビットだけ異なる集合からの寄与が集まっている状態を保ち、最後に全対象集合の和を得ます。

N=2^nとしてO(nN)時間、追加空間O(1)。加算／減算は正確にnN/2回です。型の算術演算費用を掛けます。整数・mod型では各演算O(1)です。

## 使用する問題・検証範囲

[Library Checker Bitwise AND Convolution](https://judge.yosupo.jp/problem/bitwise_and_convolution)。[部分集合変換のラッパ](../../verify/set_function/subset_transforms.test.cpp)は入出力の添字を補集合へ反転して使用します。 畳み込みは検証用の使用例で、このカテゴリの公開APIへの追加ではありません。

[検証入口](../../verify/run_set_function.py)は9ヘッダの単独／逆順／混在includeと複数翻訳単位、7使用例、4ジャッジ形式・256ケースを確認します。2,047基準mask・177,146組、6,654三値表・180ランダム表、Python任意精度2,428クエリを、定義から直接集計した和・交互和・集約と比較します。空集合・0ビット・63/64ビット、法12の型、複素数、単位元不要の型、262,144個のint8演算、符号付き／符号なし極値、途中超過と失敗前の状態、2^20項の全出力と正確な更新回数を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行し、移動した3.12 DP APIも回帰検証します。図解は2,853入力・50,838スナップショット、15拒否条件とブラウザ操作を確認します。オンラインACは未確認です。
