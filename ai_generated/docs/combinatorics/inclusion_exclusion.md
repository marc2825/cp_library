---
title: 包除原理
documentation_of: //combinatorics/inclusion_exclusion.hpp
---

# 包除原理

$$
|\bigcup_i A_i|=\sum_{S\ne\emptyset}(-1)^{|S|+1}|\bigcap_{i\in S}A_i|
$$

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/combinatorics/index.html#inclusion) / [実装](../../combinatorics/inclusion_exclusion.hpp) / [参考資料](references.md)

## usage

```cpp
#include <array>
#include <cstdint>
#include <iostream>
#include "combinatorics/inclusion_exclusion.hpp"
int main() {
    // 全体{0,1,2,3,4}。A={0,1,2}, B={1,2,3}。
    const std::array<unsigned, 2> sets{0b00111, 0b01110};
    const auto intersection = [&](std::uint64_t mask) {
        unsigned bits = 0b11111;
        for (int i = 0; i < 2; ++i) if (mask >> i & 1) bits &= sets[i];
        long long count = 0;
        for (; bits; bits &= bits - 1) ++count;
        return count;
    };
    std::cout << cp::inclusion_exclusion_union<long long>(2, intersection) << '\n';
    std::cout << cp::inclusion_exclusion_complement<long long>(2, intersection) << '\n';
}
```

[動く使用例](../../examples/combinatorics/inclusion_exclusion.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`inclusion_exclusion_union<T>(int sets, Intersection intersection) -> T`

`inclusion_exclusion_complement<T>(int sets, Intersection intersection) -> T`

- 集合iはmaskのiビット目。setsは0〜63、それ以外は `invalid_argument`。
- `intersection(uint64_t mask)` が選んだ集合の共通部分の大きさをTへ変換できる値で返します。
- unionは非空maskだけを昇順に呼びます。sets=0ではコールバックを呼ばずT(0)。
- complementはmask=0も呼び、これを全体集合の大きさとして扱います。和は `Σ(-1)^|S| intersection(S)`。sets=0では全体集合の大きさを返します。
- Tは0から構築でき、+=、-=を持つ型。標準整数（bool以外）は途中の範囲超過で `overflow_error`。独自のmod型等も使えます。

最終値が非負でも途中で負になる場合があります。符号付き整数かmod型を使ってください。符号なし整数も途中の負値・桁あふれを拒否します。コールバック内部の積や総数の範囲は利用者が管理します。

## 原理・計算量

各部分集合の共通部分を交互の符号で足します。同じ要素がt個の集合に属するとき、非空の選び方に対する係数和が1になるため、和集合で一度だけ数えられます。どの集合にも属さない数は、空集合の項を含めると係数が1、それ以外は0になります。

一般の部分集合列挙なので実行時間は指数的です。63集合はビット表現上の上限であり、現実的に全列挙できる規模を表すものではありません。対称性が使える場合は部分集合の大きさでまとめる方が速くなります。

コールバックの費用をFとしてO(2^sets*(F+加減算の費用))時間、追加O(1)空間。コールバック自身のコピーや保持するデータの費用は別です。

## 使用する問題・検証範囲

[使用する問題](https://cses.fi/problemset/task/2185)：Prime Multiples。選んだ素数の積の倍数を共通部分として数えます。

小さい部分集合・順序付きの選択・弱合成・括弧列・集合族を全列挙し、個数を独立に比較します。合成数法・法1・64ビットの法、0・不正な引数・前計算超過・加算前の範囲判定、旧APIも確認します。200万の階乗表・1500行のPascal表・22集合の包除も含みます。

[検証入口](../../verify/run_combinatorics.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、Python任意精度整数、6ジャッジ形式・384ケース。オンラインACは未確認です。
