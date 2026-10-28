---
title: Base Polyhedron・Lovász拡張
documentation_of: //combinatorial_optimization/base_polyhedron.hpp
---

# O-21 Base Polyhedron・Lovász拡張

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/base_polyhedron.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#base_polyhedron) / [一次資料](references.md)

重み順のprefixでFの増分を座標へ割り当てると、基多面体上の線形目的が最適になる。

## API・条件

Fの空集合値をoffsetとして分け、f(S)=F(S)−F(空)とします。扱う基多面体は B(f)={x : 全Sでx(S)≤f(S)、x(V)=f(V)}。Fは劣モジュラ、oracleは同じ集合に決定的なBigRationalへ変換できる値を返すことが前提です。単調性は不要で、負の限界値も扱います。

`base_polyhedron_greedy(weights,F,maximize=true)` は重みを降順に並べ、prefixの値の差を各座標へ置きます。maximize=falseでは昇順とし、同じ基多面体上の最小化を行います。同じ重みでは要素添字順です。

BasePolyhedronResultはbase（各要素の座標）、order（走査順）、chain_values（空集合を含む各prefixのF値）、offset、value（weights·base）、oracle_calls。baseの座標は元の要素順で返します。

`lovasz_extension(x,F)` はoffset+max_{s∈B(f)} x·sを返します。特にxが0/1の指示ベクトルならF(S)に一致します。劣モジュラ最小化ではこの凸な区分線形関数を[0,1]^n上で最小化します。一般の任意点の基多面体への所属判定APIや全頂点の列挙APIは含みません。

## 計算量

n要素でn+1回のoracle、O(n log n+nF)時間、O(n)領域。bit演算費用は別。

## 使用例

```cpp
#include "combinatorial_optimization/base_polyhedron.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<std::vector<int>> covers{{0, 1}, {1, 2}, {2, 3}};
    auto f = [&](const std::vector<int> &s) {
        std::vector<bool> seen(4);
        for (int e : s)
            for (int v : covers[e])
                seen[v] = true;
        int count = 0;
        for (bool x : seen)
            count += x;
        return BigRational(count);
    };
    auto r = base_polyhedron_greedy({3, 2, 1}, f);
    std::cout << r.value << "\n";
    for (std::size_t i = 0; i < r.base.size(); i++)
        std::cout << (i ? " " : "") << r.base[i];
    std::cout << "\n";
}
```

出力:

```text
9/1
2/1 1/1 1/1
```

## 使用する問題・検証

劣モジュラ関数の基の生成、Lovász拡張、凸最適化の線形oracle。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
