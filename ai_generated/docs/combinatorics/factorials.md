---
title: 階乗・逆階乗
documentation_of: //combinatorics/factorials.hpp
---

# 階乗・逆階乗

$$
f[n]=n!\bmod m,\qquad g[n]=(n!)^{-1}\bmod m
$$

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/combinatorics/index.html#factorial) / [実装](../../combinatorics/factorials.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "combinatorics/factorials.hpp"
int main() {
    cp::FactorialTable table(6, 1000000007);
    std::cout << table.factorial(5) << '\n';
    std::cout << table.inverse_factorial(5) << '\n';
    std::cout << cp::factorials(6, 12)[6] << '\n';
}
```

[動く使用例](../../examples/combinatorics/factorials.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`factorials(int max_n, long long modulus=998244353) -> vector<long long>`

0!〜max_n!を、上限を含めて作ります。逆元を使わないので、任意の正の法で使えます。

`FactorialTable(int max_n, long long modulus=998244353)`

| メンバ | 内容 |
|---|---|
| `factorial(long long n)` | n! mod m |
| `inverse_factorial(long long n)` | (n!)の逆元 mod m |
| `max_n()`, `modulus()` | 前計算の上限と法 |
| `factorials()`, `inverse_factorials()` | 表のconst参照 |

max_n>=0、modulus>=1。それ以外は `invalid_argument`。FactorialTableは `gcd(max_n!,m)=1` を要求し、不成立は `domain_error` です。素数法なら max_n<m で十分ですが、条件を満たす合成数法でも使えます。法1は全て0で、逆元の代表元も0という3.13と同じ規約です。max_n=0では1要素を作ります。

表へのnの問い合わせは `[0,max_n]`、範囲外は `out_of_range`。戻す参照の寿命はオブジェクトに従います。

## 原理・計算量

階乗を小さい順で掛け、最後の階乗の逆元を拡張Euclidで一度だけ求めます。逆階乗は `g[i-1]=i*g[i] mod m` として逆向きに埋めます。

空の積0!=1。逆元が不要な用途では `factorials` 関数を使えば、max_n!が0になる法でも階乗を計算できます。

一般の64ビット法ではO(N log m+log m)時間、O(N)空間。m<=2^32では安全な積を直接計算でき、O(N+log m)時間です。要素参照はO(1)。

## 使用する問題・検証範囲

[使用する問題](https://cses.fi/problemset/task/1715)：Creating Strings II。文字数の階乗へ各文字の個数の逆階乗を掛けます。

小さい部分集合・順序付きの選択・弱合成・括弧列・集合族を全列挙し、個数を独立に比較します。合成数法・法1・64ビットの法、0・不正な引数・前計算超過・加算前の範囲判定、旧APIも確認します。200万の階乗表・1500行のPascal表・22集合の包除も含みます。

[検証入口](../../verify/run_combinatorics.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、Python任意精度整数、6ジャッジ形式・384ケース。オンラインACは未確認です。
