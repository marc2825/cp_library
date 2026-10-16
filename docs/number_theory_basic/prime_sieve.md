---
title: 素数篩
documentation_of: //number_theory_basic/prime_sieve.hpp
---

# 素数篩

エラトステネスの篩で、上限以下の素数判定表と素数リストを作ります。素数pについてp²から倍数を消します。それより小さい合成数は、すでに小さい素因数で消されています。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#sieve) / [実装](../../number_theory_basic/prime_sieve.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/prime_sieve.hpp"
int main() {
    cp::PrimeSieve sieve(20);
    for (int p : sieve.primes()) std::cout << p << ' ';
    std::cout << '\n' << sieve.is_prime(1) << ' ' << sieve.is_prime(19) << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/prime_sieve.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`PrimeSieve(int limit=0)`

| メンバ | 内容 |
|---|---|
| `limit()` | 構築時の上限 |
| `is_prime(int n)` | nが素数か。O(1) |
| `table()` | const参照の `vector<bool>`、長さlimit+1 |
| `primes()` | const参照の `vector<int>`、昇順 |

- 範囲は両端を含む `[0,limit]`。`limit>=0`、負の上限は `invalid_argument`。
- 0と1は非素数。limit=0,1も安全で、素数リストは空です。
- `is_prime` の範囲外は `out_of_range`。表のサイズを超える整数の判定には使えません。
- 表とリストの参照の寿命はオブジェクトに従います。メモリを確保できる上限を選んでください。

構築O(N log log(N+2))時間、表O(N)ビットと素数リストO(π(N))語の空間。上限と積を比較する際は除算で判定し、倍数走査はlong longで行います。

## 使用する問題・検証範囲

[使用する問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_1_C)：AOJ ALDS1_1_C — Prime Numbers。

0〜10,000の全数を素朴な割り算と比較し、小さい上限・素数の平方・範囲外・200万の篩を確認します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
