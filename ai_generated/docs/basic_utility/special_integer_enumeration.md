---
title: 特殊な整数の昇順列挙（U-22）
documentation_of: //basic_utility/special_integer_enumeration.hpp
---

# 特殊な整数の昇順列挙（U-22）

[ユーティリティ索引](README.md) / [Uの全14項目](../utility_search/README.md) / [操作できる図解](../../visualizations/utility_search/index.html#special_integer_enumeration)

10進回文、使える数字を制限した整数、指定因子の積で作れる整数を重複なく昇順で列挙します。候補の形を利用して生成し、整数区間全体を走査しません。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/special_integer_enumeration.hpp"
int main() {
    auto print = [](std::uint64_t x) { std::cout << x << ' '; return true; };
    cp::enumerate_palindromic_integers(90, 140, print);
    std::cout << '\n';
    cp::enumerate_digit_restricted_integers(0, 25, {0, 2}, print);
    std::cout << '\n';
    cp::enumerate_generated_products(20, {2, 3}, print);
    std::cout << '\n';
}
```

出力（行末の空白は省略）:

```text
99 101 111 121 131
0 2 20 22
1 2 3 4 6 8 9 12 16 18
```

## API・使用条件

U-22の対象を次の3種類のuint64整数列挙と定めました。既存の[Gray code](gray_code.md)、[同popcount列挙](bit_operations.md)、[商が一定の区間](quotient_enumeration.md)、[倍数列挙](multiple_enumeration.md)、[整数分割](integer_partitions.md)はそのまま共有します。

| API | 出力 |
|---|---|
| `enumerate_palindromic_integers(first,last,visit)` | `[first,last)` 内で標準10進表記が回文の整数。0も回文 |
| `enumerate_digit_restricted_integers(first,last,vector<int> digits,visit)` | `[first,last)` 内でdigitsだけを使う整数。先頭0なし、0は数字0を許したときのみ |
| `enumerate_generated_products(last,vector<uint64_t> factors,visit)` | `[1,last)` 内の `∏ factors[i]^ei`、ei≥0。空積1を含む |

visitは `bool(uint64_t)` 相当。trueで継続し、falseならその値を渡した直後に停止します。関数の返り値は全探索を終えたらtrue、callbackのfalseで止まったらfalseです。最後の出力に対してfalseを返した場合もfalseになります。例外は呼出し元へ伝播します。結果を全部vectorに保存するか、最初のk件で止めるかは利用側で選べます。

範囲は半開、`last=UINT64_MAX`でも最大値そのものは含みません。逆転した範囲、0〜9以外のdigit、2未満のfactorは `invalid_argument`。引数検査は空範囲でも行います。digits/factorsは任意順で重複可、内部で正規化します。空digitsは出力なし。空factorsはlast>1なら1のみです。因子は素数・互いに素である必要はなく、{4,6}はその2数の積で生成できる値だけを出します。素数集合を渡すと、その素因数だけを持つsmooth numberの列挙になります。

## 原理・計算量

最大桁数D=20、出力数K、入力digit/factor個数q、異なる有効因子数dとします。callbackの実行費用は別です。

- 回文は前半を反転して後半を作ります。各桁数の最初の候補を前半に対する二分探索で求めるため、狭い大数区間でも小さい回文を走査しません。O(D³+KD)時間、追加記憶O(1)。反転でuint64を超える候補は生成を終えます。
- 数字制限は桁数順・数字順の深さ優先探索。prefixの数値区間が入力範囲と交わらない枝を除きます。正規化O(q log(q+1))、探索は訪問prefix数VにO(V)、Vの上界はO(10D(K+D))、記憶O(q+D)。再帰の深さは20以下です。
- 積の列挙は生成済み列と各因子の積の列を多方向マージします。同じ最小候補を持つ全ポインタを進め、合成因子や重複した表現も1回だけ出力します。O(q log(q+1)+dK)時間、O(q+K)記憶。生成済みの値は保持するため、全出力が巨大な場合は途中で打ち切ってください。乗算前に `(last−1)/factor` と比較します。

## 使用する問題・検証

回文の性質を調べる候補列挙、数字3/5/7だけを許す数の探索、2/3/5-smooth numberの順次生成など。[AtCoder ABC363 D](https://atcoder.jp/contests/abc363/tasks/abc363_d?lang=en)は回文の前半による順序付けの関連問題ですが、最大制約ではuint64を超えるため本API単体で全制約を解くとは主張しません。

[比較テスト](../../verify/utility_search/retroactive_enumeration_test.cpp)では、100万未満の回文を文字列反転と照合、全1024種類の数字集合・256種類の因子集合を整数走査／到達可能性DPと比較します。600個の範囲、20桁の回文、UINT64_MAX付近、合成因子、重複、空集合、途中打切り、不正入力も検査します。`python3 verify/run_utility_search.py`。オンライン提出ACは未確認です。

## 参考資料

[ABC363公式解説](https://atcoder.jp/contests/abc363/editorial/10488)の回文の前半による表現、[Jayadev Misra, Programming in Orc, 6.3.2.4](https://www.cs.utexas.edu/~misra/OrcBook.pdf)のHamming列のマージを原理の参考にしました。C++コードは独自実装です。[出典一覧](../utility_search/references.md)。
