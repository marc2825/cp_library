---
title: 拡張Euclid
documentation_of: //number_theory_basic/extended_gcd.hpp
---

# 拡張Euclid

`a*x+b*y=gcd(a,b)` を満たす整数係数を返します。余りを `a,b` の線形結合として持つことで、互除法の最後にBézout係数が得られます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#extended) / [実装](../../number_theory_basic/extended_gcd.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/extended_gcd.hpp"
int main() {
    auto result = cp::extended_gcd(-30, 18);
    std::cout << result.gcd << ' ' << result.x << ' ' << result.y << '\n';
    std::cout << -30 * result.x + 18 * result.y << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/extended_gcd.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`extended_gcd(long long a, long long b) -> ExtendedGCDResult`

- `gcd`: `unsigned long long` の非負な最大公約数。
- `x,y`: `long long` のBézout係数。負数・0・`LLONG_MIN` も受け取れます。
- `(a,b)=(0,0)` の結果は `{0,0,0}` です。
- 解の最小化や辞書順の保証はありません。`g>0` のとき一般解は `x+k*(b/g), y-k*(a/g)` です。符号なしの `g` と混ぜるC++の型変換には注意してください。
- 数学的な恒等式が成り立っていても、利用側の `a*x+b*y` の中間積は64ビットに収まらない場合があります。大きい入力の恒等式を確認するには任意精度整数等を使ってください。

O(log(max(|a|,|b|)+1))時間、O(1)空間。絶対値を符号なし型で持ち、最後に符号を戻します。両入力が非零の場合、終了に必要な係数は他方の入力の大きさをGCDで割った値で制限されます。係数の符号が交互に変わるため、更新の積の絶対値も更新後の係数以下です。最後の余り0の係数更新を省き、最小整数の絶対値に等しい不要な係数を生成しません。0を含む入力の係数は0と±1で表せます。

## 使用する問題・検証範囲

[使用する問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_E)：AOJ NTL_1_E。問題固有の係数最小化・同点条件は検証用ラッパで一般解の折点付近を調べます。

全40,401対とPython任意精度の恒等式、連続Fibonacci、全符号の極値。戻り値の係数がlong longに入ること、最小整数の除算・符号反転の安全性をASan/UBSanでも確認します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
