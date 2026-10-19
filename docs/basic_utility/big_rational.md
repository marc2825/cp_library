---
title: 多倍長有理数
documentation_of: //basic_utility/big_rational.hpp
---

# 多倍長有理数（F-12）

`BigRational`は[BigInt](big_integer.md)を分子・分母に持つ正確な有理数です。Boostヘッダが必要です。分母は常に正、分子と分母は互いに素、ゼロは0/1へ正規化します。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/big_rational.hpp"
int main() {
    auto a = cp::BigRational::parse("6/-8");
    cp::BigRational b(1, 6);
    std::cout << a + b << " " << a * b << "\n";
    std::cout << cp::BigRational::parse("-0/999").to_string() << "\n";
}
```

出力:

```text
-7/12 -1/8
0/1
```

## API・条件

- `BigRational()`は0/1。`BigRational(BigInt n, BigInt d=1)`と組込み整数のコンストラクタを提供します。分母0はdomain_error。
- `numerator()/denominator()`はconst参照。`sign()`は-1/0/1、明示boolは非ゼロ判定です。
- 四則演算・複合代入・単項符号・6比較・`cp::abs`に対応します。整数／BigIntとも混在できます。浮動小数点への暗黙変換はありません。
- `parse(string_view)`は10進の整数、または`n/d`を全トークンとして読む静的関数です。符号・先頭ゼロはBigIntと共通。不正な桁・空欄・空白・余分なslashはinvalid_argument、分母0はdomain_error。
- `to_string()`とostreamは常に`n/d`形式の10進です。分母1も省略しません。istreamは空白区切りの1トークンを読み、不正／分母0ならfailbitを立て、元の値を保ちます。
- ゼロによる除算はdomain_errorで、複合代入でも元の値を保持します。コピーは独立、ムーブ元は0/1。演算は別の結果を作ってからswapするため、割当失敗時も元の値を保持します。

加算は分母のGCDを使って共通分母を小さくし、乗算は交差する分子・分母を約分してから掛けます。比較は正の分母同士の交差積で行います。四則演算は定数回のBigInt演算とGCD、比較は2回のBigInt乗算です。途中の整数のビット長をnとすれば、領域はO(n)、時間はBigIntの乗除算・GCDのコストに従います。分子・分母の桁数は演算で増えるためO(1)とはみなしません。

## 使用する問題・検証

正確な比率・確率・有理数係数の行列や関数合成。小整数から作る73,984組を交差積の独立計算で比較し、Python Fractionで最大2048bitの1,200組を照合します。符号、ゼロ、約分、自己演算、入力失敗、既存Matrix／AffineFunctionMonoidへの接続を含みます。

`make verify-foundation-tools` / `python3 verify/run_foundation_tools.py --sanitize`でDebug/Release、単独include、全カテゴリ混在、説明内使用例、独立比較を検証します。オンラインACは未確認です。

[追加範囲・検証](../foundation_tools/README.md) / [一次資料](../foundation_tools/references.md) / [カテゴリ索引](README.md) / [図解](../../visualizations/foundation_tools/index.html#big_rational) / [図の範囲](../../visualizations/foundation_tools/README.md)
