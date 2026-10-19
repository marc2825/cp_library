---
title: 多倍長整数
documentation_of: //basic_utility/big_integer.hpp
---

# 多倍長整数（F-11）

`cp::BigInt`はBoost.Multiprecisionのcpp_intを使う符号付き任意精度整数です。加減乗除・剰余・比較・GCD・LCMを公開し、10進／16進の厳密な文字列変換と組込み整数への範囲検査を揃えます。**Boostヘッダが必要**です。追加リンクライブラリは不要で、ローカルではBoost 1.74・GCC 11.4・C++17で検証しています。`basic_utility/all.hpp`からも読み込まれます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/big_integer.hpp"
int main() {
    cp::BigInt a("123456789012345678901234567890"), b(97);
    auto [q, r] = cp::divmod(a, b);
    std::cout << q << " " << r << "\n";
    std::cout << cp::gcd(a, cp::BigInt(90)) << "\n";
    std::cout << cp::BigInt("-0xFF", 16).to_string(16) << "\n";
    auto [negative_q, negative_r] = cp::divmod(cp::BigInt(-17), cp::BigInt(5));
    std::cout << negative_q << " " << negative_r << "\n";
}
```

出力:

```text
1272750402189130710322005854 52
90
-ff
-3 -2
```

## API・使用条件

| API | 規約 |
|---|---|
| `BigInt()` | 0 |
| `BigInt(integer)` | 組込み整数から正確に構築。GCC/Clangの128bitにも対応 |
| `explicit BigInt(string_view, unsigned base=10)` | 10進または16進で全体を読む |
| `BigInt::parse(text, base=10)` | 同じ厳密なパーサ。BigIntを返す |
| `to_string(base=10)` | 正規化済み文字列。16進は小文字・prefixなし・負数は`-`付き |
| `to_integral<T>()` | 整数Tへ範囲検査して変換。表現不能ならout_of_range。boolは対象外 |
| `sign()` / 明示的bool変換 | -1／0／1、または非ゼロ判定 |
| `+ - * / %`、各複合代入、単項`+ -`、前置／後置`++ --` | 任意精度の整数演算。演算結果は評価済みBigInt |
| `== != < <= > >=` | 数値比較。組込み整数との混在も可能 |
| `cp::divmod(a,b)` | `{quotient,remainder}`を同時に計算 |
| `cp::abs(a)` / `cp::gcd(a,b)` / `cp::lcm(a,b)` | いずれもBigIntを返す。GCD／LCMは非負 |
| `operator<<` / `operator>>` | **常に10進**の文字列トークン。hex/oct/showbase/showpos等の数値書式は使わない |
| `native()` / `explicit BigInt(backend_type)` | cpp_intへのconst参照／cpp_intからの構築。Boostとの相互運用 |

コピーは独立した値です。ムーブ元は破棄・再代入でき、元の数値を保持する保証はありません。浮動小数点への暗黙変換、固定幅へ黙って切り詰める変換、ビット演算はこのラッパーには含めません。必要ならnative()を通じてBoostのAPIと条件を使います。Boostの式テンプレートをnative()から作る場合は、参照先の寿命に注意してください。

### 商と余り

商は**0方向へ切り捨て**、`a=q*b+r`かつ`abs(r)<abs(b)`です。非ゼロの余りは被除数aと同じ符号になります。たとえば`-17 / 5 = -3`、`-17 % 5 = -2`です。Pythonの`//`・`%`の負数規約とは異なります。ユークリッド剰余が必要なら、正のmに対してrが負の場合にmを足してください。

`/`・`%`・`divmod`の除数0はdomain_error。複合代入でも除数を先に検査し、元の値を保持します。`gcd(0,0)=0`、`lcm(a,0)=lcm(0,b)=0`です。`cp::gcd(long long,long long)`等の既存APIはそのままで、BigInt引数のときだけこのオーバーロードを使用します。

### 入出力と変換

先頭の符号`+`／`-`は省略可能です。base=16では符号の後に`0x`／`0X`を置けます。base=10ではprefixを認めず、`000009`も10進の9として読みます。負のゼロは0へ正規化します。16進の負数は`-ff`のような符号と絶対値であり、固定幅2の補数ではありません。

空文字、符号だけ、prefixだけ、前後や途中の空白、桁区切り、埋込みNUL、不正な桁、10／16以外の基数はinvalid_argument。istream入力は空白区切りで全トークンを検査し、不正ならfailbitを立て、代入先は保持します。不正なトークンは消費済みです。EOF時も値を保持します。ストリームのexceptions設定でfailbit例外を有効にすればios_base::failureが発生します。

FastIOとは文字列を介して接続します。`scanner.read(token)`後に`BigInt::parse(token)`、出力は`printer.writeln(value.to_string())`です。固定幅整数用のFastIOテンプレートにBigIntを直接渡すことはできません。

## 計算量・検証した規模

固定桁数の上限は設けず、利用可能なメモリとBoost内部の容量に従います。メモリ不足等の例外は伝播します。整数の演算をO(1)とみなさないでください。最大ビット長をn、乗算コストをM(n)、除算コストをD(n)とすると、加減算・比較はO(n)、乗算はM(n)、商と余りはD(n)、GCDは保守的にO(n D(n))以内、LCMはGCD・除算・乗算の合計です。値の保持領域はO(n)です。10進／16進の入力は定数桁ずつの積和で、桁数dに対してO(d²)の上界を持ちます。10進出力もO(d²)、16進出力はO(d)です。

乗算・除算・GCDはBoostへ委譲します。確認したBoost 1.74では十分大きい乗算にKaratsubaが使われ、等しい長さではM(n)=O(n^log₂3)、通常の除算はO(n²)を目安にします。切替閾値や性能はBoostの版・ビルド条件に依存します。独自のNTT大整数乗算は追加していません。[Boostの説明と設計資料](references.md#多倍長整数-f-11)。

今回の独立比較は**入力10,000桁、積20,000桁まで**です。これはAPIの上限ではなく、実際に検証した範囲です。より大きな桁数の時間制限適合を保証するものではありません。`python3 verify/run_big_integer.py --benchmark`で参考計測を再実行できます。性能値に合否の閾値は設けていません。

2026-10-19の単回計測例（Intel Core Ultra 9 285K、GCC 11.4、Boost 1.74、`-O2`、sanitizerなし）：

| 入力桁数（各辺） | 2数parse | 乗算（10回平均） | 積÷片方 | GCD | 積の10進出力 |
|---:|---:|---:|---:|---:|---:|
| 1,000 | 0.0068 ms | 0.0015 ms | 0.0135 ms | 0.0972 ms | 0.0311 ms |
| 10,000 | 0.240 ms | 0.0515 ms | 0.492 ms | 10.9 ms | 2.16 ms |

入力生成・検証は各演算の計測区間外です。固定seedで作る2つの同桁数整数を使います。単回のローカル計測なので、汎用ベンチマークやオンラインジャッジの速度保証ではありません。

## 使用する問題・検証

大整数の四則演算、階乗や組合せの正確な値、固定幅を超えるGCD、整数行列積、巨大な値の区間和に利用できます。既存のMatrixと多項式区間加算への接続も検証しています。各アルゴリズムのTに対する前提は別途満たす必要があります。

```sh
make verify-big-integer
python3 verify/run_big_integer.py --sanitize
python3 verify/run_big_integer.py --benchmark
node verify/visualizations/big_integer_model_test.js
node verify/visualizations/big_integer_browser_test.mjs --browser /path/to/chromium
```

単独・重複include、全カテゴリall.hppの混在、複数翻訳単位、説明と一致するusageを確認します。Debug/Releaseとも65,536組の小整数全探索、各整数型と128bitの変換境界、不正入力・EOF・例外付きistream、自己代入演算、コピー／ムーブ、負数・ゼロ、FastIO接続を検証します。Python任意精度整数を独立オラクルに、ランダム・2冪境界・10,000桁の5,207出力を比較します。オンライン提出・ACは未確認です。

[ヘッダ](../../basic_utility/big_integer.hpp) / [動く使用例](../../examples/basic_utility/big_integer.cpp) / [カテゴリ索引](README.md) / [図解](../../visualizations/big_integer/index.html) / [図解の範囲](../../visualizations/big_integer/README.md) / [残作業一覧](../remaining_work.md)
