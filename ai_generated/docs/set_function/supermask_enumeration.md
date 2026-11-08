---
title: Supermask列挙
documentation_of: //set_function/supermask_enumeration.hpp
---

# Supermask列挙

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/set_function/index.html#supermask) / [実装](../../set_function/supermask_enumeration.hpp) / [一次資料](references.md)

## usage

```cpp
#include <cstdint>
#include <iostream>
#include "set_function/supermask_enumeration.hpp"
int main() {
    cp::for_each_supermask(2, 4, [](std::uint64_t s) { std::cout << s << ' '; });
    std::cout << '\n';
}
```

[動く使用例](../../examples/set_function/supermask_enumeration.cpp)。ライブラリのルートをincludeパスに追加します。

## API・使用条件

`for_each_supermask(uint64_t mask,int bits,Callback callback) -> void`

下位bitsビットを全体集合として、maskを含む上位集合を数値の**昇順**でcallbackへ渡します。mask自身と全体集合も含みます。bitsは0〜64。bits<0、bits>64、maskがその全体集合に収まらない場合はinvalid_argumentで、callbackを一度も呼びません。

bits=0はmask=0だけを許し、0を1回渡します。bits=64も使え、全体集合UINT64_MAXから先へ折り返しません。callbackの戻り値は無視、例外は伝播します。全体集合に含まれる自由なビットの全組合せを列挙するため、その数に応じた時間が必要です。

C++17・namespace cp。[共通仕様と旧API](README.md#共通仕様)も参照してください。

## 原理・計算量

現在値sから(s+1)|maskへ進みます。数値を増やした後に必須のビットをORで立て直すと、次の上位集合になります。全体集合に達したかを加算前に検査します。64ビットの全体集合は~uint64_t(0)として作り、1<<64を計算しません。

全体集合のビット数n=bits、k=popcount(mask)としてO(2^(n−k))時間、追加空間O(1)。callback費用は別です。

## 使用する問題・検証範囲

小さい全体集合での[CSES SOS Bit Problem](https://cses.fi/problemset/task/1654)の素朴解や、指定した要素を全て含む集合の走査に利用できます。列挙APIは独立なビット位置の全組合せと比較し、数値の順序も検証します。

[検証入口](../../verify/run_set_function.py)は9ヘッダの単独／逆順／混在includeと複数翻訳単位、7使用例、4ジャッジ形式・256ケースを確認します。2,047基準mask・177,146組、6,654三値表・180ランダム表、Python任意精度2,428クエリを、定義から直接集計した和・交互和・集約と比較します。空集合・0ビット・63/64ビット、法12の型、複素数、単位元不要の型、262,144個のint8演算、符号付き／符号なし極値、途中超過と失敗前の状態、2^20項の全出力と正確な更新回数を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行し、移動した3.12 DP APIも回帰検証します。図解は2,853入力・50,838スナップショット、15拒否条件とブラウザ操作を確認します。オンラインACは未確認です。
