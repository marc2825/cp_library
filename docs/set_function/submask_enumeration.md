---
title: Submask列挙
documentation_of: //set_function/submask_enumeration.hpp
---

# Submask列挙

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/set_function/index.html#submask) / [実装](../../set_function/submask_enumeration.hpp) / [一次資料](references.md)

## usage

```cpp
#include <cstdint>
#include <iostream>
#include "set_function/submask_enumeration.hpp"
int main() {
    cp::for_each_submask(10, [](std::uint64_t s) { std::cout << s << ' '; });
    std::cout << '\n';
    cp::for_each_submask(10, [](std::uint64_t s) { std::cout << s << ' '; }, false);
    std::cout << '\n';
}
```

[動く使用例](../../examples/set_function/submask_enumeration.cpp)。ライブラリのルートをincludeパスに追加します。

## API・使用条件

`for_each_submask(uint64_t mask,Callback callback,bool include_empty=true) -> void`

maskの部分集合を数値の**降順**にcallback(sub)へ渡します。mask自身を含み、既定では最後に空集合0も含みます。include_empty=falseは0だけを省略。mask=0は既定で1回、falseなら0回です。uint64_tの最上位ビットも扱います。負の値を符号なし引数に渡さないでください。

callbackの戻り値は無視します。戻り値による打切り機能はありません。例外は呼出し元へ伝播します。列挙結果を内部に蓄積せず、コールバックへ順に渡します。

C++17・namespace cp。[共通仕様と旧API](README.md#共通仕様)も参照してください。

## 原理・計算量

現在値sから(s−1)&maskへ進みます。1を引いて下位の桁を繰り下げ、maskの外に立った1をANDで消すと、sより小さい部分集合のうち最大のものになります。s=0を確認して終了するため、0から元のmaskへ戻る無限ループを避けます。

k=popcount(mask)としてO(2^k)時間、追加空間O(1)。callback費用は別です。全てのnビットmaskとその全submaskを列挙すると、合計3^n組になります。

## 使用する問題・検証範囲

小さい全体集合での[ CSES SOS Bit Problem ](https://cses.fi/problemset/task/1654)の素朴解や、全submaskを使う集合分割DPに利用できます。列挙APIは独立なビット位置の全組合せと比較し、数値の順序も検証します。

[検証入口](../../verify/run_set_function.py)は9ヘッダの単独／逆順／混在includeと複数翻訳単位、7使用例、4ジャッジ形式・256ケースを確認します。2,047基準mask・177,146組、6,654三値表・180ランダム表、Python任意精度2,428クエリを、定義から直接集計した和・交互和・集約と比較します。空集合・0ビット・63/64ビット、法12の型、複素数、単位元不要の型、262,144個のint8演算、符号付き／符号なし極値、途中超過と失敗前の状態、2^20項の全出力と正確な更新回数を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行し、移動した3.12 DP APIも回帰検証します。図解は2,853入力・50,838スナップショット、15拒否条件とブラウザ操作を確認します。オンラインACは未確認です。
