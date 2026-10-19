---
title: 安定基数ソート
documentation_of: //basic_utility/radix_sort.hpp
---

# 安定基数ソート（F-17）

整数キーを下位byteから安定に並べるLSD基数ソートです。同じキーを持つレコードの入力順を保持します。

## 使用例

```cpp
#include <iostream>
#include <utility>
#include <vector>
#include "basic_utility/radix_sort.hpp"
int main() {
    std::vector<std::pair<int, char>> items{{3, 'a'}, {-1, 'b'}, {3, 'c'}, {0, 'd'}};
    cp::radix_sort_by(items, [](const auto& item) { return item.first; });
    for (const auto& item : items) std::cout << item.first << ":" << item.second << "\n";
}
```

出力:

```text
-1:b
0:d
3:a
3:c
```

## API・条件

- `radix_sort(vector<Int>&)`：整数配列を数値の昇順に整列。
- `radix_sort_by(vector<T>&, Key key)`：key(const T&)を各要素で1回評価して整数キーを保存し、レコードを安定に整列。

キーはboolを除く組込み整数型で、符号付き／なしの8・16・32・64bit、GCC/Clangの128bitに対応します。キー型の全byteを使います。負数はunsignedへの変換後に最上位bitを反転し、通常の大小関係へ合わせます。浮動小数点・文字列・任意の複合キーは対象外です。

Tはコピー構築可能であることが必要です。Tの既定構築は不要。キー・添字だけを各passで並べ替え、最後に別vectorへレコードをコピーしてswapします。projectionやコピーが投げた場合、入力を保持します（projection等が外部経由で元配列を変更しないこと）。成功時は元の全参照・iteratorが無効になります。

キーのbyte数W、要素数NとするとO(W(N+256))時間にprojection・レコードコピーのコストを加えます。追加領域はN個のキー、2N個のsize_t添字、N個のレコード、256個のcountです。レコードが大きい場合や小さいNではstd::stable_sort等と性能を比較してください。

## 使用する問題・検証

大量の整数の整列、同一キーの元順を必要とするイベント処理。10整数型・1,500ランダムケースをstd::stable_sortと比較。全型の極値、重複キー、空、projection回数、コピー例外、20万要素、Pythonでのsigned128bitキー比較を検証します。

`make verify-foundation-tools` / `python3 verify/run_foundation_tools.py --sanitize`でDebug/Release、単独include、全カテゴリ混在、説明内使用例、独立比較を検証します。オンラインACは未確認です。

[追加範囲・検証](../foundation_tools/README.md) / [一次資料](../foundation_tools/references.md) / [カテゴリ索引](README.md) / [図解](../../visualizations/foundation_tools/index.html#radix_sort) / [図の範囲](../../visualizations/foundation_tools/README.md)
