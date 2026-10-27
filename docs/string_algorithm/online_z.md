---
documentation_of: //string_algorithm/online_z.hpp
---

# Online Z（S-21）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/online_z.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#online_z)

## API・使用条件

`OnlineZ(text={})`、append(byte/string_view)、query(position)、values()、size()。

query(i)は現在の先頭とsuffix iのLCP。先頭値は現在長、i=nは0。追記で以前のZ値が伸びる場合も再計算なしで問い合わせる。乱択hashは使わない。values()は全Z配列を実体化する。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

prefix長i+kのborderに長さkが存在することと、suffix iがprefix kに一致することは同値。オンラインborder木のancestor判定を使ってkを二分探索する。

追記は償却O(log n)、query O(log² n)、領域O(n log n)。values()のみ既存Z AlgorithmでO(n)。定数時間queryや全Z配列の定数時間更新は主張しない。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/online_z.cpp)。

```cpp
#include "string_algorithm/online_z.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::OnlineZ z("ababa");
    std::cout << z.query(2) << ' ';
    z.append('b');
    std::cout << z.query(2) << '\n';
    print(z.values());
}
```

出力:

```text
3 4
6 0 4 0 2 0
```

## 使用する問題・検証

追記中の文字列のprefix一致長、増加するZ値への厳密なオンライン問い合わせ。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
