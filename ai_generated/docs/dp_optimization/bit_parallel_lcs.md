---
documentation_of: //dp_optimization/bit_parallel_lcs.hpp
---

# Bit-parallel LCS

[P-02の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#bit_parallel_lcs) / [ヘッダ](../../dp_optimization/bit_parallel_lcs.hpp)

`bit_parallel_lcs_length(string_view a, string_view b)` はLCS長をsize_tで返します。NUL・上位bitを含む256種類の**バイト列**で、UTF-8の文字単位ではありません。短い列をbit列に選び、各バイトの出現位置maskを前計算します。長さ64の制限はなく、word間のshift carryと減算borrowを伝播します。

LCS表の隣接差分をbitで持ち、`x = row | mask`、`y = (row << 1) | 1`、`row = x & ~(x-y)` を更新します。最終rowのpopcountが長さです。空列の答えは0。任意の比較関数・一致添字の復元が必要なら既存の[lcs](lcs.md)を使います。

n=max長、m=min長、B=ceil(m/64)。時間O(nB+m+256B)、領域O(256B)。m=0はO(1)。用途: 長いバイト列の類似度、LCS長のみの問い合わせ。63/64/65等のword境界・ランダム256バイト・10万文字を比較検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/bit_parallel_lcs.hpp"
int main() {
    std::cout << cp::bit_parallel_lcs_length("ABCBDAB", "BDCABA") << '\n';
}
```

出力:

```text
4
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
