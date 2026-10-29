---
title: IndexSet・一様ランダム選択
documentation_of: //heuristic_search/index_set.hpp
---

# E-02 IndexSet・一様ランダム選択

[一覧](README.md) / [ヘッダ](../../heuristic_search/index_set.hpp) / [実行例](../../examples/search_parsing_extensions/index_set.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#index_set) / [出典](references.md)

密な配列と逆位置を持ち、削除位置へ末尾の要素を移して穴を埋める。

## APIと条件

insert・erase・contains・position・at・sample・clear、values。

固定宇宙[0,n)。要素IDは安定、dense位置はeraseで変わる。sampleは空ならnulloptで乱数を消費しない。コピー時にも容量を確保、move元は空宇宙。

insert(id)/erase(id)は変更があればtrue、既にその状態ならfalse。contains(id)、position(id)は固定宇宙のIDを受け、positionは未所属なら−1です。at(dense_index)とvalues()は現在の密配列を参照します。範囲外ID/位置はout_of_range。

eraseは末尾IDを削除位置へ移すため、dense添字・配列への参照は更新をまたいで保存しないでください。ID自体は変わりません。sample(Random&)は所属IDのoptionalを返し、整数の棄却法によって偏りなく選びます。空集合では乱数を消費しません。

## 計算量・用途

構築/コピーO(n)、insert/erase/contains/atはO(1)、抽出は期待O(1)、clearは現在の要素数O(k)、領域O(n)。

問題例・用途: 候補集合の追加削除、未使用頂点からの一様選択。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/index_set.hpp"
#include <iostream>
using namespace cp;
int main() {
    IndexSet selected(6);
    for (int e : {2, 4, 1})
        selected.insert(e);
    selected.erase(4);
    std::cout << selected.size() << "\n";
    for (std::size_t i = 0; i < selected.size(); ++i)
        std::cout << (i ? " " : "") << selected.at(i);
    std::cout << "\n";
    Random random(7);
    const auto sampled = selected.sample(random);
    std::cout << bool(sampled && selected.contains(*sampled)) << "\n";
}
```

出力:

```text
2
2 1
1
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
