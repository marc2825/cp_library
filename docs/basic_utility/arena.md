---
title: 型付きArena
documentation_of: //basic_utility/arena.hpp
---

# 型付きArena（F-13）

`Arena<T,BlockSize=256>`はT専用のブロック型メモリプールです。各ブロックを独立確保するので、追加時に既存ノードのID・アドレスが変わりません。汎用のSTL allocatorではなく、ノードをまとめて所有するコンテナです。

## 使用例

```cpp
#include <iostream>
#include <limits>
#include "basic_utility/arena.hpp"
struct Node { int value; std::size_t previous; };
int main() {
    cp::Arena<Node, 2> nodes;
    const auto nil = std::numeric_limits<std::size_t>::max();
    auto common = nodes.emplace(Node{10, nil});
    const auto* saved = &nodes[common];
    auto first = nodes.emplace(Node{20, common});
    auto second = nodes.emplace(Node{30, common});
    std::cout << nodes[common].value << " " << nodes[first].value << " " << nodes[second].value << "\n";
    std::cout << (saved == &nodes[common]) << "\n";
}
```

出力:

```text
10 20 30
1
```

## API・条件

| API | 規約 |
|---|---|
| `emplace(args...)` | Tを直接構築し、0始まりのsize_t IDを返す |
| `operator[](id)` | T&／const T&。範囲外はout_of_range |
| `size()/capacity()/empty()` | 生存要素数／確保slot数／空判定 |
| `clear()` | 逆構築順に全Tを破棄し、全ブロックを解放 |

BlockSize>0、Tのデストラクタはnoexceptであることが必要です。Tの既定構築・コピー・ムーブは要求しません。C++17の過剰アラインメント付きnewとalignas(T)を使い、Tに必要な整列を保持します。

構築が投げた場合、sizeと生存要素は変わりません。先に確保した空ブロックは残る場合があります。容量がsize_tを超える場合はlength_error、実際の割当失敗は標準ライブラリの例外を伝播します。Arena自身のコピー・ムーブは禁止です。同じArenaをTのコンストラクタ／デストラクタから再入して変更しないでください。

**clear／破棄で全ID・参照・ポインタが無効**です。clear後のIDは0から再利用し、古いIDの世代検査はしません。個別削除やIDの再利用は提供しません。永続木では各版の根IDを保存し、変更経路だけemplaceして共有ノードを変更しないでください。使用例は二つのリスト版が共通の尾を持つ例です。ライブラリ側で永続性を強制するものではありません。

Tの構築を除き、追加はBlockSize固定なら償却O(1)、取得O(1)、clearはO(N)回の破棄。領域O(capacity×sizeof(T))とブロックポインタ配列です。新ブロック確保時の初期化にはO(BlockSize×sizeof(T))の仕事があります。

## 使用する問題・検証

永続セグメント木、探索木、Trie等の多数のノード。128byte整列、構築例外、破棄順、clear再利用、参照保持、20万要素を確認し、401版の経路コピー木を素朴な配列版と照合します。

`make verify-foundation-tools` / `python3 verify/run_foundation_tools.py --sanitize`でDebug/Release、単独include、全カテゴリ混在、説明内使用例、独立比較を検証します。オンラインACは未確認です。

[追加範囲・検証](../foundation_tools/README.md) / [一次資料](../foundation_tools/references.md) / [カテゴリ索引](README.md) / [図解](../../visualizations/foundation_tools/index.html#arena) / [図の範囲](../../visualizations/foundation_tools/README.md)
