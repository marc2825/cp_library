---
title: seed付き複合キーハッシュ
documentation_of: //basic_utility/seeded_hash.hpp
---

# seed付き複合キーハッシュ

整数・pair・tuple・列を、指定した64bit seedでハッシュします。既存の[SplitMix64乱数](../heuristic_search/random.md)を局所的に使い、呼び出す順番で結果が変わる状態を持ちません。

## 使用例

```cpp
#include <iostream>
#include <tuple>
#include <vector>
#include "basic_utility/seeded_hash.hpp"
int main() {
    cp::SeededHash hash(42), replay(42);
    auto key = std::make_tuple(-7, std::vector<int>{1, 2, 3});
    std::cout << hash.seed() << " " << (hash.hash64(key) == replay.hash64(key)) << "\n";
}
```

出力:

```text
42 1
```

## API・使用条件・計算量

| API | 意味 |
|---|---|
| `SeededHash(uint64_t seed = random_seed())` | 明示seedで再現。省略時はrandom_deviceから生成 |
| `seed()` | 保存・再実行用のseed |
| `hash64(key)` | uint64ハッシュ値 |
| `operator()(key)` | size_tへ変換した値。HashMapや標準unordered_mapのHashとして使用可能 |
| `random_seed()` | random_deviceの4出力を混ぜてseedを生成 |

対応するキーは整数（bool・128bitを含む）・enum・std::pair／tuple／array・begin/endで走査できる列の再帰的な組合せです。std::string／string_view・vector・list等を含みます。列は**順序と長さ**を反映します。std::arrayはtupleと同じ扱いで、vectorと同じハッシュである保証はありません。charは符号に依存せずバイト値0〜255として扱い、signed charは符号付き整数です。C配列は末尾NULも要素として数え、C文字列ポインタ・浮動小数点・任意のユーザー型には対応しません。独自型はHashMapに独自Hashを渡してください。

同じ型・内容・seedなら同じhash64を返します。size_tが32bitならoperator()は下位32bitに切り詰めます。default seedは乱数源の実装に依存し、生成時に例外が起こり得ます。再現性が必要ならseedを記録してください。ハッシュ一致はキーの一致の証明にはなりません。暗号学的安全性・衝突の不在・敵対入力に対する最悪計算量を保証するものではありません。

整数1個はO(1)、複合キーは含まれる全要素数Kに対しO(K)、ネスト深さDのスタック領域O(D)。入力のコピーはしません。

## 使用する問題と検証

頻度表・複合状態のメモ化・グリッド座標の辞書。独立したPythonの64bit演算で整数／128bit／pair／列／バイト列を照合します。

`python3 verify/run_foundation_structures.py` / `--sanitize`（`make verify-foundation-structures`）で単独／重複include、全カテゴリとの混在・複数翻訳単位、Debug/Release、使用例・差分比較を検証します。オンライン提出・ACは未確認です。

[追加範囲と検証](../foundation_structures/README.md) / [参考資料](../foundation_structures/references.md) / [カテゴリ索引](README.md) / [操作できる図解](../../visualizations/foundation_structures/index.html#seeded_hash) / [図解の範囲](../../visualizations/foundation_structures/README.md)
