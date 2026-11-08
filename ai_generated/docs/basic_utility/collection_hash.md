---
title: 集合・多重集合ハッシュ
documentation_of: //basic_utility/collection_hash.hpp
---

# 集合・多重集合ハッシュ（U-07）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#collection_hash)

要素の順序に依存しない可換演算でhashを集約し、追加・削除の差分だけを更新します。頻度表を持つことで二重削除や集合の重複追加がfingerprintを壊さないようにします。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/collection_hash.hpp"
int main() {
    cp::MultisetHash<int> a(42), b(42);
    a.insert(1); a.insert(1); a.insert(2); a.erase(1);
    b.insert(2); b.insert(1);
    std::cout << (a.fingerprint() == b.fingerprint()) << ' ' << a.size() << '\n';
    cp::SetHash<int> set(42);
    std::cout << set.insert(1) << ' ' << set.insert(1) << '\n';
}
```

出力:

```text
1 2
1 0
```

## API・使用条件

`SetHash<Key,Compare>` / `MultisetHash<Key,Compare>` を公開します。constructorは `(seed=SeededHash::random_seed(), compare=Compare())`。

- `insert(key)` / `erase(key)` は実際に内容を変えたかをboolで返します。集合の重複追加はfalse。多重集合は1個ずつ追加・削除し、存在しないキーの削除はfalse。
- `size()` は重複込み、`distinct_size()` は異なるキー数、`count(key)` は個数。`clear()` は空に戻します。
- `fingerprint()` は `{seed,size,first,second}`。集合は要素hashのXOR、多重集合は要素hashのmod 2^64の和を2本保持します。集合と多重集合のfingerprintを相互比較する用途ではありません。
- キーはSeededHashが対応する整数・文字列・tuple・列などで、Compareも使えること。Compareで同値なキーは同じhashを持つ必要があります。
- 比較する状態には**同じseed**を指定します。既定の乱数seedを使った別インスタンス同士はそのまま比較できません。seedはfingerprintにも含みます。

衝突の可能性があります。2本の64bit hashを使いますが、既存のSeededHashは厳密な独立ランダム関数ではなく、任意の入力に一律の2^-128上界は主張しません。攻撃者に対する暗号学的安全性もありません。誤判定が許されなければ要素・頻度表を比較してください。多重集合にXORを使わないのは、同一要素2個が必ず打ち消し合うためです。

## 計算量

異なるキー数Dに対して追加・削除・countはO(log(D+1))比較＋キーhash計算。fingerprint・sizeはO(1)、記憶O(D＋キー総量)。

## 使用する問題・検証

並び順を無視した状態の比較や探索状態のキャッシュ。頻度表から逆順に再構築したfingerprintと比較し、重複追加・削除・clear・seed再現性を確認します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
