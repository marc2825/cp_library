---
title: Open Addressing Hash Map
documentation_of: //basic_utility/hash_map.hpp
---

# Open Addressing Hash Map

線形探索によるopen addressingのHash Mapです。連続したslot配列に要素を置き、空・使用中・削除済みの3状態を区別します。hash値が衝突してもEqualでキーを比較し、正しさを保ちます。

## 使用例

```cpp
#include <iostream>
#include <utility>
#include "basic_utility/hash_map.hpp"
int main() {
    cp::HashMap<std::pair<int, int>, int> count(cp::SeededHash(42));
    count[{2, 3}] = 7;
    auto result = count.try_emplace({2, 3}, 9);
    std::cout << *result.first << " " << result.second << "\n";
    count.erase({2, 3});
    std::cout << count.contains({2, 3}) << "\n";
}
```

出力:

```text
7 0
0
```

## API・使用条件・計算量

`HashMap<Key,T,Hash=SeededHash,Equal=std::equal_to<Key>>`。コンストラクタは`HashMap(Hash hash=Hash{}, Equal equal=Equal{})`。キーと値はコピー構築可能、Hash/Equalはコピー構築可能かつnothrow swap可能であることを要求します。Tの既定構築はoperator[]を使うときだけ必要です。Hashはconst呼び出しで同じキーに同じ値を返し、Equalで等しいキー同士のhashが一致する必要があります。

| API | 戻り値・規約 |
|---|---|
| `find(key)` | T*（const版はconst T*）。不在はnullptr。挿入しない |
| `contains(key)` | 存在判定 |
| `at(key)` | T&／const T&。不在はout_of_range |
| `try_emplace(key,args...)` | `{T*,bool inserted}`。既存なら値の構築を省略 |
| `insert(key,value)` | 既存値を保持し、同じ結果型を返す |
| `insert_or_assign(key,value)` | 既存なら代入。boolは新規挿入かどうか |
| `operator[](key)` | 不在ならTを既定構築し、T&を返す |
| `erase(key)` | 削除できたらtrue。削除済みslotは探索を打ち切らない |
| `size()/empty()/capacity()/load_factor()` | 生存要素数／空判定／slot数／生存要素数÷slot数 |
| `reserve(n)` | n要素まで生存負荷率1/2以内になる容量を確保 |
| `rehash(slots)` | 必要な容量へ再構築して削除印を除去。2冪・最低8・生存数の2倍以上 |
| `clear()` | 値を破棄し、容量を保持 |
| `for_each(callback)` | callback(const Key&,T&)で全要素を走査。const版の値はconst T& |
| `hash_function()` | ハッシュ関数のコピー。既定Hashならseed()でseedを取得 |
| コピー／ムーブ／swap | seed・内容を保持。ムーブ元は空で再利用可能 |

初期容量は0。生存負荷率が1/2を超える前に拡張し、使用済みslot（削除印を含む）が容量の3/4に達したら挿入前に再構築します。size_t／vector容量を超えるreserve/rehashはlength_error、実際のメモリ不足はbad_allocです。

**rehashは全ポインタ・参照を無効化します。** 新規挿入による自動rehashも含みます。eraseはその値への参照、clear・代入は旧内容への参照を無効化します。for_each中に挿入・削除・clear等で構造を変更しないでください。列挙順は整列順ではなく、同じ操作列・容量操作・seedに対して再現するslot順です。std::unordered_mapのiterator APIとの互換はありません。

rehashは別配列へコピーしてから交換するため、コピー失敗時に元の内容を保ちます。新規挿入の候補はrehash前に構築し、既存値をコピー元の引数として渡せます。構築失敗時に内容は保持されますが、先にrehashが成功していれば容量・参照は変わり得ます。既存値を明示的にmoveして引数にする場合、その値の変更はこの保証に含めません。既存値への代入・callbackが投げる場合は、その型／callbackの例外保証に従います。

キーのhash・比較・コピーをO(1)としたとき、良好に分散するhashなら検索・更新は期待償却O(1)、最悪は容量Cに対してO(C)。rehashは期待O(C_old+C_new)、全キー衝突時はO(N²+C_old+C_new)。clear/for_eachはO(C)。領域O(C)にキー・値が所有する領域を加えます。seed付きhashは最悪計算量を消しません。

## 使用する問題と検証

[Library Checker Associative Array](https://judge.yosupo.jp/problem/associative_array)。std::mapとの6万操作比較、全キーが衝突するHash、削除後の探索、コピー例外、20万要素を検証します。

`python3 verify/run_foundation_structures.py` / `--sanitize`（`make verify-foundation-structures`）で単独／重複include、全カテゴリとの混在・複数翻訳単位、Debug/Release、使用例・差分比較を検証します。オンライン提出・ACは未確認です。

[追加範囲と検証](../foundation_structures/README.md) / [参考資料](../foundation_structures/references.md) / [カテゴリ索引](README.md) / [操作できる図解](../../visualizations/foundation_structures/index.html#hash_map) / [図解の範囲](../../visualizations/foundation_structures/README.md)
