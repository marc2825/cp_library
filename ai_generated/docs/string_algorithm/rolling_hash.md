---
title: Rolling Hash（二重ハッシュ）
documentation_of: //string_algorithm/rolling_hash.hpp
---

# Rolling Hash（二重ハッシュ）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#hash) / [実装](../../string_algorithm/rolling_hash.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/rolling_hash.hpp"
int main() {
    cp::RollingHash a("abracadabra"), b("abrax");
    std::cout << a.equal(a, 0, 4, 7, 11) << '\n';
    std::cout << a.lcp(b, 0, 0) << '\n';
    std::cout << (a.concat(a.get(0, 4), a.get(4, 7), 3) == a.get(0, 7)) << '\n';
}
```

[動く使用例](../../examples/string_algorithm/rolling_hash.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`RollingHash(string_view s={},uint32_t base=911382323)`。2つの素数法1000000007 / 1000000009で多項式ハッシュを前計算します。baseは257〜1000000006、違反はinvalid_argument。既定baseは固定で、実行ごとの乱数選択はしません。

`Hash`はarray<uint32_t,2>、`moduli` / `default_base`は公開定数です。バイトはunsigned charへ変換して**byte+1**を係数にします。空のハッシュは{0,0}。入力文字列そのものは保持せず、構築後の入力の寿命に依存しません。

| API | 意味 |
|---|---|
| size() / base() | 入力長と使用したbase |
| get(l,r) | [l,r)のハッシュ、空区間も可 |
| hash_string(s) | 同じbaseで別のバイト列のハッシュを計算 |
| concat(left_hash,right_hash,right_length) | leftの後ろへrightを連結したハッシュ |
| equal(other,l,r,other_l,other_r) | 長さと二重ハッシュが両方一致するか |
| lcp(other,l,r,other_l,other_r) | 2つの指定区間の共通接頭辞長を二分探索 |
| lcp(other,l,other_l) | 2つのsuffixの共通接頭辞長 |

区間違反はout_of_range。オブジェクトを比較するequal/lcpはbaseが違えば、空区間同士でもinvalid_argument。raw Hashには長さ・baseを格納しないので、concatには同じbaseで作った値と正しい右側の長さを渡してください。right_lengthは0〜size()、正規化されていない剰余はinvalid_argumentです。

**ハッシュ一致は文字列一致を保証しません。** 二重でも衝突はあり、equalや二分探索のlcpが誤る可能性があります。固定baseに対して単純な独立一様の衝突確率は保証しません。厳密な一致検索にはKMP、厳密な辞書順やLCPにはSuffix Array / LCP Arrayを使えます。

旧グローバルRollingHash<Str>はリポジトリ直下の `depricated/string/rolling_hash.hpp` に保存した別APIです。今回のcp::RollingHashは独立・自己完結した新実装で、旧61ビット法の数値とは互換ではありません。

## 原理・計算量

H[i+1]=H[i]*base+(byte+1)とbase^iを前計算し、get(l,r)=H[r]−H[l]*base^(r−l)を各法で求めます。各法の積はuint64_tに収まり、非標準の128ビット整数を使いません。

構築O(n)時間・O(n)領域、get/concat/equalはO(1)。hash_stringは対象長に比例、lcpはO(log(min(区間長)+1))です。前計算は4個のuint32_t値／位置で、配列本体は約16(n+1)バイトです。

## 使用する問題・検証範囲

[AOJ ALDS1_14_B](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_B)の[ラッパ](../../verify/string_algorithm/rolling_hash.test.cpp)をローカルで比較しています。これは衝突がないことの証明ではありません。各部分列の多項式を任意精度整数で定義どおりに計算し、baseの境界・連結・区間LCPを検証します。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。

base=257では、バイト列`0d 81 93 6e fc 1f 73 22`と8個の`00`が二つの法で同時に衝突することも検証しています。係数差の多項式が両法の積に等しくなる例です。
