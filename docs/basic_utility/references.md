# 共通基盤の一次資料と設計判断

3.1〜3.6のA補完の[一次資料・問題例・設計判断](../a_priority/references.md)。各APIの条件と検証はカテゴリ索引から辿れます。

参照日: 2026-10-19。F-01〜F-07用の記録です。既存A/B項目の出典は各個別説明から参照してください。

- [AtCoder Library: Modint API](https://atcoder.github.io/ac-library/production/document_ja/modint.html)：固定／実行時法、ID、逆元条件の設計を参照。今回のAPIは世代による無効値検出、例外、optional逆元、uint32全域の法を明示して独自に実装しています。ACL完全互換ではありません。
- [ACL internal_math.hpp](https://github.com/atcoder/ac-library/blob/master/atcoder/internal_math.hpp)：Barrettの逆数・高位積・補正の原理を確認。公開版はuint64全域を対象にするためq*mも128bitで保持します。[modint.hpp](https://github.com/atcoder/ac-library/blob/master/atcoder/modint.hpp)もAPI比較に使用しました。
- [Montgomery Arithmetic from a Software Perspective](https://eprint.iacr.org/2017/1057)：Montgomery表現・REDCの説明。今回はR=2^64、奇数m<2^63、canonical residueを採用しています。命令セット依存のasmは使用しません。
- [GCC: Integer Overflow Builtins](https://gcc.gnu.org/onlinedocs/gcc/Integer-Overflow-Builtins.html)：符号付きオーバーフローを起こさずに表現可能性を検査するbuiltinの仕様。
- [C11 Committee Draft N1570](https://www.open-std.org/jtc1/sc22/wg14/www/docs/n1570.pdf)：7.21のFILE入出力・7.22.1.3の浮動小数点変換を参照。新FastIOは標準stdioを使用し、所有権・EOF・エラー時の値保持を公開契約として定めています。

2^61−1の積の折り畳み、floor/ceilの余り補正、符号付き極小値をunsigned magnitudeで読み書きする部分は各説明の等式・範囲から実装しました。既存のmod_inverse／extended_gcdを共有し、逆元アルゴリズムを重複実装していません。

今回の新規ヘッダは独自実装で、外部ライブラリのソースを取り込み・同梱していません。ACLの参照先は[CC0](https://github.com/atcoder/ac-library/blob/master/LICENSE)です。上記masterリンクは参照箇所の案内であり、特定commitを取り込んだという意味ではありません。旧FastIOの出典表記とファイルは保持しました。

[追加範囲と検証](foundation.md) / [カテゴリ索引](README.md)
