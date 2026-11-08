# 一次資料・設計判断

2026-10-20確認。[一覧](README.md)。出典の固定版・ローカルハッシュは [JSON台帳](../../verification/provenance.json) に保存します。

- [GCC libstdc++ Debug Mode: Using](https://gcc.gnu.org/onlinedocs/libstdc++/manual/debug_mode_using.html)：`_GLIBCXX_DEBUG`、翻訳単位間の標準コンテナABI、一部コンテナの検査範囲。
- [ACLの取り込み版LICENSE](https://github.com/atcoder/ac-library/blob/864245a00b00dd008d1abfdc239618fdb7d139da/LICENSE)：既存記録の固定commitとCC0原文。同梱原文は [ACL-LICENSE.txt](../range_query/ACL-LICENSE.txt)。
- [KACTL ManhattanMSTの今回確認版](https://github.com/kth-competitive-programming/kactl/blob/b3548e973f8772010c1ef84e6d427c749733fd2c/content/geometry/ManhattanMST.h)：chilli / Takanori MAEHARA、ファイル単位のCC0表記。
- [Luzhiledの今回確認版LICENSE](https://github.com/ei1333/library/blob/088b4c8b49f85194099305d998e7113dc3d5dd2e/LICENSE)：Unlicense。[同梱コピー](../../verification/licenses/Unlicense.txt)。
- [noshi91/Libraryの今回確認版README](https://github.com/noshi91/Library/blob/991c68f07261b88f1fb90abe1d588844d710ec38/README.md)：CC0表記。取り込み時のcommitは残っていません。
- [Nyaanの今回確認版LICENSE](https://github.com/NyaanNyaan/library/blob/b3981adc80a800b2584980b01821324ea6c77183/LICENSE)：旧FastIOの出典確認用。取り込み版を意味しません。
- [Boost Software License 1.0](https://www.boost.org/LICENSE_1_0.txt)：Boostは外部ヘッダ依存であり、ソースの同梱ではありません。

上流リポジトリのcommitと元コードのSHA-256は公式GitHub API / rawファイルから読み取り取得しました。今回は移植コードを新たに取り込まず、既存のコード・帰属記録を整理しています。元の取り込み版が不明なものやライセンス確認が残るものは未確認の状態を保持しています。
