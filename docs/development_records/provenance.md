# 出典・ライセンス・取り込み版の台帳

[一覧](README.md) / [JSON](../../verification/provenance.json) / [実装](../../tools/cp_tools/provenance.py)

```sh
python3 -m tools.cp_tools.provenance --output build/catalog/provenance.json
```

明示的に移植・帰属の記録があるものを一元管理します。ACLのSegtree/LazySegtree、KACTLのManhattan MST、Luzhiledの三重辺連結成分、noshi91のLARSCH、旧FastIO2種とiwiwiのRadix Heap、Boostへの依存を8レコード・10ファイルとして登録しています。コードの帰属コメントと既存のreferences.mdは保持します。

| フィールド | 意味 |
|---|---|
| `id` / `kind` | 安定IDと、`adapted`（移植）、`dependency`（外部依存）、`historical`（旧実装の出典記録） |
| `sources` | 元ファイル・著者等のURL |
| `imported_revision` | 実際に取り込んだ40桁commit。不明ならnull |
| `revision_note` | 版が不明な理由、既存記録の根拠 |
| `reviewed_upstream` | 今回確認した元コードのcommit、固定URL、SHA-256。取り込み版とは別 |
| `license` | 識別子、一次資料の根拠、同梱テキストとそのSHA-256 |
| `local_sha256` | 対応するローカルファイルと現在のSHA-256 |
| `changes` / `reviewed_at` | 主な改変内容と確認日 |

ACLの取り込み版は既存記録の `864245a00b00dd008d1abfdc239618fdb7d139da` です。他の過去の取り込み版は、今回確認した最新側のcommitから推測しません。固定URLは後日の比較用です。旧yosupo FastIOとiwiwi Radix Heapのライセンスは未確認として `NOASSERTION` を記録しています。

このリポジトリ全体に新しいライセンスを設定するものではありません。個別監査を終えていないファイルは `unreviewed` / `NOASSERTION` として生成台帳に残します。外部コードを含まないことや再配布条件を確認済みという意味にはなりません。references.mdにあるアルゴリズムの参考資料と、ソースの取り込み・実行依存を区別してください。

## 変更・追加時

1. 元コードを使う場合は、元ファイルの固定URL、取り込みcommit、ライセンス原文と帰属を確認します。
2. JSONへファイルと変更内容を登録し、ヘッダにも帰属を記載します。過去の版が不明ならnullと理由を記録します。
3. ローカルコードを変更した場合は差分を確認してから `changes` と `local_sha256` を更新します。
4. validatorと関連するC++検証を実行します。

validatorは重複ID/ファイル、不正な版、根拠のない不明版、範囲外パス、消えたファイル、コードとライセンス原文のハッシュ変更、既知の帰属マーカーに対する未登録をエラーにします。ハッシュを自動更新して変更を承認する機能はありません。

自動抽出する `source_hints` はヘッダ内のURLにすぎません。参照URLを持つだけで移植やライセンスが確定するわけではなく、マーカーがないコピーを自動検出するものでもありません。
