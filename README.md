# 水中翼ボート 制御プログラム保存版

報告書で参照した1日目・2日目の通信側／制御側プログラムを保存します。

**書き込み・プログラムZIPの取得： https://temesotejam.github.io/hydrofoil-boat-control-archive/**

| 保存先 | 対象 | 原典コミット |
| --- | --- | --- |
| `day1/` | 1日目（8月8日） | `bf8067c35ee56c884bfbd16e277b27db6e72ef98` |
| `day2/` | 2日目（8月9日） | `0820d1d1c2107609c839bf6cc5d8c42f9d56e982` |

原典： https://github.com/temesotejam/2026_8_6

各日の`control/`、`communication/`、`tests/`は原典の該当コミットから無変更で収録しています。`archive.json`に原典コミットと全ファイルのSHA-256を記録し、ビルド時に検証します。保存版のため、報告書で指摘した2日目の始動確認の時間設定も修正していません。機器内の操作画面も原典のままです。

## ブラウザから書き込む

1. 上記の公開ページをPC版ChromeまたはEdgeで開きます。
2. 使用する日を選びます。通信側・制御側は同じ日の版を組み合わせます。
3. モータ・サーボの外部電源を切り、対象基板だけをUSB接続します。
4. 基板に対応する書き込みボタンを押し、対象のシリアルポートを選択します。
5. 完了したらもう一方の基板へ差し替え、同じ日の版を書き込みます。

対象はXIAO ESP32S3、8MBフラッシュです。書き込み時に現在のプログラムと設定が消去される場合があります。

## プログラムだけのZIP

公開ページから、各日の「2台分」「制御側だけ」「通信側だけ」を選べます。

- 2台分：`control/`と`communication/`、ビルド案内のREADMEのみ。
- 基板ごと：`platformio.ini`、`include/`、`src/`、`lib/`とREADME。展開したフォルダをPlatformIOで直接開けます。
- サイト、GitHub Actions、他の日のプログラム、テスト、ビルド生成物はZIPに含みません。
- 初回ビルド時にはPlatformIOが依存ライブラリを取得します。

GitHubの「Download ZIP」はリポジトリ全体を取得するため、プログラムだけが必要な場合は公開ページのZIPを使ってください。

## 手元でビルド

PythonとPlatformIOを用意し、リポジトリのルートで実行します。

```sh
python -m pip install platformio==6.1.19
python -m platformio run -d day1/control -e production
python -m platformio run -d day1/communication -e production
python -m platformio run -d day2/control -e production
python -m platformio run -d day2/communication -e production
```

ソースと`platformio.ini`は原典のままです。特に制御側のSparkFun VL53L5CXライブラリは原典で版指定がないため、再ビルドしたバイナリが当日使用したバイナリと完全一致することは保証できません。書き込み用マニフェストには、リポジトリの更新番号ではなく各日の原典コミットを表示します。

## 検証とページ生成

```sh
python scripts/verify_archive.py
python scripts/host_tests.py
python scripts/build_site.py
```

`build_site.py`は4種類のビルド結果から、ブートローダ・パーティション・アプリをまとめたBIN、ESP Web Tools用マニフェスト、6種類のソースZIPとSHA-256を`site/`へ生成します。外部電源を接続した実機での書き込み試験は自動テストには含みません。

ソースZIPとページだけを生成する場合：

```sh
python scripts/build_site.py --sources-only
```

このモードでは書き込みボタンを有効にしません。

`main`更新時のGitHub Actionsで検証・4種類のビルド・GitHub Pages公開を行います。Pagesの公開元にはGitHub Actionsを使用します。

### 原典に残っているテストの不整合

2日目の`tests/controller_test.cpp`は、当該版の`ServoTuning`より古いコンストラクタを呼ぶため、そのままではコンパイルできません。保存対象のソースとテストは変更せず、検証スクリプトではこのテストを合格扱いせずに除外し、制御実装の構文確認と残りのテストを行います。2日目の既存の未使用関数・インデント警告もソースを変更せず表示します。ファームウェア4種類のビルド成否は別に検証します。

2日目の通信側ホスト用スタブも、追加されたSD記録・WebServer・WiFi APIを網羅していません。このホスト構文チェックを合格扱いせずに除外し、実際のArduino/ESP32環境でのビルドを必須とします。
