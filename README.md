# Spresense Sketches

Sony Spresense 向けのスケッチ集です。\
Collection of sketches for Sony Spresense.

------------------------------------------------------------------------

## 概要 / Overview

Spresense の各種機能を試すためのスケッチをまとめたリポジトリです。\
This repository contains sketches for exploring various features of
Spresense.

------------------------------------------------------------------------

## 内容 / Contents

このリポジトリには、Spresense
の機能を試すためのスケッチが複数含まれています。\
This repository contains multiple sketches for testing Spresense
features.

各スケッチはディレクトリ単位で独立しています。\
Each sketch is contained in its own directory.

内容は以下のようなものを含みます。\
The repository includes sketches such as:

-   音声再生・処理\
    Audio playback and processing

-   カメラ入力・画像処理\
    Camera input and image processing

-   センサの読み取り\
    Sensor data acquisition

-   通信処理（シリアルなど）\
    Communication handling (e.g. serial)

-   SDカードなどのストレージ操作\
    Storage access (e.g. SD card)

------------------------------------------------------------------------

## 必要環境 / Requirements

-   Sony Spresense ボード\
    Sony Spresense board

-   Arduino IDE

-   Spresense Arduino コア\
    Spresense Arduino core

-   https://developer.sony.com/spresense/

-   https://github.com/sonydevworld/spresense-arduino-compatible

------------------------------------------------------------------------

## 使い方 / Usage

1.  リポジトリを取得\
    Clone the repository

``` bash
git clone https://github.com/Interested-In-Spresense/sketches.git
```

2.  Arduino IDEで任意のスケッチを開く\
    Open a sketch in Arduino IDE

3.  ボードを選択\
    Select board

SPRESENSE -\> Spresense Main Board

4.  ビルドして書き込み\
    Build and upload

------------------------------------------------------------------------
## ディレクトリ構成 / Directory Structure

このリポジトリは、各スケッチをディレクトリ単位で配置しています。\
Each sketch is organized in its own directory.

------------------------------------------------------------------------

### Audio/

音声再生・処理関連のスケッチ\
Audio playback and processing sketches

-   `4chSampler/`\
    4音同時再生のサンプラー\
    4-voice sampler

-   `4chVariableSpeedPlayer/`\
    4ch対応の可変速プレイヤー\
    4-channel variable speed player

-   `4chVariableSpeedSampler/`\
    可変速対応のサンプラー\
    Variable-speed sampler

-   `VariableSpeedPlayer/`\
    単体の可変速再生サンプル\
    Basic variable speed player

-   `VariableSpeedPlayerWithJoistick/`\
    ジョイスティックで速度制御するプレイヤー\
    Variable speed player controlled by joystick

------------------------------------------------------------------------

### Camera/

カメラ入力・画像処理関連のスケッチ\
Camera input and image processing sketches

-   `ColorDetect/`\
    色検出のサンプル\
    Color detection example

------------------------------------------------------------------------

### MultiCore/

マルチコア機能を利用したスケッチ\
Examples using multi-core features

-   `ManySensor/`\
    複数センサを扱うマルチコアサンプル
    Multi-core example for handling multiple sensors

-   `SensorSync/`\
    センサデータを複数コアで同期処理\
    Sensor synchronization across cores

-   `ShareI2C/`\
    I2Cを複数コアで共有\
    Shared I2C across cores

-   `SpinLockVerification/`\
    スピンロックの動作確認用サンプル
    Spin lock verification example

------------------------------------------------------------------------

### RumiCar/

車両制御のサンプル\
Vehicle control example

------------------------------------------------------------------------

### SoftwareSerial/

ソフトウェアシリアル通信のスケッチ\
Software serial communication examples

-   `evaluation/`\
    SoftwareSerialの評価コード\
    SoftwareSerial evaluation example

------------------------------------------------------------------------

## 注意 / Notes

-   スケッチごとに前提条件が異なります\
    Requirements differ depending on the sketch

-   SDカードや特定のファイル配置が必要な場合があります\
    Some sketches require an SD card or specific file layouts

-   エラーハンドリングは最小限です\
    Error handling is minimal

-   SDKや環境によって動作が変わる可能性があります\
    Behavior may vary depending on SDK version

------------------------------------------------------------------------

## License

LGPL (GNU Lesser General Public License)

