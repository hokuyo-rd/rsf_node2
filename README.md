# rsf_node2

自己位置推定センサ **RSF-X001** の ROS 2 ドライバです。
通信プロトコルは「RSF-X001 通信仕様書」(C-42-04636) に基づきます。

ROS 2 パッケージ名は `rsf_node2` です（リポジトリのディレクトリ名 `hokuyo_rsf` とは独立しています。
colcon は `package.xml` の名前を見るため、ディレクトリ名は変更しなくても動作します）。

## 構成

```
hokuyo_rsf/                 リポジトリ
├── package.xml             ROS 2 パッケージ: rsf_node2
├── rsf_library/            ROS 非依存のネイティブライブラリ + サンプル
│   ├── include/rsf/          プロトコル・復号・通信
│   └── examples/             rsf_driver（受信）/ rsf_mock_sensor（疑似センサ）
├── include/rsf_ros/        ROS 1 / ROS 2 で共有する移植層
│   ├── ros_compat.hpp        ROS 1/2 の差異（タイムスタンプ）を吸収
│   ├── conversions.hpp       rsf の型 → ROS メッセージ（テンプレート）
│   └── driver_config.hpp     パラメータ構造体
├── src/
│   ├── rsf_node.cpp          ROS 2 ノード（rclcpp 依存はここだけ）
│   └── send_command.cpp      コマンド送信 CLI
└── test/
    └── test_conversions.cpp  変換のテスト（ROS 不要）
```

センサ通信・復号は [rsf_library](rsf_library/README.md) が担当し、ROS ノードは
publish に専念します。詳細は [rsf_library/README.md](rsf_library/README.md) を参照してください。

### 移植性

ROS 1 ドライバを作る場合、`include/rsf_ros/` の 3 ファイルと `rsf_library/` は
**そのまま流用できます**。ROS 1 と ROS 2 でメッセージのフィールド名は同一なので、
`conversions.hpp` はメッセージ型をテンプレート引数にして 1 度だけ書いてあります。

共有層のディレクトリと名前空間を `rsf_ros` にしてあるのは、ROS 1 側のパッケージ
（例: `rsf_node1`）からも同じパスと名前空間で include できるようにするためです。

唯一の差異はタイムスタンプで、これは `ros_compat.hpp` が吸収します。

```
ROS 2  builtin_interfaces::msg::Time { int32  sec; uint32 nanosec; }
ROS 1  ros::Time                     { uint32 sec; uint32 nsec;    }
```

新しく書く必要があるのは `src/rsf_node.cpp` に相当する部分（パラメータ読み込み・
publisher 生成・ログ）だけです。`test/test_conversions.cpp` は ROS 2 形式と ROS 1 形式の
両方のタイムスタンプで変換をテストしているので、移植層が両対応であることは検証済みです。

## ビルド

### ROS 2 (humble)

```bash
cd colcon_ws/src
git clone https://github.com/Hokuyo-aut/hokuyo_rsf.git

# nmea_msgs/msg/Gpzda が必要
sudo apt remove ros-humble-nmea-msgs
git clone https://github.com/hokuyo-rd-release/nmea_msgs.git

cd colcon_ws
colcon build --packages-select nmea_msgs
colcon build --symlink-install --packages-select rsf_node2
```

### ROS 無し（Windows / Linux）

ROS 環境が無い場合はネイティブライブラリとサンプルのみビルドされます。

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build -C Release
```

### Docker

```shell
cd hokuyo_rsf/docker
docker build --network host -t hokuyo_rsf:release .
./run.bash -n hokuyo_rsf_release -s /path/to/your/share_folder
~/CONTAINER_NAME.bash
```

## 実行

```shell
# ノードのみ
ros2 launch rsf_node2 rsf_node2.launch.py

# RViz 付き
ros2 launch rsf_node2 rsf_node2_sample.launch.py

# 実機が無いとき: 疑似センサを立てて繋ぐ
ros2 run rsf_node2 rsf_mock_sensor --port 10940 &
ros2 launch rsf_node2 rsf_node2.launch.py

# ROS を介さずに受信状況を見る
ros2 run rsf_node2 rsf_driver --host 192.168.0.100
```

| 実行ファイル | 役割 |
| --- | --- |
| `rsf_node` | ドライバノード本体 |
| `send_command` | コマンド送信 CLI |
| `rsf_driver` | ROS 非依存の受信確認ツール |
| `rsf_mock_sensor` | ROS 非依存の疑似センサ |

## トピック

| トピック | 型 | データタイプ | 頻度 |
| --- | --- | --- | --- |
| `/rsf/nav_sat_fix` | `sensor_msgs/NavSatFix` | FIX | 1 Hz |
| `/rsf/gpgga` | `nmea_msgs/Gpgga` | GGA | 1 Hz |
| `/rsf/gprmc` | `nmea_msgs/Gprmc` | RMC | 1 Hz |
| `/rsf/gpzda` | `nmea_msgs/Gpzda` | ZDA | 1 Hz |
| `/rsf/hokuyo_cloud2` | `sensor_msgs/PointCloud2` | HOKUYO_CLOUD2 | 20 Hz |
| `/rsf/imu` | `sensor_msgs/Imu` | IMU | 1000 Hz |
| `/rsf/lio_imu_rate_odom` | `nav_msgs/Odometry` | LIO_ODOM | 1000 Hz |
| `/rsf/rsf_fix` | `sensor_msgs/NavSatFix` | SWITCH_FIX | 1000 Hz |
| `/rsf/utm_coord_odom` | `nav_msgs/Odometry` | UTM_ODOM | 1000 Hz |
| `/rsf/rsf_odom` | `nav_msgs/Odometry` | SWITCH_ODOM | 1000 Hz |
| `/rsf/rsf_odom_state` | `std_msgs/String` | SWITCH_ODOM_STATE | 1000 Hz |
| `/rsf/rsf_odom_type` | `std_msgs/String` | SWITCH_ODOM_TYPE | 1000 Hz |
| `/rsf/rsf_fix_state` | `std_msgs/String` | SWITCH_FIX_STATE | 1000 Hz |
| `/rsf/rsf_fix_type` | `std_msgs/String` | SWITCH_FIX_TYPE | 1000 Hz |
| `/rsf/diagnostics` | `diagnostic_msgs/DiagnosticArray` | DIAGNOSTICS_ARRAY | 1 Hz |
| `/rsf/lio_lidar_rate_odom` | `nav_msgs/Odometry` | LIO_ODOM を点群レートで再送出 | 20 Hz |

購読: `/rsf/cmd_to_spel` (`std_msgs/UInt8`)

トピック名・フレーム名はすべて [config/rsf_node2.yaml](config/rsf_node2.yaml) で変更できます。

## コマンド送信

ノードを起動してから実行します。

```shell
ros2 run rsf_node2 send_command 1
```

| 番号 | コマンド | 動作 |
| --- | --- | --- |
| 1 | START_STREAMING | データストリーミング開始 |
| 2 | STOP_STREAMING | データストリーミング終了 |
| 3 | START_RSF | 位置推定開始 |
| 4 | STOP_RSF | 位置推定終了 |
| 5 | RESET_RSF | 位置推定のリセット（4 と 3 を続けて送るのと同等） |

既定では接続時に自動で 1 → 3 を送ります（`start_streaming_on_connect` /
`start_rsf_on_connect`）。

## 主なパラメータ

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `ip_address` / `port` | `192.168.0.100` / `10940` | センサの接続先 |
| `wire_layout` | `legacy` | 通信形式。`auto` / `spec` / `legacy` |
| `broadcast_tf` | `true` | TF を送出するか |
| `tf_decimation` | `50` | 何件のオドメトリごとに TF を 1 回送るか |
| `queue_size` | `100` | publisher のキュー長 |
| `publish_lidar_rate_odom` | `true` | 点群レートでのオドメトリ再送出 |
| `enable_set_ip_address` | `false` | IP 設定コマンド（仕様書外の拡張） |

`wire_layout` は仕様書と実機ファームウェアで通信形式が食い違う箇所への対応です。
既定の `legacy` は従来のファームウェアの形式に固定します。`auto` にすると受信フレームから
自動判定しますが、通信異常を別形式として解釈する余地が残るため、形式が分かっている場合は
固定したままにしてください。詳細は
[rsf_library/README.md](rsf_library/README.md#仕様書と実機ファームウェアの差異について) を参照してください。

## テスト

```bash
colcon test --packages-select rsf_node2
colcon test-result --verbose
```

- `rsf_library_tests` — プロトコル（CRC・復号・符号化・フレーミング・疎通）
- `test_conversions` — ROS メッセージ変換。ROS のメッセージ型に依存せず、
  ROS 2 形式と ROS 1 形式の両方のタイムスタンプで検証します

## 自律走行サンプル

解説：https://sourceforge.net/p/urgnetwork/wiki/rsf_app_info_jp/

ソース：https://github.com/Hokuyo-aut/hokuyo_navigation2
