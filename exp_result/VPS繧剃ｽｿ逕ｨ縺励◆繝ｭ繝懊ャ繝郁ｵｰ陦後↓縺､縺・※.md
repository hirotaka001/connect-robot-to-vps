
# ロボットについて
今回の実験では、[turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) を使用  
またVPSの位置情報を取得するために、カメラを高さ60m程度の箇所に設置している

# VPSについて
静止画より、位置、方向を特定することが可能な位置測位システム  
[参考記事](https://time-space.kddi.com/au-kddi/20210709/3140)  

# 走行方法について
走行については、ROS※ に あるNavigationStack を使用するパターンと waypoint での走行の実験を行った  
NavigationStack を使用するパターンは地図をVPSで作成したものに変更するパターンと、  
自己位置推定をamclからVPSに変更するパターンの2種類行った  
※ ロボットソフトウェアを作成するためのライブラリとツール群
- NavigationStack(amcl) + VPS2Dマップ
- NavigationStack(no amcl) + VPS自己位置 + VPS2Dマップ
- VPS自己位置 + waypoint

# 1. NavigationStack + VPS2Dマップ
NavigationStack には走行経路を決定するための地図が必要となるが、  
その地図にVPSの地図データより生成した2Dマップを使用する

## システム構成
<img src=attachment/NavStack+VPS2DMap.png>

## 機能

### 〇VPS2Dマップ作成機能
VPSの点群データより、2Dマップを生成するツールを開発  
http://kros.sig.kddilabs.jp/robot/vps2dmap

### 〇VPS2Dマップ変換機能
VPS点群から作成したVPS2DマップをNavigationStackで使用するための処理
詳細は下記、現行手動で行う箇所がある  
[vps_to_ros.md](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/vps_manual/vps_to_ros.md)

VPS->ROSへのマップ変換イメージ  
<img src=attachment/vps_ros_map_convert.png>

## 実行方法
launch ファイルの引数の map_file に作成した2Dマップを指定する  
```
roslaunch turtlebot3_navigation turtlebot3_navigation_test.launch map_file:=$HOME/w_dir/ros_map/map_310A_VPS_0523_tb3.yaml
```

## 走行方法
rviz 上の「2D Nav Goal」よりゴールの位置、方向を指定すると走行を開始する  
<img src=attachment/2dnav.png width=300>

# 2. NavigationStack(no amcl) + VPS自己位置 + VPS2Dマップ
NavigationStackの自己位置推定(amcl)を使用せず、かわりにVPSの自己位置を使用する走行方法

## システム構成

<img src=attachment/NavStack+VPS.png>

## 仕様
カメラ画像をVPSへPOST、レスポンスで自己位置を取得、自己位置情報を  
VPS座標系からROS座標系へ変換、NavigationStackで使用できるように /tf として出力する  
また、自己位置情報は Grafana が使用する DB へ格納され、  
遠隔ユーザがダッシュボードを通してロボット位置を確認することができる

## 機能

### 〇turtlebot3 のカメラ映像を送信する機能
turtlebot3 にカメラを取付、その映像を送信する
送信は ROS の topic を使用して publish する  
gitlab: [cam_subscriber.py L153](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/cam_subscriber.py#L153)

### 〇VPS2Dマップ作成＆設定
1 と同じ機能

### 〇カメラ映像を受け取り、VPSサーバにPOSTする機能
サポートPC上に、turtlebot3 からカメラ映像を subscribe ＆ VPS へ POST する ROS ノードを追加する  
カメラ映像を受信した際の callback でそのまま VPSクライアントへ通知 へ VPSクライアントからPOST する  
受信画像全てを送信すると、VPSの処理速度に比べて頻度が多すぎるので、間引き処理が含まれる  
またVPSクライアントは別スレッドで処理される  
gitlab: [vps_client.py](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/vps_client.py)

### 〇MQTTを使用するためのクライアント機能
MQTTブローカはサポートPC外にあるので、そこへ接続してMQTTメッセージを送信を行うためのクライアント 
前述のROSノード内に実装する  
gitlab: [cam_subscriber.py L180](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/cam_subscriber.py#L180) 

### 〇VPSのレスポンスを処理、DBへ格納する機能
外部からの位置情報確認のために [Grafana](https://grafana.com/) を使用したい。
Grafana に表示するための、DB へデータを格納する。  
データのやり取りは、MQTT を使用する  
gitlab: [mqtt_sub_insert.py](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/insert_vps_result/mqtt_sub_insert_db.py)
```
PostgreSQL情報
データベース名: rbag
テーブル名: robot_pose
データ構造：
 Column |           Type           | Collation | Nullable |                Default
--------+--------------------------+-----------+----------+----------------------------------------
 id     | integer                  |           | not null | nextval('robot_pose_id_seq'::regclass)
 time   | timestamp with time zone |           | not null | now()
 posx   | double precision         |           | not null |
 posy   | double precision         |           | not null |
 posz   | double precision         |           | not null |
 rotx   | double precision         |           | not null |
 roty   | double precision         |           | not null |
 rotz   | double precision         |           | not null |
 rotw   | double precision         |           | not null |

```
### 〇VPSのレスポンスを処理、座標変換ノードへ通知する機能
ROSトピックとして、VPSの自己位置情報を publish する  
gitlab: [cam_subscriber.py L165](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/cam_subscriber.py#L165)

### 〇VPS座標->ROS座標変換機能
subscribe した VPS の自己位置情報を ROS座標系に変換する。  
原点、XYZ軸などを変換する。VPSの原点や座標軸はVPS地図作成時に決定されるので、  
変換は地図ごとに異なるので注意  
gitlab: [cam_subscriber.py L111](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/cam_subscriber.py#L111)  

### 〇ROS座標での自己位置情報をNavigationStackが使用できるように加工する機能
NavigationStack が使用する /tf はある座標系から別の座標系への変換を示す。  
amcl の[実装](https://github.com/ros-planning/navigation/blob/6e9de3f16c76329fc8d218189d90e4ebe51d61c2/amcl/src/amcl_node.cpp#L1446)を参考にVPSから取得した自己位置を変換して、/tf(/map->/odom)を出力する
gitlab: [tf_vps_position.cpp](https://github.com/hirotaka001/connect-robot-to-vps/blob/master/tf_vps_position/src/tf_vps_position.cpp)  
<img src=attachment/tf_nav.png width=400> 

## 実行&走行方法
### 概要
NavigationStackをVPSを組み合わせて自律走行する手順  
amcl の代わりに VPS を位置情報として使用する。

### 端末情報
| 端末         | IP            | 詳細                   |
| ------------ | ------------- | ---------------------- |
| Turtlebot3   | 192.168.11.11 | 走行ロボ               |
| miniPC       | 192.168.11.5  |                        |
| 通信M        | 192.168.11.2  | ロボへ走行コマンド送信 |
| VPSサーバ    | 172.19.73.224 |                        |
| MQTTブローカ | 172.19.73.190 | 仮PF                   |

### VPSについて
VPSサーバは起動済みで、APIが使用できる状態であること

### ロボ準備
**実験後は必ずバッテリーを外す事**
1. Turtlebot3 のバッテリーを充電する
2. Turtlebot3 をバッテリー駆動に切り替える
3. PCから ssh pi@192.168.11.11 で接続できることを確認  
(稀に繋がらないことがあるので、その場合 OpenCR の電源スイッチをON/OFF で再起動する)  
※ OpenCR は ラズパイ の下にあるボードのこと

### 手順
各種端末に ssh でログインして操作する必要があるが、ここでは miniPC からの操作を想定している  
4つのターミナルを使用し、Terminal1から順に実施すること

#### 1.Terminal1(Turtlebot3)
roscore & turtlebot3基本ノード & カメラノードを起動する  
※Lidarを繋いでいない場合、エラーがでるが無視する
1. ssh pi@192.168.11.11 <!-- turtlebot -->
2. ping 192.168.11.2 (通信Mへpingを打って、疎通確認)
3. cd catkin_ws
4. ./tb3-bringup.sh

#### 2.Terminal2(VPSClientを含むROSノード)
1. cd ~/catkin_ws/src/cam_subscriber/script
2. rosrun cam_subscriber cam_subscriber.py   

このノードは下記の機能を実行する。
- Turtlebot3カメラ画像をSubscribe(ROS) 受信画像はウィンドウでも表示
- カメラ画像をVPSへPOST&結果を受け取る(http)
- VPS結果をMQTTでPublish(MQTT)

#### 3.Terminal3(VPSの結果を/tfとして出力)
VPS結果をMQTT経由で取得、/tf を計算してPubする  
```
rosrun tf_vps_position  tf_vps_position_node
```

#### 4.Terminal4(NavigationStack)
amclを起動しないように修正した launch と、VPS点群より作成した2Dマップを指定して起動する  
```
roslaunch turtlebot3_navigation turtlebot3_navigation_test.launch map_file:=$HOME/w_dir/ros_map/map_310A_VPS_0523_tb3.yaml
```

#### 5. rviz
rviz上で 2D Nav goal を指定することで、走行開始。(local_costマップの計算が終わっていない場合、走りださないので待つ)  
![62b95ebd49c4062586b1ed24](https://user-images.githubusercontent.com/47406018/188348859-310819e4-3f5f-4844-9723-4e34f747e499.png)


#### Tips
キーボードで turtlebot3 を操作するコマンド、ナビで問題があった場合、ナビを落としてこちらを使用して手動操作する    
起動中は常にコマンドを送るので、ナビ中は使用しないこと
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

# 3. VPS自己位置 + Waypoint走行
あらかじめ走行ルートを決定しておき、その経路をVPSの自己位置情報を使う走行方法

# システム構成
<img src=attachment/Waypoint+VPS.png>

## 仕様
ロボ上のカメラ画像を、VPSサーバへPOST、レスポンスより自己位置を取得する  
取得後、現在の自己位置と次のwaypointの距離、角度を計算して、走行や旋回を通信Mを通して指示する  
また自己位置情報は Grafana の DB へ格納することで、ユーザが遠隔よりダッシュボードより位置を確認することが可能

## 機能
### 〇turtlebot3 のカメラ映像を送信する機能
### 〇カメラ映像を受け取り、VPSサーバにPOSTする機能
### 〇MQTTを使用するためのクライアント機能
### 〇VPSのレスポンスを処理、DBへ格納する機能
上記は全て 2 と同じ機能

### 〇VPSのレスポンスを処理、Waypoint走行機能へ通知する機能
VPSのレスポンスを処理して、自己位置を waypoint 走行機能へキューを使い通知  
gitlab: [cam_subscriber.py L104](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/cam_subscriber.py#L104)

### 〇Waypoint走行機能
別スレッドで稼働。キューよりVPS自己位置を取得、次 waypoint との距離、角度を計算して、
前進もしくは旋回(右、左)を決定する。  
ロボ移動中のカメラのブレを考慮して、前進、旋回を一定時間行った後は、一時停止する機能を含む  
gitlab: [auto_driving_controller.py](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/auto_driving_controller.py)  
gitlab: [trajectory_planner.py](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/trajectory_planner.py)

### 〇通信Mへの走行指示通知機能
走行指示にしたがって、通信M用のコマンドを作成して通知する。
通信には MQTT を使用する  
gitlab: [robo_commander.py](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/cam_subscriber/script/robo_commander.py)

## 実行&走行方法
[Growiに記載](https://growi.sig.kddilabs.jp/user/xta-tadanou/20220317_VPSによるturtlebot3自動走行手順)

# 他
- [VPS地図作成方法](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/vps_manual/VPS%E5%9C%B0%E5%9B%B3%E4%BD%9C%E6%88%90%E6%96%B9%E6%B3%95.md)
- [VPSサーバ起動方法](https://github.com/hirotaka001/connect-robot-to-vps/tree/master/vps_manual/VPS%E3%82%B5%E3%83%BC%E3%83%90%E8%B5%B7%E5%8B%95%E6%96%B9%E6%B3%95.md)
 - VPS送信コマンドサンプル(詳細は 研究所VPS PF利用マニュアル_r0.9.pdf )  
```
curl -X POST -F qimage=@/test.jpg-F “message_id=random_string” –F “FocalLengthIn35mmFilm=28.0”http://vps-load-balancer-123456.ap-northeast-1.elb.amazonaws.com:1234/map_token_id/
```

# 走行実験
VPSを使用した走行方法3つに、オリジナルのNavigationStackを追加して比較走行  
ショートコースとロングコースの2つで検証  
ロングコースに関しては、ゴールできていないパターンが多いので、走行状況のまとめを参照
