# Communication Module Experimental tool

# calc_camera_param
VPSで使用するWebカメラの内部パラメータを求めるスクリプト

# cam_subscriber
ロボットのカメラ映像を取得するためのROSノード  、追加で下記の機能を持つ  
・取得した画像をVPSサーバへPOST (VPSクライアント機能)  
・VPSの結果をMQTTでパブリッシュ  (トピック名 vps_result)  
・予め指定しておいたWaypointへのロボットの走行指示 (走行指示は通信Mを介して実施)

# grafana-plugin-robopose
VPSの結果を Grafana 上で表示するための Grafana プラグイン  
表示用データは insert_vps_result でデータベースで格納する

# insert_vps_result
cam_subscriber がパブリッシュするVPSの結果を  
サブスクライブしてデータベースへ登録するスクリプト

# tf_vps_position
VPS 結果を tf(/map->/odom) へパブリッシュする  
/amcl の代わりに VPS 座標を使用して Navigation Stack を実行する際に使用する 

# turtlebot3
マップなしでの move_base を実行するための launch ファイル等  
NavigationStack に VPS を使用する場合の param ファイルなど

# vps_manual
VPSサーバの起動方法、360度映像からVPS地図の作成方法、VPS地図から2D地図の作成方法のドキュメントなど

# vps2dmap
VPSの点群データから、2Dマップを生成するための Unity プロジェクト

# VPSを利用したWaypoint自律走行手順
端末情報  

端末	IP	詳細
Turtlebot3	192.168.11.11	走行ロボ
miniPC	192.168.11.5	
通信M	192.168.11.2	ロボへ走行コマンド送信
VPSサーバ	172.19.73.224	
VPSについて  
VPSサーバは起動済みで、APIが使用できる状態であること

ロボ準備  
実験後は必ずバッテリーを外す事

Turtlebot3 のバッテリーを充電する
Turtlebot3 をバッテリー駆動に切り替える
PCから ssh pi@192.168.11.11 で接続できることを確認
(稀に繋がらないことがあるので、その場合 OpenCR の電源スイッチをON/OFF で再起動する)
※ OpenCR は ラズパイ の下にあるボードのこと
Waypointについて  
cam_subscriber/script/trajectory_planner.pyにハードコーディングしてある
ルートを変更したい場合

各waypoint上でVPSの結果を取得
取得したVPS座標を trajectory_planner.py の配列に格納する
手順  
各種端末に ssh でログインして操作する必要があるが、ここでは miniPC からの操作を想定している
4つのターミナルを使用し、Terminak1から順に実施すること

1.Terminal1(Turtlebot3)  
roscore & turtlebot3基本ノード & カメラノードを起動する
※Lidarを繋いでいない場合、エラーがでるが無視する

ssh pi@192.168.11.11
ping 192.168.11.2 (通信Mへpingを打って、疎通確認)
cd catkin_ws
./tb3-bringup.sh
2.Terminal2(通信M)  
ssh pi@192.168.11.2
source ~/.bashrc
sh ros_catkin_ws/src/connect-robot-to-platform/scripts/launch_dongle.sh
Tips1 コントローラから1回目のコマンドを受けた後エラーになることが多い。
その場合 kill_dongle.sh 実行後、再度 launch_dongle.sh すること
その他、調子がおかしくなったら kill_dongle.sh → launch_dongle.sh を実行する
Tips2 Turtlebot3のroscoreが起動しているにも関わらず、ROSマスタに繋がらないエラーが表示されることがある。
その場合、Turtlebot3のターミナルから ping 192.168.11.2 を実行することで繋がるようになる
3.Terminal3(VPS結果をGrafanaに反映)  
VPSの結果をSubscribe＆GrafanaのDBへ格納するスクリプト
なお、このスクリプトを起動する時、DBのデータは一旦すべて消去される

cd ~/w_dir/communication-module-experimental-tool/insert_vps_result
python3 mqtt_sub_insert_db.py vps_result
4.ブラウザ(Grafana起動)  
ロボ位置確認用

http://172.19.73.134:3000 を開く ユーザ名は 「admin」
ページ左上の「General」をクリック
![image](https://user-images.githubusercontent.com/47406018/187331021-48c93a86-2f24-4a0e-a1ba-0920de20a691.png)

「Dashboard for my plugin」をクリック
![image](https://user-images.githubusercontent.com/47406018/187331038-3bdcc4bc-e780-4d1c-ac5e-bec3363451c1.png)

ページ右上のデータ表示期間を「Last 1 hour」、データ更新感覚を「1s」に設定」
![image](https://user-images.githubusercontent.com/47406018/187331053-d7aaac68-bb2c-4b85-adb4-46d2f5f4cfe1.png)

Tips Grafanaのダッシュボードにアクセスできない場合、サービスが起動しているか確認する
再起動後などは実行する必要あり
ステータス確認

sudo systemctl status grafana-server.service
サービス起動

sudo systemctl start grafana-server.service
5.Turtlebot3 をスタート位置に移動  
waypoint のスタート地点へロボを移動させる

6.Terminal4(ロボカメラ受信＆VPS実行等)  
cd ~/catkin_ws/src/cam_subscriber/script
rosrun cam_subscriber cam_subscriber.py
カメラ映像のウィンドウをアクティブにした状態でキーボードの「s」を押下して自律走行を開始
このノードは下記の機能を実行します。

Turtlebot3カメラ画像をSubscribe(ROS) 受信画像はウィンドウでも表示
カメラ画像をVPSへPOST&結果を受け取る(http)
VPS結果をMQTTでPublish(MQTT)
VPS結果を受けて、あらかじめ指定してあるWaypointへ自動走行
Tips:停止させる時はロボットに「stop」命令が送られてきた後に、Ctrl−Cで止めること
forward の時に止めると、ロボットが前進し続ける

仮PF  
コントローラ操作を行う場合、下記を実行する。
コントローラに使用するパッケージの関係で、仮PFで直接実行する必要がある(SSH経由ではエラー)
コントローラを使用しない場合、この項目は必要ない

python remote_control.py cmd_to_robot turlteobt3
