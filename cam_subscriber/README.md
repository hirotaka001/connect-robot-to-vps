# cam_subscriber
ロボの画像をVPSへPOST、結果をROSトピック(/vps_pose)へ流すROSノード

## ビルド方法
ベースはROSノードなので、  
`catkin_ws/src` 配下に配置後、`catkin_make` を実行  
今のとこ実装は全て python なので一度make後はビルドする必要はない 

## 実行方法
```
cd ~/catkin_ws/src/cam_subscriber/script
rosrun cam_subscriber cam_subscriber.py
```

## オプション  
rosparam で静止画入力に切り替えることが可能  
```
rosparam set use_test_img true
```

## 自律走行
ロボのカメラ映像が表示されているウィンドウをアクティブにした状態で、  
キーボードの「s」を押下

## Topic
### Publish
/vps_pose --- VPSの位置推定結果
### Subscribe
/usb_cam/image_raw --- ロボットのカメラ画像トピック  

## ファイル
**auto_driving_controller.py**  
自律走行を管理する。trajectory_planner の上位クラス  

**cam_subscribe.py**  
メインスクリプト。各種機能を実行する  

**mqtt_publisher.py**  
MQTTトピックをパブリッシュする  

**ROBO_CMD.py**  
ロボットへのコマンド一覧    

**robo_commander.py**  
通信M用のロボット操作コマンドを作成する  

**trajectory_planner.py**  
Waypointの管理、走行方向、旋回、直進の決定を行う  

**vps_client.py**  
VPSクライアント、画像をサーバへPOSTし結果を受け取る  

**config.ini**  
サーバのIPなど各種設定

