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
https://growi.sig.kddilabs.jp/user/xta-tadanou/20220317_VPS%E3%81%AB%E3%82%88%E3%82%8Bturtlebot3%E8%87%AA%E5%8B%95%E8%B5%B0%E8%A1%8C%E6%89%8B%E9%A0%86
