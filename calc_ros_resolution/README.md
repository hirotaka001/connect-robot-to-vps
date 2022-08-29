
# About
VPS2DマップをROSのNavigationStackで使用するために必要になる、  
map.yaml ファイルに設定する値を求めるスクリプト


# ROS map yaml
yaml は下記の通り、この中で解像度(resolution) と原点(origin) の位置を計算する  
```
image: /home/rbag/w_dir/ros_map/map_310A_VPS.png
resolution: 0.023000

origin: [-13.6850, -4.2456, 0.000000]
negate: 0
occupied_thresh: 0.45
free_thresh: 0.44
```
occupied_thresh と free_thresh は必要であれば変更する。  
項目の詳細は [ROS map_server](http://wiki.ros.org/map_server) のページを参照  

# How to use
calc_ros_resolution.py を開いて下記項目を書き換える  
```
vps_x_cm = 1581.5
vps_z_cm = 510.1
map_x_pixel = 573
map_y_pixel = 187

o_x_pixel = 58
o_y_pixel = map_y_pixel - 149
```
vps_x_cm、vps_z_cm は、UnityでVPS2DMapを生成した際に出力される  
CloudPointsStats.txt 内の range を使用する(高さ方向を使わないように注意)  
map_x_pixel、map_y_pixel はトリミング＆回転した画像のサイズを入力  
resolution は軸毎に計算されるので2つでるが似たような値なのでどちらを使用してもよい。  

o_x_pixel、o_y_pixel はVPSの原点をROS座標空間の原点と一致させるために行う  
Unityから出力された2Dマップ画像上の緑の点がVPSの原点なので、緑の点の座標で下記を更新する  

変更が終わったら`calc_ros_resolution.py`を実行すると
resolution、および origin の位置が計算される  
