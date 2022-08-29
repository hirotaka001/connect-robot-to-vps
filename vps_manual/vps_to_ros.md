# VPS with NavigationStack 手順

## 前提
VPSの点群より、VPS2DMapを作成済み。
そうでない場合、下記を参照のこと
### VPS点群作成
360度カメラで撮影して、点群を作成する。「VPS地図作成方法」を参照

### VPS2Dマップ作製
点群ファイルより、Unityを使用して2Dマップを生成する。「VPS2DMap」の Readme 参照

## 1. マップファイルをトリミング
Unityから出力した状態だと、障害物の外側が全て黒で塗りつぶされているが  
不要なのでトリミングする。
ソフトは問わないが、フリーソフトなら Gimp で可能  
Gimpでの作業手順
1. 自動選択ツール(左ペインの魔法の杖みたいなアイコン)で背景(黒)を選択
2. メニューより選択>選択範囲の反転を実行。これで地図部分のみ選択される
3. 選択部分をコピー
4. メニューよりファイル>新しい画像(クリップボードから)を実行
5. 地図部分のみの画像が生成されるので、ファイル>エクスポートを実行

## 2. 画像の回転
map_serverには回転の指定はできないので、VPSの値が適用しやすいように回転する  
(必須ではないがVPSと軸を合わせておくと、軸の入れ替えなどをしなくて済む)

## 3. ROSのmap_serverに使用するyamlファイルを作成
yaml template
```
image: /home/rbag/w_dir/ros_map/map_310A_VPS.png
resolution: 0.023000

origin: [-13.6850, -4.2456, 0.000000]
negate: 0
occupied_thresh: 0.45
free_thresh: 0.44
```
この中で、image、resolution, oring を設定する必要あり。  
occupied_thresh と free_thresh は必要であれば変更する。  
項目の詳細は [ROS map_server](http://wiki.ros.org/map_server) のページを参照

#### 3.1 resolution の計算
`calc_ros_resolution.py` で計算可能  
実行する前にファイル内の項目を書き換える。
```
vps_x_cm = 1581.5
vps_z_cm = 510.1
map_x_pixel = 573
map_y_pixel = 187
```
vps_x_cm、vps_z_cm は、UnityでVPS2DMapを生成した際に出力される   
`CloudPointsStats.txt` 内の range を使用する(高さ方向を使わないように注意)  
map_x_pixel、map_y_pixel はトリミング＆回転した画像のサイズを入力  
上記入力後、実行すると resolution が出力されるので、これを yaml に記載する。  
resolution は軸毎に計算されるので2つでるがどちらを使用してもよい。

#### 3.2 origin の位置の計算
VPSの原点をROS座標空間の原点と一致させるために行う
`calc_ros_resolution.py` で計算可能  
Unityから出力した2Dマップ画像上の緑の点がVPSの原点なので、緑の点の座標で下記を更新する
```
o_x_pixel = 58
o_y_pixel = map_y_pixel - 149
```
更新後、スクリプトを実行。
origin_x、origin_y の値を yaml に設定する

#### 3.3 画像パスの更新
環境に応じて作成したVPS2DMap画像のパスを image に記載する

### 4. 動作確認
実際に map_server を起動して rviz で確認する  
turtlebot3であれば navigation_stack を起動すればよい(マップのパスを作成したyamlファイルにすること)
```
 roslaunch turtlebot3_navigation turtlebot3_navigation_test.launch map_file:=$HOME/map.yaml
```

### 5. 調整
原点がズレている場合、originが間違っているので正負が逆になっていないか確認する
ロボ座標が間違っている場合、
