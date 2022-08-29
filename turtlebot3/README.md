
# turtlebot3 関連ファイル

## launch
- turtlebot3_slam/launch/turtlebot3_slam_VPS.launch  
VPSを利用してマップを生成する際のlaunch ファイル(仮)  

- turtlebot3_navigation/launch/turtlebot3_navigation_test.launch  
acmlの代わりにVPSを使用する場合の launch ファイル  
(オリジナルから /amcl ノードを削除)  

- turtlebot3_navigation/param/  
上記VPS版で使用するパラメータ群  
VPSに合わせて、global_cost_map更新頻度、タイムスタンプの許容時間を変更済