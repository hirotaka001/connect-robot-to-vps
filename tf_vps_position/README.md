# tf_vps_position

ROSノード  
/vps_position トピックから VPS の結果を受け取り、  
/tf(/map->/ocom)へ流す値を計算する  
また、VPS位置をrvizへ表示するためのトピック(vps_pose_marker)をPublish

## build
`catkin_ws/src` に配置後、`catkin_make`

## how to use
```
rosrucn tf_vps_position tf_vps_position_node
```
## mode
/tf を流す頻度を変更可能  
0 で運用中(2022/05/26現在)  
```
0 --- VPS結果をそのままPub  
1 --- 間引く。3回に1回Pub  
2 --- /odom に一定以上(10cm)の移動があった場合にPub  
3 --- 平均をとる(4回分)  
```
mode は rosparam で変更する  
```
rosparam set tf_vps_position_mode 1
```

## subscribe topic 
/vps_position  
/odom

## publish topic
/tf (/map->/odom)  
/vps_pose_marker
