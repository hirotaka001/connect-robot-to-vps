
# VPS2D マップを ROS Navigation Stack の map_server へ登録する際の
# yaml ファイルに記載する値を計算するスクリプト

# vps_x_cm --- Unity上で得られる点群のX軸方向の範囲(cm)
# vps_z_cm --- Unity上で得られる点群のZ軸方向の範囲(cm)
# map_x_pixel --- Unityから出力した2Dマップの画素数(横方向)
# map_y_pixel --- Unityから出力した2Dマップの画素数(縦方向)

# マップ画像はROSでUnity上と同じ方向で表示できるように回転してから計算すること

# 310A 0523 walk
vps_x_cm = 1581.5
vps_z_cm = 510.1
map_x_pixel = 573
map_y_pixel = 187

# 旧310A
# vps_x_cm = 1049.5
# vps_z_cm = 1565.8
# map_x_pixel = 452
# map_y_pixel = 678

o_x_pixel = 58
o_y_pixel = map_y_pixel - 149

def main():
	vps_x_m = vps_x_cm / 100
	vps_z_m = vps_z_cm / 100
	resolution_x = vps_x_m / map_x_pixel
	resolution_y = vps_z_m / map_y_pixel
	print('resolution_x:', resolution_x)
	print('resolution_y:', resolution_y)

	o_x_m = o_x_pixel * resolution_x
	o_y_m = o_y_pixel * resolution_y 
	print('origin_x_m:', o_x_m)
	print('origin_y_m:', o_y_m)



if __name__ == "__main__":
    main()