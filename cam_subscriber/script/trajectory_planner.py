import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA

from ROBO_CMD import ROBO_CMD

class TrajectoryPlanner:

    """
    ロボットを waypoint を通過して移動させるために、
    現在の姿勢(位置、回転)と、waypoint データより、ロボットの動作(旋回、前進、停止)を決定する

    Attributes
    ----------
    __waypoint : float[]
        waypoint データ x と z のみ使用
    __waypoint_index : int
        現在の目標としている waypoint のインデックス
    __th_theta : float
        旋回時に目的の方向との誤差を許容するしきい値(度)
    __th_distance : float
        waypoint までどの程度近づいたら到達したと判断するかの距離(m)
    """

    def __init__(self):
        # 全Waypoint取得 TODO:Jsonファイルから読取り。VPS座標系のままなのでグローバル座標系に変える。
        # self.__waypoint = [
        #         #[2.26, 1.45, 9.360001], 
        #         [2.47, 1.43, 6.66], [2.5, 1.42, 3.79], 
        #         [1.29, 1.44, 2.26], #机手前
        #         [1.40, 1.22, 1.25],
        #         [1.00, 1.43, 1.08],
        #         [-2.04, 1.43, 0.78], 
        #         [-2.84, 1.41, 1.50], 
        #         [-1.56, 1.29, 1.30], # ここへいくとき回転途中でVPS検出失敗
        #         [0.42, 1.3, 1.62], 
        #         [0.54, 1.33, 3.77], 
        #         [0.73 ,1.35, 4.5], 
        #         [2.54, 1.41, 5.23], [2.71, 1.41, 7.86]
        #     ]

        # test waypoint 
        self.__waypoint = [
            [2.10, 0, 2.47],
            [1.75, 0, 1.61],
            [1.05, 0, 1.54],
            [0.13, 0, 1.49],
            [-0.73, 0, 1.32],
            [-1.49, 0, 1.13],
            [-2.37, 0, 0.25],
            [-3.23, 0, 0.91],
            [-1.55, 0, 1.13],
            [-0.73, 0, 1.33],
            [0.13, 0, 1.52],
            [0.38, 0, 3.08], # 壁激突(1個前にいる、パーティションの足にひっかかる)
            [1.23, 0, 3.47] # 上の点からこの点に移動するとき、机に当たる
	    ]

        self.__waypoint_index = 0
        self.__th_theta = 10.0
        self.__th_distance = 0.2
        return

    def create_move_direction_from_waypoints(self, cur_pos, cur_rot_quat):
        print('[PLANNER] run trajectory_planner')
        # 位置測位結果を変換（クォータニオン->オイラー）
        quat = cur_rot_quat
        q = [quat[1], quat[2], quat[3], quat[0]]
        euler = self._quaternion_to_euler_zxy(q)
        # TODO: 原点をグローバル座標系に修正※地図の左上が原点
        v_position = cur_pos

        # Waypoint位置(x, z)
        waypoint = np.array([self.__waypoint[self.__waypoint_index][0], -self.__waypoint[self.__waypoint_index][2]])

        # VPS位置(x, z)
        vps_position = np.array([v_position[0], -v_position[2]])
        print('[PLANNER] vps_position:', vps_position, 'vps_euler:', euler[2])
        print('[PLANNER] waypont:', waypoint, ' waypoint_index:', self.__waypoint_index)
        wp_vec = waypoint - vps_position

        # robo vector
        vps_angle_deg = euler[2]
        robo_vec = (np.cos(math.radians(vps_angle_deg - 90)), np.sin(math.radians(vps_angle_deg - 90)))

        # 進行すべき方向(wp_vec)と現在の向き(robo_vec)の角度を求める
        print('[PLANNER] robo_vec:', robo_vec, ' wp_vec:', wp_vec)
        test_theta = self._vec_angle(wp_vec, robo_vec)
        cross = np.cross(robo_vec, wp_vec)
        print('[PLANNER] 2vec angle(deg):', test_theta, ' cross:', cross)

        # 現在地とWaypoint間の距離r[m]を計測
        distance = np.linalg.norm(wp_vec)
        print('[PLANNER] distance:', distance)

        # r<=thr ならロボットを停止してWaypoint取得に戻る
        if distance <= self.__th_distance:
            print('[PLANNER] ***** The robot has reached waypoint[', str(self.__waypoint_index) + ']:', self.__waypoint[self.__waypoint_index])
            if (len(self.__waypoint) - 1) > self.__waypoint_index:
                self.__waypoint_index = self.__waypoint_index + 1
            return ROBO_CMD.STOP

        # θ > ths ならロボットの向きを変える      
        if math.fabs(test_theta) > self.__th_theta:
            if cross >= 0:
                print('[PLANNER] do left')
                return ROBO_CMD.LEFT
            else:
                print('[PLANNER] do right')
                return ROBO_CMD.RIGHT

        # ロボットの前進を指示. TODO:指定距離を進む
        print('[PLANNER] do forward')
        return ROBO_CMD.FORWARD

    def _vec_angle(self, v1, v2):
        u = v1
        v = v2
        i = np.inner(u, v)
        n = LA.norm(u) * LA.norm(v)
        c = i / n
        a = np.rad2deg(np.arccos(np.clip(c, -1.0, 1.0)))
        return a

    def _quaternion_to_euler_zxy(self, q):
        r = R.from_quat([q[0], q[1], q[2], q[3]])
        return r.as_euler('zxy', degrees=True)

        
