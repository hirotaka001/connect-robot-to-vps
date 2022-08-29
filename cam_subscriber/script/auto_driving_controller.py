
import threading
import time
from ROBO_CMD import ROBO_CMD
from trajectory_planner import TrajectoryPlanner

class AutoDrivingController(threading.Thread):

    """
    ロボットの自律走行を計画するクラス
    VPSの結果を受け取り TrajectoryPlanner を使用して、旋回、前進、停止を決定する
    モードは WAIT→MOVE→STOP→WAIT を繰り返し、VPSの結果が取得できない時 RECOVERY となる
    移動時の時間や、停止後次の結果を利用するまでの待機時間をパラメータで指定可能	

    Attributes
    ----------
    __mode : int
        ロボットの状態(MOVE, STOP, WAIT, RECOVERY)
    __move_duration_sec : float
        移動コマンドを実行した際の持続時間(秒)
    __move_stop_duration_sec : float
        停止コマンドを実行した後の次のコマンドを実行するまでの待機時間(秒) 
    __time_started : float
        時間計測のための変数
    __time_stopped : float  
        時間計測のための変数 
    """

    MODE_MOVE = 1
    MODE_STOP = 2
    MODE_WAIT = 3
    MODE_RECOVERY = 4

    def __init__(self):
        super(AutoDrivingController, self).__init__()
        self.__planner = TrajectoryPlanner()

        self.__mode = self.MODE_WAIT
        self.__move_duration_sec = 0.5
        self.__stop_duration_sec = 2.0
        self.__time_started = 0
        self.__time_stopped = 0
        self.__fail_count = 0
        self.__work_thread = None
        return

    def prepare(self, vps_result_queue, callback, user_data):
        print('driving thread start!')
        self.__work_thread = threading.Thread(target=self._driving_worker, args=(vps_result_queue, callback, user_data))
        self.__work_thread.setDaemon(True)
        
    def start(self):
        self.__work_thread.start()

    """
        VPSの結果を受け取り、ロボの挙動を決定するスレッド

        Parameters
        ----------
        vps_result_queue : Queue
            VPSのレスポンスが格納されたキュー

        callback : function(ROBO_CMD, user_data)
            コールバックメソッド、決定した ROBO_CMD と user_data を引数にとる

        user_data : 
            ユーザデータ
    """
    def _driving_worker(self, vps_result_queue, callback, user_data):

        while 1:
            vps_result = None
            pos = quat = 0
            status = -1 # -1 はそもそも結果がきていない場合
            if vps_result_queue.qsize() != 0:
                vps_result = vps_result_queue.get()                
                status = vps_result['status']
                if status == 1:
                    pos = vps_result['position']
                    quat = vps_result['rotation']
            
            # 指定された回数VPSの結果が取得できない場合、RECOVERYモードに移行する
            if status == 0:
                self.__fail_count += 1
                if self.__fail_count > 3:
                    self.__mode = self.MODE_RECOVERY
                    print('fail count over. goto recovery mode')
                else:
                    continue
            elif status == 1:
                if self.__mode == self.MODE_RECOVERY:
                    self.__mode = self.MODE_WAIT
                self.__fail_count = 0

            if self.__mode == self.MODE_WAIT and vps_result is not None:
                mv_direction = self.__planner.create_move_direction_from_waypoints(pos, quat)
                self.__time_started = time.time()
                self.__mode = self.MODE_MOVE
                if mv_direction == ROBO_CMD.LEFT or mv_direction == ROBO_CMD.RIGHT:
                    # 旋回時の移動、待機時間設定
                    self.__move_duration_sec = 0.5
                    self.__stop_duration_sec = 1.0
                else:
                    self.__move_duration_sec = 2.0
                    self.__stop_duration_sec = 2.0
                callback(mv_direction, user_data)
            elif self.__mode == self.MODE_MOVE:
                cur_time = time.time()
                spent = cur_time - self.__time_started
                if spent > self.__move_duration_sec: 
                    self.__time_stopped = time.time()
                    self.__mode = self.MODE_STOP
                    callback(ROBO_CMD.STOP, user_data)
                else:
                    callback(ROBO_CMD.NONE, user_data)
            elif self.__mode == self.MODE_STOP:
                cur_time = time.time()
                spent = cur_time - self.__time_stopped
                if spent > self.__stop_duration_sec: 
                    self.__mode = self.MODE_WAIT
                callback(ROBO_CMD.NONE, user_data)
            elif self.__mode == self.MODE_RECOVERY:
                self.__time_started = time.time()
                self.__mode = self.MODE_MOVE
                self.__move_duration_sec = 1.5
                self.__stop_duration_sec = 1.0
                callback(ROBO_CMD.LEFT, user_data)

        return