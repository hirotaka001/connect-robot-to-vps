from ROBO_CMD import ROBO_CMD

class RoboCommandCreator:
    """
    ロボットへ通知する移動コマンドを生成する基底クラス
    """

    def __init__(self):
        return

    def create_move_cmd(self, cmd):
        return

    def forward(self):
        return
    
    def backward(self):
        return

    def left(self):
        return

    def right(self):
        return

    def stop(self):
        return


class Turtlebot3CommandCreateor(RoboCommandCreator):
    """
    turtlebot3 へ通知する移動コマンドを生成するクラス

    Attributes
    ----------
    __robot_name : string
        ロボット名
    """

    def __init__(self):
        super().__init__()
        self.__robot_name = 'turtlebot3'
        return

    def create_move_cmd(self, cmd):
        if cmd == ROBO_CMD.FORWARD:
            return self.forward()
        elif cmd == ROBO_CMD.BACKWARD:
            return self.backward()
        elif cmd == ROBO_CMD.LEFT:
            return self.left()
        elif cmd == ROBO_CMD.RIGHT:
            return self.right()
        elif cmd == ROBO_CMD.STOP:
            return self.stop()

    def forward(self):
        return self._move_cmd(ROBO_CMD.FORWARD)

    def backward(self):
        return self._move_cmd(ROBO_CMD.BACKWARD)

    def left(self):
        return self._move_cmd(ROBO_CMD.LEFT)

    def right(self):
        return self._move_cmd(ROBO_CMD.RIGHT)

    def stop(self):
        return self.__robot_name + "|" + ROBO_CMD.MOVE.value + "|" + ROBO_CMD.STOP.value + "|"

    def _move_cmd(self, direction):
        return self.__robot_name + "|" + ROBO_CMD.MOVE.value + "|" + direction.value + ", 0.1|"