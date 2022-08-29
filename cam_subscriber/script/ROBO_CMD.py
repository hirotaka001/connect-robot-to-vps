from enum import Enum

class ROBO_CMD(Enum):
    MOVE = "move"
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"
    NONE = "none"