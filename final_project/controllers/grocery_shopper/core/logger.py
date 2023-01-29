from enum import Enum
from time import time
from datetime import datetime


class LogLevel(Enum):
    NONE = 0
    ERROR = 1
    INFO = 2
    DEBUG = 3


ll = None


def set_log_level(level):
    global ll
    ll = level


def log(msg: str, level: LogLevel = LogLevel.DEBUG):
    global ll
    if ll is None:
        set_log_level(LogLevel.DEBUG)

    if level.value <= ll.value:
        print(f"[{datetime.now()}] [{str(level)[9:]}]: {msg}")
