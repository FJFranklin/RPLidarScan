import sys
import os
import termios

from enum import Enum

from typing import Tuple, Callable, Dict

import asyncio
import aiofiles

if sys.version_info.minor >= 11:
    from asyncio import TaskGroup
else:
    from taskgroup import TaskGroup

class GamepadKey(Enum):
    UNKNOWN       =  0
    MOVE_LEFT     =  1
    MOVE_RIGHT    =  2
    MOVE_FORWARD  =  3
    MOVE_BACK     =  4
    ROTATE_LEFT   =  5
    ROTATE_RIGHT  =  6
    ROTATE_DOWN   =  7
    ROTATE_UP     =  8
    BUMPER_LEFT   =  9
    BUMPER_RIGHT  = 10
    TRIGGER_LEFT  = 11
    TRIGGER_RIGHT = 12
    THUMB_LEFT    = 13
    THUMB_RIGHT   = 14
    SELECT        = 15
    START         = 16
    MOTION        = 17

GamepadKeys = {
    GamepadKey.UNKNOWN       : "(unknown)",
    GamepadKey.MOVE_LEFT     : "Button 4 or Move left",
    GamepadKey.MOVE_RIGHT    : "Button 2 or Move right",
    GamepadKey.MOVE_FORWARD  : "Button 1 or Move forward",
    GamepadKey.MOVE_BACK     : "Button 3 or Move back",
    GamepadKey.ROTATE_LEFT   : "Rotate left",
    GamepadKey.ROTATE_RIGHT  : "Rotate right",
    GamepadKey.ROTATE_DOWN   : "Rotate forwards/down",
    GamepadKey.ROTATE_UP     : "Rotate backwards/up",
    GamepadKey.BUMPER_LEFT   : "Bumper (left)",
    GamepadKey.BUMPER_RIGHT  : "Bumper (right)",
    GamepadKey.TRIGGER_LEFT  : "Trigger (left)",
    GamepadKey.TRIGGER_RIGHT : "Trigger (right)",
    GamepadKey.THUMB_LEFT    : "Thumb (left)",
    GamepadKey.THUMB_RIGHT   : "Thumb (right)",
    GamepadKey.SELECT        : "Select",
    GamepadKey.START         : "Start",
    GamepadKey.MOTION        : "(motion)" }

def GamepadKeyMotion(seconds: bytes, state: bytes) -> Tuple[float, float]:
    t = 0.001 * int.from_bytes(seconds, "little")
    s = int.from_bytes(state, "little", signed=True)
    return t, s

def GamepadKeyLookup(modifier: bytes, keycode: bytes, state: bytes) -> Tuple[GamepadKey, bool]:
    button_state = int.from_bytes(state, "little", signed=True)
    press_event = (button_state != 0)

    key = GamepadKey.UNKNOWN

    if modifier == 1:
        match keycode:
            case 0:
                key = GamepadKey.MOVE_FORWARD
            case 1:
                key = GamepadKey.MOVE_RIGHT
            case 2:
                key = GamepadKey.MOVE_BACK
            case 3:
                key = GamepadKey.MOVE_LEFT
            case 4:
                key = GamepadKey.BUMPER_LEFT
            case 5:
                key = GamepadKey.BUMPER_RIGHT
            case 6:
                key = GamepadKey.TRIGGER_LEFT
            case 7:
                key = GamepadKey.TRIGGER_RIGHT
            case 8:
                key = GamepadKey.SELECT
            case 9:
                key = GamepadKey.START
            case 10:
                key = GamepadKey.THUMB_LEFT
            case 11:
                key = GamepadKey.THUMB_RIGHT

    if modifier == 2:
        match keycode:
            case 0:
                if button_state < 0:
                    key = GamepadKey.ROTATE_LEFT
                else:
                    key = GamepadKey.ROTATE_RIGHT
            case 1:
                if button_state < 0:
                    key = GamepadKey.ROTATE_DOWN
                else:
                    key = GamepadKey.ROTATE_UP
            case 2:
                key = GamepadKey.MOTION

    return key, press_event
    
async def GamepadKeyLoop(buttonpress_handler: Callable = None, motion_handler: Callable = None):
    async with aiofiles.open('/dev/input/js0', 'rb') as J:
        while True:
            j = await J.read(8)
            key, press = GamepadKeyLookup(j[6], j[7], j[4:6])
            if key == GamepadKey.MOTION:
                if motion_handler is not None:
                    t, s = GamepadKeyMotion(j[0:4], j[4:6])
                    motion_handler(t, s)
            elif key != GamepadKey.UNKNOWN:
                if buttonpress_handler is not None:
                    if not buttonpress_handler(key, press): # returns False if cycle should end
                        break

async def GamepadKeyTask(D: Dict, buttonpress_handler: Callable = None, motion_handler: Callable = None):
    async with aiofiles.open('/dev/input/js0', 'rb') as J:
        while D["loop"]:
            task = asyncio.create_task(J.read(8))
            D["deviceread"] = task
            await asyncio.gather(task)
            D["deviceread"] = None
            j = task.result()

            key, press = GamepadKeyLookup(j[6], j[7], j[4:6])
            if key == GamepadKey.MOTION:
                if motion_handler is not None:
                    t, s = GamepadKeyMotion(j[0:4], j[4:6])
                    motion_handler(t, s)
            elif key != GamepadKey.UNKNOWN:
                if buttonpress_handler is not None:
                    if not buttonpress_handler(key, press): # returns False if cycle should end
                        D["loop"] = False

async def KeyboardTask(D: Dict, keypress_handler: Callable = None):
    current = termios.tcgetattr(sys.stdin.fileno())
    try:
        # Warning: This is Linux-specific
        settings = termios.tcgetattr(sys.stdin)
        settings[3] = settings[3] & ~(termios.ECHO | termios.ICANON | termios.ISIG) # no echo, canonical mode off, no signals like CTRL-C
        settings[6][termios.VMIN ] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        while D["loop"]:
            key = os.read(sys.stdin.fileno(), 1)
            if key is not None:
                if len(key) > 0 and keypress_handler is not None:
                    if not keypress_handler(key):
                        D["loop"] = False
                        task = D["deviceread"]
                        if task is not None:
                            task.cancel()
            await asyncio.sleep(1E-3)

    except KeyboardInterrupt: # Disabled via ISIG, but isn't generated anyway
        print(" (raw-tty-interrupt)")
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, current)

def demo_keypress_handler(key: bytes) -> bool:
    print("(key)     {k} ({o})".format(k=key, o=ord(key)))
    return not (ord(key) == 4 or ord(key) == 3) # ^c or ^d to exit

def demo_motion_handler(seconds: float, state: float) -> None:
    print("(motion)  {s:6.3f} @time {t:.3f}s".format(s=state, t=seconds))

def demo_buttonpress_handler(key: GamepadKey, press_event: bool) -> bool:
    if press_event:
        action = "(press)   "
    else:
        action = "(release) "
    print(action + GamepadKeys[key])

    return not (press_event and key == GamepadKey.START)

async def main_async(buttonpress_handler: Callable = None, motion_handler: Callable = None, keypress_handler: Callable = None):
    async with TaskGroup() as tg:
        D = { "loop": True, "taskgroup": tg, "deviceread": None }
        tg.create_task(GamepadKeyTask(D, buttonpress_handler, motion_handler))
        tg.create_task(KeyboardTask(D, keypress_handler))

if __name__ == '__main__':
    print("Press 'Start' on the gamepad to exit, or CTRL-C or CTRL-D followed by any gamepad action.")
    try:
        asyncio.run(main_async(demo_buttonpress_handler, demo_motion_handler, demo_keypress_handler))
        #asyncio.run(GamepadKeyLoop(demo_buttonpress_handler, demo_motion_handler))
    except KeyboardInterrupt:
        print(" (interrupt)")
