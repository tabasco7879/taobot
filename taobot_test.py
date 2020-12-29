from taobot import Taobot
import time
from bluedot import BlueDot
from signal import pause

tbot = Taobot()
bd = BlueDot()

with tbot as robot:
    def move(pos):
        if pos.top:
            robot.forward(0.3)
        elif pos.bottom:
            robot.backward(0.3)
        elif pos.left:
            robot.left(0.3)
        elif pos.right:
            robot.right(0.3)

    def stop():
        robot.stop()

    bd.when_pressed = move
    bd.when_moved = move
    bd.when_released = stop

    pause()
