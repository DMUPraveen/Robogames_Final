from controller import Keyboard
from dave_lib import Dave

TURN_VELOCITY = 2.0
LINEAR_VELOCITY = 5.0
actions = {
    ord('A'): lambda dave: Dave.simple_turn_left(dave, TURN_VELOCITY),
    ord('D'): lambda dave: Dave.simple_turn_right(dave, TURN_VELOCITY),
    ord('S'): lambda dave: Dave.simple_reverse(dave, LINEAR_VELOCITY),
    ord('W'): lambda dave: Dave.simple_forward(dave, LINEAR_VELOCITY)
}


def control_dave_via_keyboard(keyboard: Keyboard, dave: Dave):
    key = None
    key = keyboard.getKey()
    if(key in actions):
        actions[key](dave)
    else:
        dave.simple_stop()
