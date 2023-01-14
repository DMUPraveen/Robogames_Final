from controller import Keyboard
from dave_lib import Dave

TURN_VELOCITY = 2.0
LINEAR_VELOCITY = 5.0
keyboard_control_on = True
CONTROL_KEY = ord('Q')
actions = {
    ord('A'): lambda dave: Dave.simple_turn_left(dave, TURN_VELOCITY),
    ord('D'): lambda dave: Dave.simple_turn_right(dave, TURN_VELOCITY),
    ord('S'): lambda dave: Dave.simple_reverse(dave, LINEAR_VELOCITY),
    ord('W'): lambda dave: Dave.simple_forward(dave, LINEAR_VELOCITY)
}


def control_dave_via_keyboard(keyboard: Keyboard, dave: Dave):
    key = None
    global keyboard_control_on
    key = keyboard.getKey()
    if(key == CONTROL_KEY):
        keyboard_control_on = not(keyboard_control_on)
    if(not keyboard_control_on):
        return
    if(key in actions):
        actions[key](dave)
        return

    dave.simple_stop()
