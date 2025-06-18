F310_CODE_MAP = {
    'ABS_Y':0,
    'ABS_X':1,
    'ABS_RZ':2,
    'ABS_Z':3,
    'BTN_BASE5':0,
    'BTN_BASE6':1,
    'BTN_TOP2':2,
    'BTN_BASE':3,
    'BTN_PINKIE':4,
    'BTN_BASE2':5,
    'BTN_TOP':6,
    'BTN_TRIGGER':7,
    'BTN_THUMB2':8,
    'BTN_THUMB':9,
    'BTN_BASE4':10,
    'BTN_BASE3':11,
    'ABS_HAT0Y':4,
    'ABS_HAT0X':5,
    'ABS_HAT0Y':6,
    'ABS_HAT0X':7,
}

F310_VALUE_MAP = {
    0: (0, 255),
    1: (0, 255),
    2: (0, 255),
    3: (0, 255),
    4: (-1, 1),
    5: (-1, 1),
    6: (-1, 1),
    7: (-1, 1)
}

JOYSTICK_CODE_VALUE_MAP = {
    'Logitech Logitech Dual Action': (F310_CODE_MAP, F310_VALUE_MAP),
}