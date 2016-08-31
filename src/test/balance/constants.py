from Box2D.b2 import staticBody, dynamicBody

# Colors.
WHITE = (255,255,255,255)
BLACK = (0,0,0,255)
SHIKIRI = (53,40,0,255)
BACKGROUND = (178,156,91,255)
RED = (255,0,0,255)
BLUE = (0,0,255,255)
YELLOW = (255,255,0,255)
GREEN = (0,255,0,255)
GREEN_T = (0,255,0,27)
GRAY = (127,127,127,255)
DEFAULT_BODY_COLORS = {
    staticBody  : WHITE,
    dynamicBody : GRAY,
}

COLOR_TO_NAME = dict(
    WHITE = WHITE,
    BLACK = BLACK,
    SHIKIRI = SHIKIRI,
    BACKGROUND = BACKGROUND,
    RED = RED,
    BLUE = BLUE,
    YELLOW = YELLOW,
    GREEN = GREEN,
    GREEN_T = GREEN_T,
    GRAY = GRAY,    
)

FACE_TO_FACE = 'face-to-face'
BACK_TO_BACK = 'back-to-back'
PARALLEL_SAME = 'parallel-same'
PARALLEL_OPPOSITE = 'parallel-opposite'
CATTY_CORNER_FACING = 'catty-corner-facing'
CATTY_CORNER_BACKING = 'catty-corner-backing'

POSITIONS = (
    FACE_TO_FACE,
    BACK_TO_BACK,
    PARALLEL_SAME,
    PARALLEL_OPPOSITE,
    CATTY_CORNER_FACING,
    CATTY_CORNER_BACKING,
)

ACTION_NONE = 'none'
ACTION_STOP = 'stop'
ACTION_STRAIGHT = 'straight'
ACTION_FORWARD = 'forward'
ACTION_BACKWARD = 'backward'
ACTION_TURN = 'turn'
