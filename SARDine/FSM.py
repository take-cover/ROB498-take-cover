from enum import Enum, auto

class State(Enum):
    IDLE = auto()
    LAUNCHING = auto()
    HOVERING = auto()
    SEARCHING = auto()
    TRACKING = auto()

SERVICE_CALL_LAUNCH_DONE = False
SERVICE_CALL_TEST_DONE = False


class Event(Enum):
    SERVICE_CALL_LAUNCH = auto()
    REACHED_HOVER_HEIGHT = auto()
    SERVICE_CALL_TEST = auto()
    RECEIVED_ARUCO_POSITION = auto()
    REACHED_SETPOINT = auto()
    TIMER_NOT_RECEIVED_ARUCO_POSITION = auto()


TRANSITIONS = {
    (State.IDLE, Event.SERVICE_CALL_LAUNCH): State.LAUNCHING,
    (State.LAUNCHING, Event.REACHED_HOVER_HEIGHT): State.HOVERING,

    (State.HOVERING, Event.SERVICE_CALL_TEST): State.SEARCHING,
    (State.HOVERING, Event.RECEIVED_ARUCO_POSITION): State.TRACKING if SERVICE_CALL_TEST_DONE else State.HOVERING,
    (State.HOVERING, Event.TIMER_NOT_RECEIVED_ARUCO_POSITION): State.SEARCHING if SERVICE_CALL_TEST_DONE else State.HOVERING,

    (State.SEARCHING, Event.RECEIVED_ARUCO_POSITION): State.TRACKING,

    (State.TRACKING, Event.REACHED_SETPOINT): State.HOVERING,
    (State.TRACKING, Event.TIMER_NOT_RECEIVED_ARUCO_POSITION): State.SEARCHING,
    (State.TRACKING, Event.RECEIVED_ARUCO_POSITION): State.TRACKING
}


def service_call_launch_done(state):
    global SERVICE_CALL_LAUNCH_DONE
    SERVICE_CALL_LAUNCH_DONE = state


def service_call_test_done(state):
    global SERVICE_CALL_TEST_DONE
    SERVICE_CALL_TEST_DONE = state


def transition(state, event):
    return TRANSITIONS.get((state, event), None)


def state_equal(state1, state2):
    return state1 == state2

