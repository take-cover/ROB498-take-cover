from enum import Enum, auto

ARUCO_POS_NOT_RECEIVED_TIME = 6 # [s]

class State(Enum):
    IDLE = auto()
    LAUNCHING = auto()
    HOVERING = auto()
    SEARCHING = auto()
    TRACKING = auto()
    DROP_PAYLOAD = auto()

SERVICE_CALL_LAUNCH_DONE = False
SERVICE_CALL_TEST_DONE = False


class Event(Enum):
    SERVICE_CALL_LAUNCH = auto()
    REACHED_HOVER_HEIGHT = auto()
    SERVICE_CALL_TEST = auto()
    RECEIVED_ARUCO_POSITION = auto()
    REACHED_SETPOINT = auto()
    TIMER_NOT_RECEIVED_ARUCO_POSITION = auto()
    DROPPED_PAYLOAD = auto()

TRANSITIONS = {
    (State.IDLE, Event.SERVICE_CALL_LAUNCH): State.LAUNCHING,
    
    (State.LAUNCHING, Event.REACHED_HOVER_HEIGHT): State.HOVERING,

    (State.HOVERING, Event.SERVICE_CALL_TEST): State.SEARCHING,

    (State.SEARCHING, Event.RECEIVED_ARUCO_POSITION): State.TRACKING,

    (State.TRACKING, (Event.REACHED_SETPOINT, False)): State.DROP_PAYLOAD,
    (State.TRACKING, (Event.REACHED_SETPOINT, True)): State.TRACKING,
    (State.TRACKING, Event.TIMER_NOT_RECEIVED_ARUCO_POSITION): State.SEARCHING,
    (State.TRACKING, Event.RECEIVED_ARUCO_POSITION): State.TRACKING,
    
    (State.DROP_PAYLOAD, Event.DROPPED_PAYLOAD): State.TRACKING
}


def service_call_launch_done(state):
    global SERVICE_CALL_LAUNCH_DONE
    SERVICE_CALL_LAUNCH_DONE = state


def service_call_test_done(state):
    global SERVICE_CALL_TEST_DONE
    SERVICE_CALL_TEST_DONE = state


def transition(state, event):
    new_state = TRANSITIONS.get((state, event), None)
    if new_state is None:
        print(f"ERROR: FSM: Transition does not exist from state {state} with event {event}")
        new_state = state
    elif not state_equal(new_state, state):
        print(f"FSM: Transitioned from state {state} to {new_state} with event {event}")
    return new_state


def state_equal(state1, state2):
    return state1 == state2


def evaluate(
        state,
        state_vars,
):
    new_state = state

    received_aruco_pos = False
    if state_vars.get("received_aruco_pos_time", None) is not None:
        if state_vars.get("now_s") - state_vars.get("received_aruco_pos_time") < ARUCO_POS_NOT_RECEIVED_TIME:
            received_aruco_pos = True

    if state_equal(state, State.IDLE):
        if state_vars.get("started_launch", False):
            new_state = transition(state, Event.SERVICE_CALL_LAUNCH)
            service_call_launch_done(True)
            print(SERVICE_CALL_LAUNCH_DONE)

    elif state_equal(state, State.LAUNCHING):
        if state_vars.get("hover_height_reached", False):
            new_state = transition(state, Event.REACHED_HOVER_HEIGHT)

    elif state_equal(state, State.HOVERING):
        if state_vars.get("started_test", False):
            new_state = transition(state, Event.SERVICE_CALL_TEST)
            service_call_test_done(True)
            print(SERVICE_CALL_TEST_DONE)

    elif state_equal(state, State.SEARCHING):
        if received_aruco_pos:
            new_state = transition(state, Event.RECEIVED_ARUCO_POSITION)

    elif state_equal(state, State.TRACKING):
        if state_vars.get("tracking_setpoint_reached", False):
            new_state = transition(state, (Event.REACHED_SETPOINT, state_vars.get("dropped_payload", False)))
        elif received_aruco_pos:
            new_state = transition(state, Event.RECEIVED_ARUCO_POSITION)
        else:
            new_state = transition(state, Event.TIMER_NOT_RECEIVED_ARUCO_POSITION)
            
    elif state_equal(state, State.DROP_PAYLOAD):
        if state_vars.get("dropped_payload", False):
            new_state = transition(state, Event.DROPPED_PAYLOAD)

    else:
        print(f"FSM: invalid state: {state}")

    return new_state

