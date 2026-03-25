import FSM
import time

state_vars = {
    "now_s": None,
    "started_launch": False,
    "started_test": False,
    "hover_height_reached": False,
    "tracking_setpoint_reached": False,
    "received_aruco_pos_time": None
}

state = FSM.State(FSM.State.IDLE)
print(f"Start with state {state}")

def eval():
    state_vars["now_s"] = time.time()
    return FSM.evaluate(state, state_vars)

time.sleep(3) # wait 3 seconds

# launch
state_vars["started_launch"] = True
state = eval()

time.sleep(3) # wait 3 seconds

# reached hover height
state_vars["hover_height_reached"] = True
state = eval()

time.sleep(3) # wait 3 seconds

# start test
state_vars["started_test"] = True
state = eval()

time.sleep(10) # wait 10 seconds

# found aruco
state_vars["received_aruco_pos_time"] = time.time()
state = eval()

# tracking aruco
time.sleep(2) # wait 2 seconds
state_vars["received_aruco_pos_time"] = time.time()
state = eval()
time.sleep(2) # wait 2 seconds
state_vars["tracking_setpoint_reached"] = True
state = eval()
time.sleep(4)
# state_vars["received_aruco_pos_time"] = time.time()
state = eval()
