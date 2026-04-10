# send_do_winch_10m.py

WINCH_CMD = 42600   # MAV_CMD_DO_WINCH
WINCH_NUM = 1
DEPLOY_LENGTH = 10  # meters

print("Sending DO_WINCH: deploy to 10 m")

if not cs.armed:  # noqa: F821
    print("Vehicle not armed. Aborting.")
    exit()

MAV.doCommand(  # noqa: F821
    MAV.MAV_CMD.DO_WINCH,  # noqa: F821
    WINCH_NUM,     # param1: winch instance
    1,             # param2: action (1 = deploy)
    DEPLOY_LENGTH, # param3: length in meters
    0,             # param4: rate (0 = default)
    0, 0, 0
)

print("Command sent")
