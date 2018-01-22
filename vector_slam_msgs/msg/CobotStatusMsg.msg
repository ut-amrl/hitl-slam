Header header

float64 timestamp

float32 loc_x
float32 loc_y
float32 orientation

float32[] particles_x
float32[] particles_y
float32[] particles_weight
float32[] locations_weight

uint16 currentNavCommand
float32 obsAvDir_x
float32 obsAvDir_y
bool pathBlocked

float32 batteryVoltage
bool emergencyStop

# Cobot Operation modes:
# 0 = JoystickMode
# 1 = JoystickObstAvMode
# 2 = MotionPrimitiveMode
# 3 = MotionPrimitiveObstAvMode
# 4 = TeleOpMode
# 5 = TestMode
# 6 = AllStopMode
int16 cobotMode

# Cobot Commands:
# 0 = NavCmdNone
# 1 = NavCmdGoTo
# 2 = NavCmdMoveStraight
# 3 = NavCmdMoveDownCorridor
# 4 = NavCmdIntegratedTurn
# 5 = NavCmdInPlaceTurn
# 6 = NavCmdTurnAndMoveStraight
# 7 = NavCmdAbort
int16 currentCommand
bool navComplete

int32 currentEdge
float32 edgeProgress
float32 edgeRemaining
float64 edgeNavDuration

float32[] pathPlan_x
float32[] pathPlan_y
