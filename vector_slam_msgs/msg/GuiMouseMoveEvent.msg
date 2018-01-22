# Feedback in terms of mouse clicks on the localization GUI.
#

# Standard header. The frame should normally be "map", since the localization
# GUI reports coordinates in the world (map) frame.
std_msgs/Header header

# The 2D location where the mouse pointer was moved to. The z coordinate of the
# Point32 should be set to 0 by publishers, and ignored by subscribers.
geometry_msgs/Point32 location

# Binary mask of mouse buttons that were depressed, as enumerated by the
# Qt::MouseButton enumerator. Comon values are:
# 0x00 = No button
# 0x01 = Left button
# 0x02 = Right button
# 0x04 = Middle button
uint32 buttons

# Binary mask of keyboard modifiers that were active when the mouse event occurred.
# 0x01 = Alt key  modifier
# 0x02 = Ctrl key modifier
# 0x04 = Shift key modifier
uint32 modifiers

