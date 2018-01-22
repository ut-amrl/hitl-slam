# Feedback in terms of mouse clicks on the localization GUI.
#

# Standard header. The frame should normally be "map", since the localization
# GUI reports coordinates in the world (map) frame.
std_msgs/Header header

# The 2D location where the "MouseDown" event occurred. The z coordinate of the
# Point32 should be set to 0 by publishers, and ignored by subscribers.
geometry_msgs/Point32 mouse_down

# The 2D location where the "MouseUp" event occurred. The z coordinate of the
# Point32 should be set to 0 by publishers, and ignored by subscribers.
geometry_msgs/Point32 mouse_up

# Binary mask of keyboard modifiers that were active when the mouse event occurred.
# 0x01 = Alt key  modifier
# 0x02 = Ctrl key modifier
# 0x04 = Shift key modifier
uint32 modifiers

