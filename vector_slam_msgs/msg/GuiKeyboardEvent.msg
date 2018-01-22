# Feedback in terms keyboard keypress events on the localization GUI.
#

# Standard header. The frame should normally be "map", since the localization
# GUI reports coordinates in the world (map) frame.
std_msgs/Header header

# The Keycode constant, as enumerated in the Qt::Key enumerator.
uint32 keycode

# Binary mask of keyboard modifiers that were active when the mouse event occurred.
# 0x01 = Alt key  modifier
# 0x02 = Ctrl key modifier
# 0x04 = Shift key modifier
uint32 modifiers

