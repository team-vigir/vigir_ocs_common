# internal message used to communicate key events
int32 keycode        # keyboard binary signal indicating which key generated the event
int32 keysym         # key converted to symbol in the default xkb mapping (e.g., 40 keycode = 0x64 keysym in US keyboard layout)
string keystr        # key symbol converted to string (standard, makes it easy to compare and to detect special keys)
int8  state          # pressed/released

# absolute mouse cursor position
int32 cursor_x       
int32 cursor_y       
