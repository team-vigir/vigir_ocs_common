# text for notification
string text

# size
uint32 height
uint32 width

# font type
string font

# font size
uint32 text_size

# maximum width of a line?
uint32 line_width

# foreground (font) color  
std_msgs/ColorRGBA fg_color
# background color
std_msgs/ColorRGBA bg_color

# types of actions
uint8 ADD    =  1
uint8 DELETE =  2
# determines which action will be performed
uint8 action

#types of alignment
uint8 LEFTCOLUMN = 1
uint8 CENTERCOLUMN = 2
uint8 RIGHTCOLUMN = 3

uint8 TOPROW = 1
uint8 CENTERROW = 2
uint8 BOTTOMROW = 3

#determines alignment
uint8 row
uint8 column

#times for the ui element to fadein/out/stay up
float32 upTime
float32 fadeIn
float32 fadeOut

