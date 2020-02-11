#ifndef LCD_HPP
#define LCD_HPP

#include "main.h"

void auton_index_left();
void auton_index_right();
void auton_locker();
void log_to_lcd(int line, const char* string, int argument);
int get_selector_var();


#endif
