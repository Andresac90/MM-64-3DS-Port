#ifndef PORT_CONTROLLER_KEYBOARD_H
#define PORT_CONTROLLER_KEYBOARD_H
#include <stdbool.h>
bool keyboard_on_key_down(int scancode);
bool keyboard_on_key_up(int scancode);
void keyboard_on_all_keys_up(void);
#endif
