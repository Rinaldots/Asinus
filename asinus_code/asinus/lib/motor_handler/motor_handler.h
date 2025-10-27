#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <stddef.h>

extern const size_t motor_count_total;
extern int motors_all[];

extern const size_t motor_count_port1;
extern int motors_port1[];
extern const size_t motor_count_port2;
extern int motors_port2[];

extern const size_t motor_count_right;
extern int motors_right[];
extern const size_t motor_count_left;
extern int motors_left[];

extern int motor_speed[];
extern int slave_state[];
extern int motoroffset;

bool isMotorLeft(int motorId);

void setup_motor();
void update_motor();

#endif // MOTOR_HANDLER_H
