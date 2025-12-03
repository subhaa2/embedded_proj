#ifndef MAIN_H
#define MAIN_H

#include "pico/stdlib.h"
#include <stdio.h>

extern bool isCompleted;

void SendSpiderCommand(int _command);
void SendHumanFoundCommand();
#endif