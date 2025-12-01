#ifndef ALGORITHM_MAIN_H
#define ALGORITHM_MAIN_H

#include "defs.h"

void algo_init(void);
void algo_execute_spider_command(void);
void ProcessSensorDataAndSendCommand(const char* radar_data);
#endif