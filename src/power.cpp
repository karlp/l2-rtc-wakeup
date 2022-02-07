/*
 * Handles the nitty gritty of entering and exiting low power modes,
 * and when that's allowed.
 */
#include <stdio.h>
#include "FreeRTOS.h"

extern "C" {
	void myPreSleepFunction(TickType_t xModifiableIdleTime) {
		printf("pre\n");
	}
	void myPostSleepFunction(const TickType_t xExpectedIdleTime) {
		printf("post\n");
	}
}
