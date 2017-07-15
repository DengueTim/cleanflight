
#pragma once

uint8_t snatchFrameStatus(timeUs_t currentTimeUs, uint8_t frameStatus);
bool snatchInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

void snatchSendPidLoopEvent(timeUs_t currentTimeUs);
