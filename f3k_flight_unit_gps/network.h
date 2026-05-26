#pragma once

#include <Arduino.h>

// UDP packet senders and inbound scorer window-command polling.
// Function names intentionally match the old .ino functions so existing
// call sites do not need to change during this staged refactor.

void sendUdpPacket();
void sendDebugPacket();
void sendGpsPacket();
void sendAnnouncement();
void checkWindowCommand(unsigned long now);
