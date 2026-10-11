/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Wi-Fi radar: motion and presence sensed from channel state information
// (CSI). While the board is on Wi-Fi it pings its gateway so the access point
// keeps answering, and the radio reports the per-subcarrier channel estimate
// of every frame the AP sends. People moving in the room disturb that
// estimate; an empty room leaves it steady.
#pragma once

#include <stddef.h>
#include "cJSON.h"

// Start the presence task. It turns CSI on whenever Wi-Fi comes up and off
// when it drops.
void presence_csi_init(void);

// presence.read: whether anyone has moved recently, the current motion level
// and the raw statistics behind it. An error while Wi-Fi is down or before the
// first seconds of samples are in.
cJSON *presence_csi_command(void);

// One short line of state for the heartbeat log.
void presence_csi_status(char *buf, size_t size);
