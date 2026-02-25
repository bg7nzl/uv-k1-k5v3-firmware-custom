/* Copyright 2025 muzkr https://github.com/muzkr
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 *
 */

#include "driver/vcp.h"
#include "usb_config.h"
#include "py32f071_ll_bus.h"

#ifdef ENABLE_FEAT_F4HWN_SCREENSHOT
#include "driver/keyboard.h"
// Packet type for serial key injection (K5Viewer → radio)
#define VCP_TYPE_KEY 0x03
#endif

uint8_t VCP_RxBuf[VCP_RX_BUF_SIZE];
volatile uint32_t VCP_RxBufPointer = 0;

void VCP_Init()
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA); // PA12:11
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USBD);

    cdc_acm_rx_buf_t rx_buf = {
        .buf = VCP_RxBuf,
        .size = sizeof(VCP_RxBuf),
        .write_pointer = &VCP_RxBufPointer,
    };
    cdc_acm_init(rx_buf);

    NVIC_SetPriority(USBD_IRQn, 3);
    NVIC_EnableIRQ(USBD_IRQn);
}

bool VCP_ScreenshotPing(void)
{
#ifdef ENABLE_FEAT_F4HWN_SCREENSHOT
    // State machine for parsing incoming packets:
    //   Keepalive:  0x55 0xAA 0x00 0x00  → viewer alive
    //   Key packet: 0xAA 0x55 0x03 <key> → inject key
    //
    // State transitions:
    //   IDLE  → 0x55 → KA_1
    //   KA_1  → 0xAA → KA_2  (else IDLE)
    //   KA_2  → 0x00 → KA_3  (else IDLE)
    //   KA_3  → 0x00 → keepalive OK, IDLE
    //
    //   IDLE  → 0xAA → KEY_1
    //   KEY_1 → 0x55 → KEY_2  (else IDLE)
    //   KEY_2 → 0x03 → KEY_3  (else IDLE)
    //   KEY_3 → <b>  → inject key, IDLE

    typedef enum {
        STATE_IDLE = 0,
        STATE_KA_1,
        STATE_KA_2,
        STATE_KA_3,
        STATE_KEY_1,
        STATE_KEY_2,
        STATE_KEY_3,
    } ParseState_t;

    static uint32_t     read_ptr = 0;
    static ParseState_t state    = STATE_IDLE;

    bool     connected = false;
    uint32_t write_ptr = VCP_RxBufPointer;

    while (read_ptr != write_ptr)
    {
        uint8_t b = VCP_RxBuf[read_ptr];
        read_ptr++;
        if (read_ptr >= VCP_RX_BUF_SIZE)
            read_ptr = 0;

        switch (state)
        {
            case STATE_IDLE:
                if      (b == 0x55) state = STATE_KA_1;
                else if (b == 0xAA) state = STATE_KEY_1;
                break;

            case STATE_KA_1:
                state = (b == 0xAA) ? STATE_KA_2 : STATE_IDLE;
                break;

            case STATE_KA_2:
                state = (b == 0x00) ? STATE_KA_3 : STATE_IDLE;
                break;

            case STATE_KA_3:
                if (b == 0x00) connected = true;
                state = STATE_IDLE;
                break;

            case STATE_KEY_1:
                state = (b == 0x55) ? STATE_KEY_2 : STATE_IDLE;
                break;

            case STATE_KEY_2:
                state = (b == VCP_TYPE_KEY) ? STATE_KEY_3 : STATE_IDLE;
                break;

            case STATE_KEY_3:
                KEYBOARD_InjectKey(b);
                connected = true;
                state = STATE_IDLE;
                break;

            default:
                state = STATE_IDLE;
                break;
        }
    }

    return connected;

#else
    // Simple ping: just detect any incoming byte from the viewer
    static uint32_t read_ptr = 0;
    uint32_t write_ptr = VCP_RxBufPointer;

    if (read_ptr != write_ptr) {
        read_ptr = write_ptr;
        return true;
    }
    return false;
#endif
}