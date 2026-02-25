/* Copyright 2025 muzkr https://github.com/muzkr
 * Copyright 2023 Manuel Jinger
 * Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
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
 */

#include "driver/gpio.h"
#include "driver/keyboard.h"
#include "driver/systick.h"
#include "driver/i2c.h"
#include "misc.h"

KEY_Code_t gKeyReading0     = KEY_INVALID;
KEY_Code_t gKeyReading1     = KEY_INVALID;
uint16_t   gDebounceCounter = 0;
bool       gWasFKeyPressed  = false;

#ifdef ENABLE_FEAT_F4HWN_SCREENSHOT
// Number of consecutive KEYBOARD_Poll() calls that return the injected key.
// Must exceed key_debounce_10ms (typically 2) to pass the debounce gate in app.c.
// After HOLD polls the key is cleared, which triggers a natural release cycle.
#define SERIAL_KEY_HOLD_POLLS 5

volatile KEY_Code_t gKeyFromSerial      = KEY_INVALID;
static   uint8_t    gSerialKeyHoldCount = 0;

// Inject a key received from serial (UART or VCP).
// keyCode is the raw packet byte; validated against KEY_Code_e range before use.
// KEY_PTT is explicitly blocked — PTT release cannot be guaranteed over serial.
void KEYBOARD_InjectKey(uint8_t keyCode)
{
    if (keyCode < KEY_INVALID && keyCode != KEY_PTT) {
        gKeyFromSerial      = (KEY_Code_t)keyCode;
        gSerialKeyHoldCount = 0;
    }
}
#endif

#define GPIOx               GPIOB
#define PIN_MASK_COLS       (LL_GPIO_PIN_6 | LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3)
#define PIN_COLS            GPIO_MAKE_PIN(GPIOx, PIN_MASK_COLS)
#define PIN_COL(n)          GPIO_MAKE_PIN(GPIOx, 1u << (6 - (n)))

#define PIN_MASK_ROWS       (LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_13 | LL_GPIO_PIN_12)
#define PIN_MASK_ROW(n)     (1u << (15 - (n)))

static inline uint32_t read_rows()
{
    return PIN_MASK_ROWS & LL_GPIO_ReadInputPort(GPIOx);
}

static const KEY_Code_t keyboard[5][4] = {
    {   // Zero col
        // Set to zero to handle special case of nothing pulled down
        KEY_SIDE1, 
        KEY_SIDE2, 

        // Duplicate to fill the array with valid values
        KEY_INVALID, 
        KEY_INVALID, 
    },
    {   // First col
        KEY_MENU, 
        KEY_1, 
        KEY_4, 
        KEY_7, 
    },
    {   // Second col
        KEY_UP, 
        KEY_2 , 
        KEY_5 , 
        KEY_8 , 
    },
    {   // Third col
        KEY_DOWN, 
        KEY_3   , 
        KEY_6   , 
        KEY_9   , 
    },
    {   // Fourth col
        KEY_EXIT, 
        KEY_STAR, 
        KEY_0   , 
        KEY_F   , 
    }
};

KEY_Code_t KEYBOARD_Poll(void)
{
#ifdef ENABLE_FEAT_F4HWN_SCREENSHOT
    // Serial-injected key: hold it for SERIAL_KEY_HOLD_POLLS consecutive calls
    // so the debounce counter in app.c has time to reach key_debounce_10ms.
    // Once the hold count is exhausted we clear it — the next call returns
    // KEY_INVALID, which triggers the release path in app.c naturally.
    if (gKeyFromSerial != KEY_INVALID) {
        KEY_Code_t injected = gKeyFromSerial;
        if (++gSerialKeyHoldCount >= SERIAL_KEY_HOLD_POLLS) {
            gKeyFromSerial      = KEY_INVALID;
            gSerialKeyHoldCount = 0;
        }
        return injected;
    }
#endif

    KEY_Code_t Key = KEY_INVALID;

    // Scan all 5 columns - never break early to avoid GPIO state issues
    for (unsigned int j = 0; j < 5; j++)
    {
        uint32_t reg;
        uint32_t match_count = 0;  // Count consecutive matching reads

        // Set all columns high first
        GPIO_SetOutputPin(PIN_COLS);

        // Clear the specific column we are selecting
        if (j > 0)
        {
            GPIO_ResetOutputPin(PIN_COL(j - 1));
        }

        // Debounce: Read rows multiple times and look for stable reads
        // CRITICAL FIX #1: Proper debounce logic (replaces confusing i *= syntax)
        // CRITICAL FIX #2: Increased delay from 1µs to 10µs for real keyboard bounce capture
        // CRITICAL FIX #3: Clear match_count if reads differ (noise detection)
        reg = 0;
        for (unsigned int k = 0; k < 8; k++)
        {
            SYSTICK_DelayUs(10);  // FIX #2: Increased from 1µs to 10µs
            uint32_t reg2 = read_rows();
            
            // FIX #3: Clear match counter if values don't match
            if (reg2 != reg)
            {
                match_count = 0;
                reg = reg2;
            }
            else
            {
                match_count++;
            }
            
            // Success: We have 3 consecutive matching reads = stable signal
            if (match_count >= 2)
            {
                break;  // Debounce complete for this column
            }
        }

        // FIX #1: Do NOT break on noise - continue scanning all columns
        // Only skip key detection if debounce failed, but GPIO state is cleaned
        if (match_count < 2)
        {
            // Debounce failed (too much noise), but continue to next column
            // This prevents GPIO from staying in invalid state and blocking other columns
            continue;
        }

        // Debounce successful, check which row is pressed in this column
        for (unsigned int i = 0; i < 4; i++)
        {
            if (!(reg & PIN_MASK_ROW(i)))
            {
                Key = keyboard[j][i];
                break;
            }
        }

        if (Key != KEY_INVALID)
        {
            break;  // Found a valid key, stop scanning
        }
    }

    // CRITICAL FIX #4: Always clean up GPIO state - set all columns high at end
    // This ensures GPIO pins are in a known state even if function exited early due to noise
    GPIO_SetOutputPin(PIN_COLS);

    return Key;
}