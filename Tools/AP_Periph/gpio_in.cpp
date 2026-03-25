/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  GPIO digital input (open or tied to ground) reading and DroneCAN
  hardpoint_Status broadcast.

  The pin is configured as an input with internal pull-up so that:
    - open (floating)  -> reads HIGH -> status 1
    - tied to ground   -> reads LOW  -> status 0

  The state is sent periodically at gpio_in_rate Hz, and also
  immediately on any change.
*/

#include "AP_Periph.h"

#if AP_PERIPH_GPIO_IN_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  initialise GPIO input pin
*/
void AP_Periph_FW::gpio_in_init()
{
    const int8_t pin = g.gpio_in_pin;
    if (pin < 0) {
        return;
    }

    // configure as input and enable internal pull-up
    // (open = HIGH = 1, tied to ground = LOW = 0)
    hal.gpio->pinMode((uint8_t)pin, HAL_GPIO_INPUT);
    hal.gpio->write((uint8_t)pin, 1);  // write(pin, 1) on an input enables pull-up
}

/*
  read GPIO input and broadcast state via DroneCAN hardpoint_Status
  Called from can_update() main loop.
*/
void AP_Periph_FW::gpio_in_update()
{
    const int8_t pin = g.gpio_in_pin;
    if (pin < 0) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // calculate period from rate parameter (clamped to 1-50 Hz)
    const uint8_t rate = (uint8_t)constrain_int16(g.gpio_in_rate.get(), 1, 50);
    const uint32_t period_ms = 1000U / rate;

    // read current pin state
    const bool state = (hal.gpio->read((uint8_t)pin) != 0);

    // send if state changed OR periodic timer expired
    if (state == gpio_in.last_state && (now - gpio_in.last_send_ms) < period_ms) {
        return;
    }

    gpio_in.last_state = state;
    gpio_in.last_send_ms = now;

    uavcan_equipment_hardpoint_Status msg {};
    msg.hardpoint_id = (uint8_t)g.gpio_in_hardpoint_id.get();
    msg.status = state ? 1 : 0;

    uint8_t buffer[UAVCAN_EQUIPMENT_HARDPOINT_STATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_hardpoint_Status_encode(&msg, buffer, !canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_HARDPOINT_STATUS_SIGNATURE,
                     UAVCAN_EQUIPMENT_HARDPOINT_STATUS_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);
}

#endif  // AP_PERIPH_GPIO_IN_ENABLED
