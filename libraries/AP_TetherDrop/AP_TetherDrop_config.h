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

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_TETHERDROP_ENABLED
// TetherDrop requires flash space, only enable on larger boards for now
// In future, this could be changed to APM_BUILD_TYPE(APM_BUILD_Rover) once
// all config headers are evaluated after APM_BUILD_DIRECTORY is defined
#define AP_TETHERDROP_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

#ifndef AP_TETHERDROP_SERIAL_ENABLED
#define AP_TETHERDROP_SERIAL_ENABLED AP_TETHERDROP_ENABLED
#endif
