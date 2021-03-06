/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * emergency_parser.cpp - Intercept special commands directly in the serial stream
 */

#include "MarlinConfig.h"

#if ENABLED(EMERGENCY_PARSER)

#include "emergency_parser.h"

// Static data members
bool EmergencyParser::killed_by_M112; // = false
EmergencyParser::State EmergencyParser::state; // = EP_RESET
bool EmergencyParser::quickstop_byM410 = false;
bool EmergencyParser::clear_gcode_byM411 = false;
// Global instance
EmergencyParser emergency_parser;

#endif // EMERGENCY_PARSER
