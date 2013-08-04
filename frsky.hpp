#ifndef QUANTRACKER_SIM_FRSKY_HPP_INCLUDED
#define QUANTRACKER_SIM_FRSKY_HPP_INCLUDED

/*
 Copyright (c) 2012 - 2013 Andy Little 

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <quan/uav/position.hpp>
#include <quan/angle.hpp>
#include <quan/length.hpp>

void FrSky_send_message();

//void update_aircraft_gps_position(
//   quan::uav::position<  
//      quan::angle_<int32_t>::deg10e7,
//      quan::length_<int32_t>::m 
//   > const & p
//);

#endif //QUANTRACKER_SIM_FRSKY_HPP_INCLUDED
