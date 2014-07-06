#include <cstdint>
#include <cassert>
#include <cstring>

#include <quan/uav/cobs/protocol.hpp>
#include <quan/uav/position.hpp>
#include <quan/utility/fifo.hpp>
#include <quan/uav/fletcher16.hpp>
#include "aircraft.hpp"
#include "app.h"

namespace {

   struct packet {
      static constexpr uint8_t latitude_id = 1;
      static constexpr uint8_t longtitude_id = 2;
      static constexpr uint8_t altitude_id = 3;
      
      static quan::uav::position<
      quan::angle_<int32_t>::deg10e7,
      quan::length_<int32_t>::m
      > m_current_position;
      
      static void get_aircraft_gps_position()
      {
         m_current_position = ::get_aircraft_gps_position();
      }
      
      static uint8_t raw[7];
      static uint8_t encoded[8];
      
      static int32_t normalise (quan::angle_<int32_t>::deg10e7 const & in)
      {
         constexpr int64_t ang_max  = 1800000000LL;
         return static_cast<int32_t> (in.numeric_value() % ang_max);
      }
      
      static void update_id (uint8_t id, int32_t val)
      {
         auto & app = wxGetApp();
         if (app.have_sp()) {
            raw[0] = id;
            union {
               uint8_t ar[4];
               int32_t val;
            } converter;
            converter.val = val;
            for (uint8_t i = 0; i < 4; ++i) {
               raw[i+1] = converter.ar[i];
            }
            uint16_t const ck_sum = quan::uav::fletcher16 (raw,5);
            raw[5] = static_cast<uint8_t> (ck_sum & 0xFF);
            raw[6] = static_cast<uint8_t> ((ck_sum & 0xFF00) >> 8U);
            quan::uav::cobs::encode (raw,7,encoded);
            uint8_t const frame = 0;
            app.get_sp()->write (&frame,1);
            app.get_sp()->write (encoded,8);
         }
      }
      
      static void update (uint8_t var_id)
      {
         switch (var_id) {
         case latitude_id:
            update_id (var_id, normalise (m_current_position.lat));
            return;
         case longtitude_id:
            update_id (var_id, normalise (m_current_position.lon));
            return;
         case altitude_id:
            update_id (var_id, m_current_position.alt.numeric_value() * 1000);
            return;
         default:
            // shouldnt get here
            return;
         }
      }
   };

   quan::uav::position<
      quan::angle_<int32_t>::deg10e7,
      quan::length_<int32_t>::m
      > packet::m_current_position ;

      uint8_t packet::raw[7];
      uint8_t packet::encoded[8];
}
 
// called every 83 millisec
// for nearly 1/4 sec refresh rate
void COBS_send_message()
{
   static uint8_t var_id = 0;
   if (var_id  == 0) {
      packet::get_aircraft_gps_position();
   }
   var_id = (var_id + 1) % 3;
   packet::update (var_id + 1);
}

 
 