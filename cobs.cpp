

#include <cstdint>
#include <cassert>
#include <cstring>

#include <quan/uav/cobs/protocol.hpp>
#include <quan/uav/position.hpp>
#include "frsky.hpp"
#include "aircraft.hpp"
#include "app.h"

namespace {
 
      // shorter packets are very good!
      // penalty for corrupt data getting through is very high since may send tracker pointing in wrong direction
      // penalty for missed packet is relatively low since tracker wil remain pointing same direction
      // so need to make chance of data getting through very small
      // prob want a longer checksum
      uint8_t checksum(uint8_t * ar, uint8_t length)
      {
         uint8_t sum = ar[0];
          for (  uint8_t i = 1; i < length; ++i) {
              uint16_t const sum1 = sum + static_cast<uint16_t>( ar[i]);
              sum += static_cast<uint8_t>( (sum1 & 0xff) + ( (sum1 >> 8) && 0xff ) );
          }
          return sum;
      }

      struct packet {

         static uint8_t const  num_elements = 11;

         static ssize_t update_full(uint8_t* src)
         {
            uint8_t unencoded[13] = {6U};// id for a full data packet
            memcpy(unencoded + 1,src,11);
            unencoded[12] = checksum(unencoded,12);
            uint8_t encoded[15]= {0}; // framing byte
            quan::uav::cobs::encode(unencoded,13,encoded+1);
            return write(encoded,15);
         }

         //make a switch for  either direct or FrSky RCTx proto opt
         static ssize_t write( uint8_t * src, uint8_t num)
         {
            if (true){ // either write direct or via FrSky RcTx telem output protocol
               auto & app =wxGetApp();
               assert(app.have_sp());
               auto sp = app.get_sp();
               return  sp->write(src,num);
            }else{
               quan::fifo<uint8_t,16> fifo;
               for ( uint8_t i = 0; i < num; ++i){
                  fifo.put(src[i]);
               }
               auto & app =wxGetApp();
               assert(app.have_sp());
               return send_as_frsky_Telemetry_from_RcTx(fifo, app.get_sp());
            }
         }
   
         static ssize_t update ( quan::uav::position<  
            quan::angle_<int32_t>::deg10e7,
            quan::length_<int32_t>::m 
         > const & p)
         {
             uint8_t new_array[] = {    
               static_cast<uint8_t>(p.lat.numeric_value() & 0xFF),
               static_cast<uint8_t>((p.lat.numeric_value() & 0xFF00) >> 8),
               static_cast<uint8_t>((p.lat.numeric_value() & 0xFF0000) >> 16),
               static_cast<uint8_t>((p.lat.numeric_value() & 0xFF000000) >> 24),

               static_cast<uint8_t>(p.lon.numeric_value() & 0xFF),
               static_cast<uint8_t>((p.lon.numeric_value() & 0xFF00) >> 8),
               static_cast<uint8_t>((p.lon.numeric_value() & 0xFF0000) >> 16),
               static_cast<uint8_t>((p.lon.numeric_value() & 0xFF000000) >> 24),
         
               static_cast<uint8_t>(p.alt.numeric_value() & 0xFF),
               static_cast<uint8_t>((p.alt.numeric_value() & 0xFF00) >> 8),
               static_cast<uint8_t>((p.alt.numeric_value() & 0xFF0000) >> 16)
            };

            // update the values anyway in rotation
            // each value gets updated at least 
            // once every num_elements passes
            static uint32_t cur_update_idx = num_elements - 1;
            cur_update_idx = (cur_update_idx + 1 ) % num_elements;

            uint8_t buf[12] = {0}; // number of byte idx to update in buf[0]
            uint8_t idx = 1; // start of data

            for ( uint8_t i = 0; i < num_elements; ++i){
               if ( (new_array[i] != m_cur_array[i]) || (i == cur_update_idx )  ){
                  m_cur_array[i] = new_array[i];
                  // increment the number of values found changed
                  if ( ++buf[0] == 6) { 
                     // if more than 5 changed its  more efficient to do the full packet
                     // but first blind copy all the nonchecked new stuff to old
                     auto num_left_to_copy = num_elements - (i+1);
                     if ( num_left_to_copy != 0){
                        memcpy( m_cur_array + i + 1,new_array + i + 1,num_left_to_copy);
                     }
                     // send all the data
                     return  update_full(m_cur_array);
                  }else{
                     
                     buf[idx++] = static_cast<uint8_t>(i); // channels are 1 based
                     buf[idx++] = m_cur_array[i]; 
                  }
               }   
            }
            // if here send any values that changed
            // length is 1 for num changed vals, + 2 * num changed vals + 1 for cksum  
             uint8_t const data_len = buf[0] * 2 + 2;
             buf[data_len-1] = checksum(buf,data_len-1);
             uint8_t encoded[15] = {0}; // framing byte
             quan::uav::cobs::encode(buf,data_len,encoded+1);
             return write(encoded,data_len + 2);
         }
        
         static uint8_t m_cur_array[11];
      };

      uint8_t packet::m_cur_array[11];
    
}// namespace

// if called every 50th sec
void COBS_send_message()
{
    // called every 1/5th sec for 1200 baud update to be ok
    static uint8_t count =0;
    if ( ++count == 10){
      count = 0;
      packet::update(get_aircraft_gps_position());
    }
}