

#include <cstdint>
#include <cassert>
#include <cstring>

#include <quan/uav/cobs/protocol.hpp>
#include <quan/uav/position.hpp>
#include <quan/utility/fifo.hpp>
#include <quan/uav/fletcher16.hpp>
#include "frsky.hpp"
#include "aircraft.hpp"
#include "app.h"

namespace {
 
      // shorter packets are very good but
      // penalty for corrupt data getting through is very high since may send tracker pointing in wrong direction
      // penalty for missed packet is relatively low since tracker wil remain pointing in same direction
      // so need to make chance of invalid data getting through very small
      // so use 16 bit checksum as well as other checks

      static bool write_as_FrSky = true; // true to wrap data in FrSky protocol
      // so emulating the output from FrSky transmitter serial port

      struct packet {

         // the number of uint8_t data elements in the full packet
         static uint8_t const  num_elements = 11;

         // Send all the data
         // The first byte is 8
         // followed by 11 data bytes
         // followed by a 2 byte checksum
         // then encoded using COBS
         // and a leading zero added
         static ssize_t update_full(uint8_t* src)
         {
            uint8_t unencoded[14] = {8U};// id for a full data packet
            memcpy(unencoded + 1,src,11);
            uint16_t const ck =  quan::uav::fletcher16(unencoded,12);
            unencoded[12] = static_cast<uint8_t>(ck & 0xFF);
            unencoded[13] = static_cast<uint8_t>((ck >> 8) & 0xFF);
            uint8_t encoded[16]= {0}; // framing byte
            quan::uav::cobs::encode(unencoded,14,encoded + 1);
            return write(encoded,16);
         }

         // How the data is written depends on whether we are emulating
         // the FrSky telemetry output from RcTx
         static ssize_t write( uint8_t * src, uint8_t num)
         {
              auto & app =wxGetApp();
               assert(app.have_sp());
               auto sp = app.get_sp();
            if (write_as_FrSky == false){ // either write direct or via FrSky RcTx telem output protocol
               return  sp->write(src,num);
            }else{
               quan::fifo<uint8_t,16> fifo;
               // naff should be an array but spose its only needed on PC
               for ( uint8_t i = 0; i < num; ++i){
                  fifo.put(src[i]);
               }
               return send_as_frsky_Telemetry_from_RcTx(fifo, app.get_sp());
            }
         }

         static bool is_odd(uint8_t val){ return (val & 1 ) == 1;}
   
         /*
            The current position state is serialised into a local array
            Each element is checked in order to see if its changed from last update
            if the element is changed its index is appended to the message . 
            The stateful old local array, m_cur_array is also
            updated with the new value
            The index is packed into 4 bits, the first being the high nibble of byte 0)
            The low nibble of byte 0 holds the number of bytes to update
            After all the nibbles representing the indices ( and any one nibble padding of last byte of indices)
            the data bytes are appended to the messsage
            After the data is a 2 byte checksum.
            However if more than 7 elements require updating then 
            the whole data is sent using the update_full function
            Periodically each byte in the array is sent regardless of wheteher its modified
            in case e.g remote was reset.
            In that case remote data will be valid after num_elements calls to update
      */
         static ssize_t update ( quan::uav::position<  
            quan::angle_<int32_t>::deg10e7,
            quan::length_<int32_t>::m 
         > const & p)
         { // (little endian)
             uint8_t new_array[] = {    
               static_cast<uint8_t>(p.lat.numeric_value() & 0xFF),
               static_cast<uint8_t>((p.lat.numeric_value() >> 8  ) & 0xFF),
               static_cast<uint8_t>((p.lat.numeric_value() >> 16 ) & 0xFF),
               static_cast<uint8_t>((p.lat.numeric_value() >> 24 ) & 0xFF),

               static_cast<uint8_t>(p.lon.numeric_value() & 0xFF),
               static_cast<uint8_t>((p.lon.numeric_value() >> 8  ) & 0xFF ),
               static_cast<uint8_t>((p.lon.numeric_value() >> 16 ) & 0xFF) ,
               static_cast<uint8_t>((p.lon.numeric_value() >> 24 ) & 0xFF),
         
    // may need to add a check that high bit of height is negative if number is negative
    // will only apply for huge negative heights and prob an error then
               static_cast<uint8_t>(p.alt.numeric_value() & 0xFF),
               static_cast<uint8_t>((p.alt.numeric_value() >> 8  ) & 0xFF),
               static_cast<uint8_t>((p.alt.numeric_value() >> 16 ) & 0xFF)
            };

            // update the values regardless in rotation.
            // each value gets updated at least 
            // once every num_elements passes
            // gives a 2 sec latency if invalid mind
            static uint32_t periodic_update_idx = num_elements - 1;
            periodic_update_idx = (periodic_update_idx + 1 ) % num_elements;

            // buffer to build the unencoded message
            uint8_t buf[14] = {0}; // number of byte idx to update in buf[0]
            uint8_t idx = 0; // index to buf

            // data_buf holds the sequence of data to be updated
            // the index of each is put in the message first
            // the data is appended at the end
            uint8_t data_buf[7];
            uint8_t num_data_changed = 0;
            for ( uint8_t i = 0; i < num_elements; ++i){
               // look for data thats has changed
               if ( (new_array[i] != m_cur_array[i]) || (i == periodic_update_idx ) ){
                  m_cur_array[i] = new_array[i];
                  if ( num_data_changed == 7) { 
                     // 7 + 1 already changed so its  more efficient to send the full packet
                     // but first blind copy all the nonchecked new stuff to current array
                     auto const num_left_to_copy = num_elements - ( i + 1 );
                     if ( num_left_to_copy != 0){
                        memcpy( m_cur_array + i + 1,new_array + i + 1,num_left_to_copy);
                     }
                     return  update_full(m_cur_array);
                  }else{ // less than 7 changed so far
                     // temporarily store the data items in the data_buf
                     data_buf[num_data_changed++] = m_cur_array[i];
                     if ( is_odd(num_data_changed)){
                        buf[idx] |= static_cast<uint8_t>(i << 4);
                     }else{
                        buf[++idx] = i;
                     }
                  }
               }   
            }
            // Done. Put the number of items in 
            // remembering high nibble of buf[0] already contains offset of 1st data in array
            buf[0] |= num_data_changed;
            // move to next byte after the last data indices
            ++idx;
            //  and put the changed data into the buffer
            memcpy(buf + idx ,data_buf,num_data_changed);
            idx += num_data_changed;
            // do checksum
            uint16_t const ck = quan::uav::fletcher16(buf,idx);
            buf[idx++] = static_cast<uint8_t>(ck & 0xFF);
            buf[idx++] = static_cast<uint8_t>((ck >> 8) & 0xFF);
            // encode
            uint8_t encoded[16] = {0}; // framing byte
            quan::uav::cobs::encode(buf,idx,encoded+1);

            return write(encoded,idx+2);
         }

         static uint8_t m_cur_array[11];
      };

      uint8_t packet::m_cur_array[11];
    
}// namespace

// if called every 50th sec
void COBS_send_message()
{
    // e.g called every 1/5th sec 
    // dependent on baud rate etc
    static uint8_t count =0;
    if ( ++count == 10){
      count = 0;
      packet::update(get_aircraft_gps_position());
    }
}
