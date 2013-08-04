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

/*
   
   create a message in ByteStuff format
   Each message is 7 bytes long
########################################
   not very satisfactory here as Frame Character is really from the higher level protocol
   message[0] is the  Frame character
###############################
   message[1] is the message id
   message[2:5] is the value (in little endian format?)
   message[6] is the checksum

*/

#include "app.h"
#include "frsky.hpp"
#include "aircraft.hpp"
#include <quan/serial_port.hpp>
#include <quan/utility/fifo.hpp>
//
//#include <quan/uav/position.hpp>
//
//namespace {
//
//   quan::uav::position<  
//      quan::angle_<int32_t>::deg10e7,
//      quan::length_<int32_t>::m 
//   >  aircraft_gps_position;
//
//}
//
//void update_aircraft_gps_position(     quan::uav::position<  
//      quan::angle_<int32_t>::deg10e7,
//      quan::length_<int32_t>::m 
//   > const & p)
//{
//   aircraft_gps_position = p;
//}  

namespace {

   struct FrSky_msgID {
       static const uint8_t header_value =   0x5e;
       static const uint8_t escape_value =   0x5d;

       static const uint8_t GPSLAT   =       0x1; // deg to int32_t
       static const uint8_t GPSLON   =       0x2; // deg to int32_t
       static const uint8_t GPSALT   =       0x3; // cm to uint32_t
   // poss slower updates could cycle one each time or just send for testing
   // can split off data for frsky later
       static const uint8_t GPSHEADING =     0x4; // way aircraft is going
       static const uint8_t GPSSPEED  =      0x5; // groundspeed
       static const uint8_t COMPASSBEARING = 0x6; // way aircraft is pointing from Magnetometer
       static const uint8_t AIRSPEED =       0x7; // from pitot
       static const uint8_t ATTITUDE =       0x9; // 10 bits each of x,y,z
       static const uint8_t BAROALT =        0x8;
       static const uint8_t BATTERY   =      0xA; // Voltage , Current
     
   #if 0
       static const uint8_t NUM_SATS
       static const uint8_t FLIGHT_MODE
    #endif
   };

   template<typename T>
   struct create_message {
       create_message(uint8_t msg_id, T const & val);
       static const size_t length = sizeof(T) + 3;
       typedef uint8_t (&msg_buf_t)[length];
       msg_buf_t get() {
           return m_msg_buf;
       }
       create_message & operator = (T const & val);
   private:
       typedef union {
           uint8_t m_array[sizeof(T)];
           T m_raw_value;
       } converter;
       uint8_t      m_msg_buf[length];
   };
    
   /*
    len is length of buf including checksum, but checksum is last byte in buf so
   last byte not included
   */
   template <uint8_t Len>
   static uint8_t FrSky_do_checksum(uint8_t * ar)
   {
       uint8_t sum = static_cast<uint8_t>(ar[0]);
       for ( size_t i = 1; i < (Len-1); ++i) {
           uint16_t const sum1 = sum  + static_cast<uint16_t>( static_cast<uint8_t>(ar[i]));
           sum += static_cast<uint8_t>(sum1 & 0xff)  + static_cast<uint8_t>((sum1 >> 8) && 0xff);
       }
       return static_cast<uint8_t>(sum);
   }
   // sort so lat and lon are uint32_t.. currently int32_t
   // unfortunately the checksum includes the higher level protocol header value
   // we could make the checksum start with that value?
   template<typename T>
   create_message<T>::create_message(uint8_t msg_id, T const & val)
   {
       m_msg_buf[0] = FrSky_msgID::header_value;
       m_msg_buf[1] = msg_id;
    
       converter conv;
       conv.m_raw_value = val;
    
       for (size_t i = 0; i < sizeof(T); ++i) {
           m_msg_buf[2 + i] = conv.m_array[i];
       }
       m_msg_buf[length -1] = FrSky_do_checksum<length>(m_msg_buf);
   }
    
   template <typename T>
   create_message<T> & create_message<T>::operator = (T const & val)
   {
       converter conv;
       conv.m_raw_value = val;
    
       for (size_t i = 0; i < sizeof(T); ++i) {
           m_msg_buf[2 + i] = conv.m_array[i];
       }
       m_msg_buf[length -1] = FrSky_do_checksum<length>(m_msg_buf);
       return *this;
   }
    
   create_message<int32_t> lat_msg(FrSky_msgID::GPSLAT,0);
   create_message<int32_t> lon_msg(FrSky_msgID::GPSLON,0);
   create_message<int32_t> baroalt_msg(FrSky_msgID::BAROALT,0);
   create_message<int32_t> airspeed_msg(FrSky_msgID::AIRSPEED,0);
    /*
    angle is +- deg  - 180 to 180
    convert to angle of +- deg * 1e7
   */
   inline int32_t normalise_angle(quan::angle_<int32_t>::deg10e7 const & in)
   {
       constexpr int64_t ang_max  = 1800000000LL;
       return static_cast<int32_t>(in.numeric_value() % ang_max);
   }
   /* 
   max FrSky update rate is 1200 baud. If this is called at 50 Hz then
   works out that can send only 7 bytes over 3 cycles.
   so send 2, 2, then 3
    gives update rate for all variables of around 5 Hz

   or do one every 7 ms?

   The following functions are put in an array and called in rotation
     First func fro var does setup, others just put out some bytes
     called in 50 Hz loop so as not to exceed max data rate
   */
   // Latitude

   quan::fifo<uint8_t,1000> buffer;

   // write data into the buffer in the higher level bytestuff protocol
   // packet is assumed to be len bytes long
   // start_of_frame is true if its the start of a new higher level frame
   // the frame is then in
   inline int16_t esc_write_sp(uint8_t * buf, int16_t len, bool start_of_frame)
   {
     // auto & app =wxGetApp();
    //  assert(app.have_sp());
    //  auto sp = app.get_sp();
      int16_t pos = 0;
      int16_t count = 0;
      if(start_of_frame){
        // though the header is in buf[0] we know what it is so use a constant here...
        // sp->write( FrSky_msgID::header_value);
       // sp->write(buf,1);
       buffer.put(buf[0]);
        ++pos;
        ++count;
      }
      for( ; pos < len; ++pos){
          uint8_t ch = buf[pos];
          if ( (ch == FrSky_msgID::header_value) || (ch == FrSky_msgID::escape_value) ){
             uint8_t ar[]= {FrSky_msgID::escape_value, static_cast<uint8_t>(ch ^ 0x60)};
            // sp->write(ar,2);
             buffer.put(ar[0]);
             buffer.put(ar[1]);
             count += 2;
          }else{
            // sp->write(buf + pos,1);
            buffer.put(buf[pos]);
             ++count;
          }
      }
      return count;
   }

   void send_as_frsky_rx()
   {

     if ( buffer.num_in_buffer() ==0) { return; }
     uint8_t  msg_array[11] = {0x7E,0xFD,0,0,0,0,0,0,0,0,0x7E};
//     
//     msg_array[0] = 0x7E; // header
//     msg_array[1] = 0xFD; // id user frame
//     msg_array[2] = 0;   // number of user bytes
//     msg_array[3] = 0;    // unused
//     for( uint32_t i = 0; i < 6; ++i){ // pre clear user data
//        msg_array[4+i] = 0;
//     }
//     msg_array[10] = 0x7E; // end of frame
     uint8_t * const num_user_bytes = &msg_array[2];
     uint8_t * p_bytes = &msg_array[4];
     while( buffer.num_in_buffer() > 0){
       uint8_t ch;
       buffer.peek(ch);
       if ( (ch == 0x7E) || (ch == 0x7D)){
         if ( *num_user_bytes < 5){
             buffer.get(ch);
             *p_bytes++ = 0x7D;
             *p_bytes++ = ch ^ 0x20;
             *num_user_bytes +=2;
         }
         else{
            break;
         }
       }else{
          if (*num_user_bytes < 6){
            buffer.get(ch);
            *p_bytes++ = ch;
            ++ *num_user_bytes;
          }else{
             break;
          }
       }
     }
     auto & app =wxGetApp();
     assert(app.have_sp());
     app.get_sp()->write(msg_array,11);
   }
   //return actual num of uint8_ts written in escapes
   int16_t update_lat_msg1()
   {
       lat_msg = normalise_angle(get_aircraft_gps_position().lat);
       return esc_write_sp(lat_msg.get(), 2, true);
   }
   int16_t update_lat_msg2()
   {
       return esc_write_sp(lat_msg.get() + 2, 2, false);
   }
   int16_t update_lat_msg3()
   {
      return esc_write_sp(lat_msg.get() + 4, 3, false);
   }

   //longtitude
   int16_t update_lon_msg1()
   {
      lon_msg = normalise_angle(get_aircraft_gps_position().lon);
      return esc_write_sp(lon_msg.get(), 2, true);
   }

   int16_t update_lon_msg2()
   {
      return esc_write_sp(lon_msg.get() + 2, 2, false);
   }
   int16_t update_lon_msg3()
   {
      return esc_write_sp(lon_msg.get() + 4, 3, false);
   }

   //altitude
   int16_t update_baroalt_msg1()
   {
      baroalt_msg = static_cast<int32_t>(get_aircraft_gps_position().alt.numeric_value());
      return esc_write_sp(baroalt_msg.get(),2,true);
   }
   int16_t update_baroalt_msg2()
   {
      return esc_write_sp(baroalt_msg.get() + 2, 2, false);
   }
   int16_t update_baroalt_msg3()
   {
      return esc_write_sp(baroalt_msg.get() + 4, 3,false);
   }

   //airspeed
   int16_t update_airspeed_msg1()
   {

//TODO#########################
        // airspeed_msg = static_cast<int32_t>(the_aircraft.airspeed.numeric_value());
      airspeed_msg = static_cast<int32_t>(0);
//##########################
     return esc_write_sp(airspeed_msg.get(),2,true);
   }

   int16_t update_airspeed_msg2()
   {
      return esc_write_sp(airspeed_msg.get() + 2, 2, false);
   }
   int16_t update_airspeed_msg3()
   {
      return esc_write_sp(airspeed_msg.get() + 4, 3,false);
   }

   typedef int16_t(*msgfun)();
    
   msgfun msgfuns[] = {
       update_lat_msg1,
       update_lat_msg2,
       update_lat_msg3,
       update_lon_msg1,
       update_lon_msg2,
       update_lon_msg3,
       update_baroalt_msg1,
       update_baroalt_msg2,
       update_baroalt_msg3,
       update_airspeed_msg1,
       update_airspeed_msg2,
       update_airspeed_msg3
   };

}//namespace

// call functions to output data in sequence
// called at 1 call every 1/50th sec
void FrSky_send_message()
{

    static uint8_t idx = 0;
    static uint8_t byte_idx = 0;
    static int16_t write_count = 0;
    
    while (write_count <= 0 ){ // ready for more comms
       // call current fun
       write_count += msgfuns[idx]();
       //and update to next fun
       idx = (idx + 1) % (sizeof(msgfuns)/sizeof(msgfun));
    }
    // number of bytes allowed is 7  per 60 msec ( each call happens every 20 msec)
   // so split the bytes into 2,2,3 allowed each time.
    byte_idx = (byte_idx + 1 ) % 3; 
    int const bytes_gone = (byte_idx == 2)?3:2;
    write_count -= bytes_gone;
    // cap the read ahead...
    if( write_count < -64){
       write_count = -64;
    }

    while ( buffer.num_in_buffer()){
        send_as_frsky_rx();
      // uint8_t ch;
      // buffer.get(ch);
    }

}
