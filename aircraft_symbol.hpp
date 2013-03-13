#ifndef AIRCRAFT_SYMBOL_HPP_INCLUDED
#define AIRCRAFT_SYMBOL_HPP_INCLUDED

#include <quan/gx/primitives/polyline.hpp>
#include <quan/two_d/vect.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/length.hpp>
#include <quan/gx/graphics_context.hpp>

// add a scaling so plane is bigger on small maps
struct aircraft_symbol{
    typedef quan::length::mm mm;
    typedef quan::two_d::vect<mm> vect2_mm;
    typedef quan::three_d::vect<mm> vect3_mm;
  aircraft_symbol(vect3_mm const & pos, quan::angle::deg const & heading)
   : m_position(pos), m_heading(heading) { }

    void draw(quan::gx::graphics_context<mm> const & gx)const
   {
      vect2_mm pos{m_position.x,m_position.y};
      quan::gx::primitives::polyline<mm> poly{mm{0.5}, quan::gx::rgb::colors::blue}; 
      for ( size_t i = 0; i < 11 ; ++i){
         poly.push_back(rotate(m_array[i],-m_heading) + pos);
      }
      gx.draw_polyline(poly);
   }
//   void set_position(quan::three_d::vect<quan::length::mm> const & p){ m_position = p;}
//   void set_headingng(quan::angle::deg b){m_bearing = b;}
private:
   static vect2_mm m_array[11];
   vect3_mm m_position;
   quan::angle::deg m_heading;
};

#endif // AIRCRAFT_SYMBOL_HPP_INCLUDED
