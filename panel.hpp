#ifndef QUANTRACKER_SIM_PANEL_HPP_INCLUDED
#define QUANTRACKER_SIM_PANEL_HPP_INCLUDED


#include <wx/panel.h>
#include <wx/button.h>
#include <wx/statline.h>
#include <wx/slider.h>
#include <wx/checkbox.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

struct panel : wxPanel
{
  panel (wxWindow * parent); 
private:
   enum
   {
      idMenuQuit = 1000,
      idMenuAbout,
      idTimer,
      idBtnConnect,
      idAltitudeSlider,
      idScaleSlider
   };

   DECLARE_EVENT_TABLE()

   void OnConnectDisconnect(wxCommandEvent& event);
   void OnAircraftPositionChanged(wxEvent& event);
   void OnRemoteDistanceChanged(wxEvent& event);
   void OnRemoteElevationChanged(wxEvent& event);
   void OnRemoteBearingChanged(wxEvent& event);
   void OnAltitudeSlider(wxScrollEvent & event);
   void OnScaleSlider(wxScrollEvent & event);
 
   
   wxButton* BtnConnect;
   wxTextCtrl* PortText;
   wxTextCtrl* LatText;
   wxTextCtrl* LonText;
   wxTextCtrl* AltText;

   wxTextCtrl* LocalDistanceText;
   wxTextCtrl* LocalBearingText;
   wxTextCtrl* LocalElevationText;
  
   wxTextCtrl* RemoteDistanceText;
   wxTextCtrl* RemoteBearingText;
   wxTextCtrl* RemoteElevationText;

   wxTextCtrl* ScaleText;

   wxSlider* ScaleSlider;  
   wxSlider* AltitudeSlider;  

};


#endif // QUANTRACKER_SIM_PANEL_HPP_INCLUDED
