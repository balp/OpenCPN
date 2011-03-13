//////////////////////////////////////////////////////////////////////
//
// WX Cocoa 2.9 workarounds
//
//////////////////////////////////////////////////////////////////////
// begin rms
#include "wx/wx.h"
#include "macutils.h"

// workaround for 2.9 cocoa where the equality operator for region is currently (2011-03-09) not implemented
bool ocpn_mac_region_compare(const wxRegion& r1, const wxRegion& r2)
{
	wxCoord x1,x2,y1,y2,w1,w2,h1,h2;
	r1.GetBox(x1,y1,w1,h1);
	r2.GetBox(x2,y2,w2,h2);
	if(x1 == x2 && y1 == y2 && w1 == w2 && h1 && h2) return true;
	return false;
}
