
/***************************************************************************
 *
 * Project:  OpenCP
 * Purpose:  Generic Comport Handler
 * Author:   David Register
 *
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register   *
 *   bdbcat@yahoo.com   *
 *   Copyright (C) 2012 by Anders Arnholm                                  *
 *   Anders@Arnholm.se                                                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,  USA.         *
 ***************************************************************************
 *
 *
 *
 *
 */


#ifndef __COMPORT_H__
#define __COMPORT_H__

#include "wx/wxprec.h"

#ifndef  WX_PRECOMP
  #include "wx/wx.h"
#endif //precompiled header

#include "dychart.h"
#include "chart1.h"

#ifndef OCPN_NO_SOCKETS
#ifdef __WXGTK__
// newer versions of glib define its own GSocket but we unfortunately use this
// name in our own (semi-)public header and so can't change it -- rename glib
// one instead
#include <gtk/gtk.h>
#define GSocket GlibGSocket
#endif

#include "wx/socket.h"
#endif
#include <wx/datetime.h>

#ifdef __POSIX__
#include <sys/termios.h>
#endif

#define AIS_SOCKET_ID             7

enum
{
    EVT_AIS_DIRECT,
    EVT_AIS_PARSE_RX
};


//----------------------------------------------------------------------------
// AISEvent
//----------------------------------------------------------------------------

class OCPN_AISEvent: public wxEvent
{
      public:
            OCPN_AISEvent( wxEventType commandType = wxEVT_NULL, int id = 0 );

            OCPN_AISEvent(const OCPN_AISEvent & event)
            : wxEvent(event),
                m_NMEAstring(event.m_NMEAstring),
                m_extra(event.m_extra)
                { }

                ~OCPN_AISEvent( );

    // accessors
            wxString GetNMEAString() { return m_NMEAstring; }
            void SetNMEAString(wxString string) { m_NMEAstring = string; }

            void SetExtraLong(long n){ m_extra = n;}
            long GetExtraLong(){ return m_extra;}

    // required for sending with wxPostEvent()
            wxEvent *Clone() const;

      private:
            wxString    m_NMEAstring;
            long        m_extra;

};

//    DECLARE_EVENT_TYPE(wxEVT_OCPN_AIS, -1)
extern /*expdecl*/ const wxEventType wxEVT_OCPN_AIS;



namespace OpenCPN
{
    namespace Utils
    {
	/**
	 * Event when something happend on a nmea port.
	 */
	class OCPN_PortEvent: public wxEvent
	{
	    public:
		OCPN_PortEvent( wxEventType commandType = wxEVT_NULL, int id = 0 );
		OCPN_PortEvent( const OCPN_PortEvent& event )
		    : wxEvent(event),
		    m_NMEAstring(event.m_NMEAstring),
		    m_extra(event.m_extra)
		    { }
                ~OCPN_PortEvent( );
		wxString GetNMEAString() { return m_NMEAstring; }
		void SetNMEAString(wxString string) { m_NMEAstring = string; }
		void SetExtraLong(long n){ m_extra = n;}
		long GetExtraLong(){ return m_extra;}
		wxEvent *Clone() const;
	    private:
		wxString    m_NMEAstring;
		long        m_extra;


	};
	/**
	 * Com port description
	 */
	class OpenCommPortElement
	{
	    public:
		wxString    com_name;
		int         port_descriptor;
		int         n_open;
		int         baud_rate;
	};

	WX_DECLARE_LIST(OpenCommPortElement, ListOfOpenCommPorts);

	/**
	 * A generic com port handl
	 */
	class ComPortManager:  public wxEvtHandler
	{
	    public:
		/**
		 * Constructor
		 */
		ComPortManager();
		~ComPortManager();

		bool OpenTcpPort(wxFrame *pParent, wxEvtHandler* pListener, const wxString& dataSource);
		int OpenComPort(wxString &com_name, int baud_rate);
		OpenCommPortElement *GetComPort(wxString &com_name);
		int CloseComPort(int fd);

		int WriteComPort(wxString& com_name, const wxString& string);
		int WriteComPort(wxString& com_name, unsigned char *msg, int count);
		int ReadComPort(wxString& com_name, int count, unsigned char *p);
		bool SerialCharsAvail(wxString& com_name);


		bool GetLogFlag(){ return m_blog; }
		void SetLogFlag(bool flag){ m_blog = flag; }

	    private:
		void OnSocketEvent(wxSocketEvent& event);
		int OpenComPortPhysical(wxString &com_name, int baud_rate);
		int CloseComPortPhysical(int fd);
		int WriteComPortPhysical(int port_descriptor, const wxString& string);
		int WriteComPortPhysical(int port_descriptor, unsigned char *msg, int count);

		int ReadComPortPhysical(int port_descriptor, int count, unsigned char *p);
		bool CheckComPortPhysical(int port_descriptor);

		ListOfOpenCommPorts     m_port_list;

		bool        m_blog;
		bool              m_busy;
		wxEvtHandler* tcpListener;
#ifndef OCPN_NO_SOCKETS
		wxIPV4address     addr;
		wxSocketClient    *m_sock;
#endif
		DECLARE_EVENT_TABLE()

	};

    };
};

#endif // __COMPORT_H__
