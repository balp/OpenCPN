/******************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  Shared Comport implementaiton
 * Author:   David Register
 * Author:   Anders Arnholm
 *
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register   *
 *   bdbcat@yahoo.com   *
 *   Copyright (C) 2012 by Anders Arnholm <Anders@Arnholm.se>              *
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
 */
#include "wx/wxprec.h"

#ifndef  WX_PRECOMP
  #include "wx/wx.h"
#endif //precompiled headers

#include "wx/tokenzr.h"
#include <wx/datetime.h>

#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "comport.h"
#include "nmea.h"           // for DNSTestThread

extern  int             s_dns_test_flag;
BEGIN_EVENT_TABLE(OpenCPN::Utils::ComPortManager, wxEvtHandler)
  EVT_SOCKET(AIS_SOCKET_ID, OpenCPN::Utils::ComPortManager::OnSocketEvent)
END_EVENT_TABLE()

namespace OpenCPN
{
namespace Utils
{

//-------------------------------------------------------------------------------------------------------------
//
//    Communications Port Manager
//
//-------------------------------------------------------------------------------------------------------------
#include <wx/listimpl.cpp>
WX_DEFINE_LIST(ListOfOpenCommPorts);
WX_DEFINE_LIST(ListOfTCPPorts);
WX_DEFINE_LIST(ListOfEvtHandlers);



ComPortManager:: ComPortManager()
      : m_blog(true),
      tcpListener(NULL)
{
}

ComPortManager::~ComPortManager()
{
}

//    Common Methods

bool ComPortManager::OpenTcpPort(wxFrame *pParent, wxEvtHandler* pListener, const wxString& dataSource)
{
      TCPPortElement* port = GetTcpPort(dataSource);
      if(pe != NULL) {
	    pe->listeners.Append(pListener);
      } else {
	    long ais_port = 3047; // GPSD Known port number
	    tcpListener = pListener;

	    wxString AIS_data_ip;
	    AIS_data_ip = dataSource.Mid(7);         // extract the IP
	    int portIndex = AIS_data_ip.Find(wxChar(':'), true);
	    if(portIndex != wxNOT_FOUND) { // have hostname:port
		  wxString portStr = AIS_data_ip.Mid(portIndex + 1);
		  portStr.ToLong(&ais_port);
		  wxString msg(_T("AIS Data Source port...."));
		  msg.Append(portStr);
		  wxLogMessage(msg);
		  AIS_data_ip = AIS_data_ip.Mid(0, portIndex);
	    }
	    wxString msg(_T("AIS Data Source is TCP/IP...."));
	    msg.Append(AIS_data_ip);
	    wxLogMessage(msg);

	    // Create the socket
	    m_sock = new wxSocketClient();

	    // Setup the event handler and subscribe to most events
	    m_sock->SetEventHandler(*this, AIS_SOCKET_ID);

	    m_sock->SetNotify(wxSOCKET_CONNECTION_FLAG |
			wxSOCKET_INPUT_FLAG |
			wxSOCKET_LOST_FLAG);
	    m_sock->Notify(TRUE);

	    m_busy = FALSE;

	    DNSTestThread *ptest_thread = NULL;
	    ptest_thread = new DNSTestThread(AIS_data_ip);

	    ptest_thread->Run();                      // Run the thread from ::Entry()


	    //    Sleep and loop for N seconds
#define SLEEP_TEST_SEC  2

	    for(int is=0 ; is<SLEEP_TEST_SEC * 10 ; is++)
	    {
		  wxMilliSleep(100);
		  if(s_dns_test_flag)
			break;
	    }

	    if(!s_dns_test_flag)
	    {

		  wxString msg(AIS_data_ip);
		  msg.Prepend(_("Could not resolve TCP/IP host '"));
		  msg.Append(_("'\n Suggestion: Try 'xxx.xxx.xxx.xxx' notation"));
		  OCPNMessageDialog md(pParent, msg, _("OpenCPN Message"), wxICON_ERROR );
		  md.ShowModal();

		  m_sock->Notify(FALSE);
		  m_sock->Destroy();
		  m_sock = NULL;

		  return false;
	    }


	    //      Resolved the name, somehow, so Connect() the socket
	    wxString msg2(_T("Connect to "));
	    msg2.Append(AIS_data_ip);
	    msg2.Append(wxString::Format(_(" port %d "), ais_port));
	    wxLogMessage(msg2);

	    TCPPortElement* port = new TCPPortElement;
	    port->name = dataSource;
	    port->listeners.Append(pListener);

	    addr.Hostname(AIS_data_ip);
	    addr.Service(ais_port);
	    if(m_sock->Connect(addr, FALSE))       // Non-blocking connect
	    {
		  wxString msg(_T("Connect returned OK...."));
		  wxLogMessage(msg);
		  port->connected = true;
	    } else {
		  wxString msg(_T("Connect returned False, still not error...."));
		  wxLogMessage(msg);
		  port->connected = false;
	    }
	    m_tcp_list.Append(port);

      }
      return true;
}

TCPPortElement *ComPortManager::GetTcpPort(const wxString &name)
{
      for ( ListOfTCPPorts::Node *node = m_tcp_list.GetFirst(); node; node = node->GetNext() )
      {
            TCPPortElement *current = node->GetData();

            if(current->name.IsSameAs(name))
                  return current;
      }

      return NULL;
}

int ComPortManager::OpenComPort(wxString &com_name, int baud_rate)
{
      // Already open?
      int port_descriptor;
      OpenCommPortElement *pe = GetComPort(com_name);
      if(NULL == pe)
      {
            port_descriptor = OpenComPortPhysical(com_name, baud_rate);
            if( port_descriptor < 0)
                  return port_descriptor;                                // error

            OpenCommPortElement *pocpe = new OpenCommPortElement;
            pocpe->com_name = com_name;
            pocpe->port_descriptor = port_descriptor;
            pocpe->n_open = 1;
            pocpe->baud_rate = baud_rate;

            m_port_list.Append(pocpe);

            if(m_blog)
            {
                  wxString s;
                  s.Printf(_T("OpenPD: %d, new_count = %d for "), port_descriptor, pocpe->n_open );
                  s.Append(com_name);
                  wxLogMessage(s);
            }

      }
      else                    // port is already open, so increment its counter
      {
            pe->n_open++;
            port_descriptor = pe->port_descriptor;

            if(m_blog)
            {
                  wxString s;
                  s.Printf(_T("Re-OpenPD: %d, new_count = %d for "), port_descriptor, pe->n_open );
                  s.Append(com_name);
                  wxLogMessage(s);
            }
      }



      return port_descriptor;
}

int ComPortManager::CloseComPort(int fd)
{

      for ( ListOfOpenCommPorts::Node *node = m_port_list.GetFirst(); node; node = node->GetNext() )
      {
            OpenCommPortElement *current = node->GetData();

            if(current->port_descriptor == fd)
            {
                  current->n_open--;

                  if(m_blog)
                  {
                        wxString s;
                        s.Printf(_T("ClosePD: %d, count_after_close = %d for "), fd, current->n_open );
                        s.Append(current->com_name);
                        wxLogMessage(s);
                  }
                  if(0 == current->n_open)
                  {
                        CloseComPortPhysical(fd);
                        if(m_blog)
                              wxLogMessage(_T("  and so CloseComPortPhysical"));

                        m_port_list.DeleteObject(current);
                        delete current;
                        break;
                  }
            }
      }

      return 0;
}

//------------------------------------------------------------
//    GetComPort()
//    Return the descriptor for an already open com port.
//    return -1 if the port is not already open
//------------------------------------------------------------

OpenCommPortElement *ComPortManager::GetComPort(const wxString &com_name)
{
      for ( ListOfOpenCommPorts::Node *node = m_port_list.GetFirst(); node; node = node->GetNext() )
      {
            OpenCommPortElement *current = node->GetData();

            if(current->com_name.IsSameAs(com_name))
                  return current;
      }

      return NULL;
}

int ComPortManager::WriteComPort(wxString& com_name, const wxString& string)
{
      int port_descriptor;
      int status;

      OpenCommPortElement *pe = GetComPort(com_name);

      if(NULL == pe)
            port_descriptor = OpenComPort(com_name, 4800);              // defaults to 4800
      else
            port_descriptor = pe->port_descriptor;

            status = WriteComPortPhysical(port_descriptor, string);

      return status;
}


int ComPortManager::WriteComPort(wxString& com_name, unsigned char *msg, int count)
{
      int port_descriptor;
      int status;

      OpenCommPortElement *pe = GetComPort(com_name);

      if(NULL == pe)
            port_descriptor = OpenComPort(com_name, 4800);              // defaults to 4800
      else
            port_descriptor = pe->port_descriptor;

      status = WriteComPortPhysical(port_descriptor, msg, count);

      return status;
}

int ComPortManager::ReadComPort(wxString& com_name, int count, unsigned char *p)
{
      int port_descriptor;

      OpenCommPortElement *pe = GetComPort(com_name);

      if(NULL == pe)
            return 0;
      else
            port_descriptor = pe->port_descriptor;

      return ReadComPortPhysical(port_descriptor, count, p);


}

bool ComPortManager::SerialCharsAvail(wxString& com_name)
{
      int port_descriptor;

      OpenCommPortElement *pe = GetComPort(com_name);

      if(NULL == pe)
            return false;
      else
            port_descriptor = pe->port_descriptor;

      return CheckComPortPhysical(port_descriptor);
}

void ComPortManager::OnSocketEvent(wxSocketEvent& event)
{
      printf("ComPortManager::OnSocketEvent()\n");
#ifndef OCPN_NO_SOCKETS

#define RD_BUF_SIZE    200

      if(tcpListener)
      {

	    switch(event.GetSocketEvent())
	    {
		  case wxSOCKET_INPUT :                     // from  Daemon
			{
			      char *bp;
			      char buf[RD_BUF_SIZE + 1];
			      int char_count;
			      m_sock->SetFlags(wxSOCKET_NOWAIT);


			      //          Read the reply, one character at a time, looking for 0x0a (lf)
			      bp = buf;
			      char_count = 0;

			      while (char_count < RD_BUF_SIZE)
			      {
				    m_sock->Read(bp, 1);
				    if(*bp == 0x0a)
				    {
					  bp++;
					  break;
				    }

				    bp++;
				    char_count++;
			      }

			      *bp = 0;                        // end string

			      //          Validate the string

			      if(!strncmp((const char *)buf, "!AIVDM", 6))
			      {
				    //                  Decode(buf);

				    //    Signal the main program thread

				    OCPN_AISEvent event(wxEVT_OCPN_AIS , ID_AIS_WINDOW );
				    event.SetEventObject( (wxObject *)this );
				    event.SetExtraLong(EVT_AIS_PARSE_RX);
				    event.SetNMEAString(wxString(buf,  wxConvUTF8));
				    tcpListener->AddPendingEvent(event);
			      }



			}
		  case wxSOCKET_LOST       :
		  case wxSOCKET_CONNECTION :
		  default                  :
			break;
	    }
      }
#endif
}



#ifdef __POSIX__
typedef struct {
      int fd;         /* File descriptor */
      struct termios gps_ttysave;
} posix_serial_data;

int ComPortManager::OpenComPortPhysical(wxString &com_name, int baud_rate)
{

    // Declare the termios data structures
      termios ttyset_old;
      termios ttyset;

    // Open the serial port.
      int com_fd;
      if ((com_fd = open(com_name.mb_str(), O_RDWR|O_NONBLOCK|O_NOCTTY)) < 0)
//      if ((com_fd = open(com_name.mb_str(), O_RDWR|O_NOCTTY)) < 0)
            return com_fd;


      speed_t baud_parm;
      switch(baud_rate)
      {
            case 4800:
                  baud_parm = B4800;
                  break;
            case 9600:
                  baud_parm = B9600;
                  break;
            case 38400:
                  baud_parm = B38400;
                  break;
            default:
                  baud_parm = B4800;
                  break;
      }



     if (isatty(com_fd) != 0)
      {
            /* Save original terminal parameters */
            if (tcgetattr(com_fd,&ttyset_old) != 0)
                  return -128;

            memcpy(&ttyset, &ttyset_old, sizeof(termios));

      //  Build the new parms off the old

      //  Baud Rate
            cfsetispeed(&ttyset, baud_parm);
            cfsetospeed(&ttyset, baud_parm);

            tcsetattr(com_fd, TCSANOW, &ttyset);

      // Set blocking/timeout behaviour
            memset(ttyset.c_cc,0,sizeof(ttyset.c_cc));
            ttyset.c_cc[VTIME] = 5;                        // 0.5 sec timeout
            fcntl(com_fd, F_SETFL, fcntl(com_fd, F_GETFL) & !O_NONBLOCK);

      // No Flow Control

            ttyset.c_cflag &= ~(PARENB | PARODD | CRTSCTS);
            ttyset.c_cflag |= CREAD | CLOCAL;
            ttyset.c_iflag = ttyset.c_oflag = ttyset.c_lflag = (tcflag_t) 0;

            int stopbits = 1;
            char parity = 'N';
            ttyset.c_iflag &=~ (PARMRK | INPCK);
            ttyset.c_cflag &=~ (CSIZE | CSTOPB | PARENB | PARODD);
            ttyset.c_cflag |= (stopbits==2 ? CS7|CSTOPB : CS8);
            switch (parity)
            {
                  case 'E':
                        ttyset.c_iflag |= INPCK;
                        ttyset.c_cflag |= PARENB;
                        break;
                  case 'O':
                        ttyset.c_iflag |= INPCK;
                        ttyset.c_cflag |= PARENB | PARODD;
                        break;
            }
            ttyset.c_cflag &=~ CSIZE;
            ttyset.c_cflag |= (CSIZE & (stopbits==2 ? CS7 : CS8));
            if (tcsetattr(com_fd, TCSANOW, &ttyset) != 0)
                  return -129;

            tcflush(com_fd, TCIOFLUSH);
      }

      return com_fd;
}


int ComPortManager::CloseComPortPhysical(int fd)
{

      close(fd);

      return 0;
}


int ComPortManager::WriteComPortPhysical(int port_descriptor, const wxString& string)
{
      ssize_t status;
      status = write(port_descriptor, string.mb_str(), string.Len());

      return status;
}

int ComPortManager::WriteComPortPhysical(int port_descriptor, unsigned char *msg, int count)
{
      ssize_t status;
      status = write(port_descriptor, msg, count);

      return status;
}

int ComPortManager::ReadComPortPhysical(int port_descriptor, int count, unsigned char *p)
{
//    Blocking, timeout protected read of one character at a time
//    Timeout value is set by c_cc[VTIME]

      return read(port_descriptor, p, count);            // read of (count) characters
}


bool ComPortManager::CheckComPortPhysical(int port_descriptor)
{
      fd_set rec;
      struct timeval t;
//      posix_serial_data *psd = (posix_serial_data *)port_descriptor;
//      int fd = psd->fd;

      int fd = port_descriptor;
      FD_ZERO(&rec);
      FD_SET(fd,&rec);

      t.tv_sec  = 0;
      t.tv_usec = 1000;
      (void) select(fd+1,&rec,NULL,NULL,&t);
      if(FD_ISSET(fd,&rec))
            return true;

      return false;
}


#endif            // __POSIX__

#ifdef __WXMSW__
int ComPortManager::OpenComPortPhysical(wxString &com_name, int baud_rate)
{

//    Set up the serial port
      wxString xcom_name = com_name;
      xcom_name.Prepend(_T("\\\\.\\"));                  // Required for access to Serial Ports greater than COM9

#ifdef SERIAL_OVERLAPPED
      DWORD open_flags = FILE_FLAG_OVERLAPPED;
#else
      DWORD open_flags = 0;
#endif

      HANDLE hSerialComm = CreateFile(xcom_name.fn_str(),      // Port Name
                                 GENERIC_READ | GENERIC_WRITE,     // Desired Access
                                 0,                               // Shared Mode
                                 NULL,                            // Security
                                 OPEN_EXISTING,             // Creation Disposition
                                 open_flags,
                                 NULL);

      if(hSerialComm == INVALID_HANDLE_VALUE)
      {
            return (0 - abs((int)::GetLastError()));
      }

      if(!SetupComm(hSerialComm, 1024, 1024))
      {
            return (0 - abs((int)::GetLastError()));
      }


      DCB dcbConfig;

      if(GetCommState(hSerialComm, &dcbConfig))           // Configuring Serial Port Settings
      {
            dcbConfig.BaudRate = baud_rate;
            dcbConfig.ByteSize = 8;
            dcbConfig.Parity = NOPARITY;
            dcbConfig.StopBits = ONESTOPBIT;
            dcbConfig.fBinary = TRUE;
            dcbConfig.fRtsControl = RTS_CONTROL_ENABLE;
            dcbConfig.fDtrControl = DTR_CONTROL_ENABLE;
            dcbConfig.fOutxDsrFlow = false;
            dcbConfig.fOutxCtsFlow = false;
            dcbConfig.fDsrSensitivity = false;
            dcbConfig.fOutX = false;
	      dcbConfig.fInX = false;
	      dcbConfig.fInX = false;
	      dcbConfig.fInX = false;
	}

      else
      {
            return (0 - abs((int)::GetLastError()));
      }

      if(!SetCommState(hSerialComm, &dcbConfig))
      {
            return (0 - abs((int)::GetLastError()));
      }

      COMMTIMEOUTS commTimeout;
      int TimeOutInSec = 2;
      commTimeout.ReadIntervalTimeout = 1000*TimeOutInSec;
      commTimeout.ReadTotalTimeoutConstant = 1000*TimeOutInSec;
      commTimeout.ReadTotalTimeoutMultiplier = 0;
      commTimeout.WriteTotalTimeoutConstant = 1000*TimeOutInSec;
      commTimeout.WriteTotalTimeoutMultiplier = 0;


      if(!SetCommTimeouts(hSerialComm, &commTimeout))
      {
            return (0 - abs((int)::GetLastError()));
      }


      return (int)hSerialComm;
}

int ComPortManager::CloseComPortPhysical(int fd)
{
      if((HANDLE)fd != INVALID_HANDLE_VALUE)
            CloseHandle((HANDLE)fd);
      return 0;
}

int ComPortManager::WriteComPortPhysical(int port_descriptor, const wxString& string)
{
      unsigned int dwSize = string.Len();
      char *pszBuf = (char *)malloc((dwSize + 1) * sizeof(char));
      strncpy(pszBuf, string.mb_str(), dwSize+1);

#ifdef SERIAL_OVERLAPPED

      OVERLAPPED osWrite = {0};
      DWORD dwWritten;
      int fRes;

   // Create this writes OVERLAPPED structure hEvent.
      osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
      if (osWrite.hEvent == NULL)
      {
      // Error creating overlapped event handle.
            free(pszBuf);
            return 0;
      }

   // Issue write.
      if (!WriteFile((HANDLE)port_descriptor, pszBuf, dwSize, &dwWritten, &osWrite))
      {
            if (GetLastError() != ERROR_IO_PENDING)
            {
         // WriteFile failed, but it isn't delayed. Report error and abort.
                  fRes = 0;
            }
            else
            {
         // Write is pending.
                  if (!GetOverlappedResult((HANDLE)port_descriptor, &osWrite, &dwWritten, TRUE))
                        fRes = 0;
                  else
            // Write operation completed successfully.
                        fRes = dwWritten;
            }
      }
      else
      // WriteFile completed immediately.
            fRes = dwWritten;

      CloseHandle(osWrite.hEvent);

      free (pszBuf);

#else
      DWORD dwWritten;
      int fRes;

        // Issue write.
      if (!WriteFile((HANDLE)port_descriptor, pszBuf, dwSize, &dwWritten, NULL))
            fRes = 0;         // WriteFile failed, . Report error and abort.
      else
            fRes = dwWritten;      // WriteFile completed immediately.

      free (pszBuf);

#endif

      return fRes;
}

int ComPortManager::WriteComPortPhysical(int port_descriptor, unsigned char *msg, int count)
{
      return 0;
}

int ComPortManager::ReadComPortPhysical(int port_descriptor, int count, unsigned char *p)
{
      return 0;
}


bool ComPortManager::CheckComPortPhysical(int port_descriptor)
{
      return false;
}




#endif            // __WXMSW__

}; // namespace Utils
}; // namespace OpenCPN

