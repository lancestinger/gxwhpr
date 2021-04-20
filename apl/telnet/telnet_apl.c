/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network:Service
 * Copyright (c) 2004-2015 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    Telnet_Server_UIF.c
 * Purpose: Telnet Server User Interface
 * Rev.:    V7.0.0
 *----------------------------------------------------------------------------*/
//! [code_Telnet_Server_UIF]
#include <stdio.h>
#include "rl_net.h"
#include "rl_net_lib.h" 
#include "pubdef.h"
#include "rngbuf/rngbuf.h"
#include "sshell/sshell_fml.h"
#include "uart/uart_fml.h"
#include "rl_net_legacy.h"
// ANSI ESC Sequences for terminal control
#define CLS     "\033[2J"
#define TBLUE   "\033[37;44m"
#define TNORM   "\033[0m"

// My structure of status variable pvar
typedef struct {
  uint8_t id;
  uint8_t nmax;
  uint8_t idx;
} MY_BUF;
#define MYBUF(p)        ((MY_BUF *)p)
extern RNGBUF_t * printRngBuf;
// Local variables


static const char meas_header[] = {
  "\r\n"
  " Nr.   ADIN0  ADIN1  ADIN2  ADIN3  ADIN4  ADIN5  ADIN6  ADIN7\r\n"
  "=============================================================="
};

static const char tcp_stat_header[] = {
  CLS "\r\n"
  " " TBLUE
  "==============================================================================\r\n" TNORM
  " " TBLUE
  " Sock  State        Port  Timer  Remote Address                Port           \r\n" TNORM
  " " TBLUE
  "==============================================================================\r\n" TNORM
};


// Request message for Telnet server session.
uint32_t netTELNETs_ProcessMessage (netTELNETs_Message msg, char *buf, uint32_t buf_len) {
  uint32_t len = 0;

  (void)buf_len;

  switch (msg) {
    case netTELNETs_MessageWelcome:
      // Initial welcome message
      len = (uint32_t)sprintf (buf, "\r\n"
                                    "GXW Telnet Server ready!!!\r\n");
      break;
    case netTELNETs_MessagePrompt:
      // Prompt message
      len = sprintf (buf, "\r\n"
                           "[sshell@rtx]# ");
      break;
    case netTELNETs_MessageLogin:
     // Login message, if authentication is enabled
      len = (uint32_t)sprintf (buf, "\r\n"
                                    "Please login...");
      break;
    case netTELNETs_MessageUsername:
      // Username request login message
      len = (uint32_t)sprintf (buf, "\r\n"
                                    "Username: ");
      break;
    case netTELNETs_MessagePassword:
      // Password request login message
      len = (uint32_t)sprintf (buf, "\r\n"
                                    "Password: ");
      break;
    case netTELNETs_MessageLoginFailed:
      // Incorrect login error message
      len = (uint32_t)sprintf (buf, "\r\n"
                                    "Login incorrect");
      break;
    case netTELNETs_MessageLoginTimeout:
      // Login timeout error message
      len = (uint32_t)sprintf (buf, "\r\n"
                                    "Login timeout\r\n");
      break;
    case netTELNETs_MessageUnsolicited:
      // Unsolicited message (ie. from basic interpreter)
      break;
  }
  return (len);
}

// Process a command and generate response.
uint32_t netTELNETs_ProcessCommand (const char *cmd, char *buf, uint32_t buf_len, uint32_t *pvar) {
    int32_t socket;
    netTCP_State state;
    NET_ADDR client;
    char ip_ascii[40];
    const char *passwd;
	const char *usernm;
    uint32_t nmax;
    uint32_t len = 0;
    extern RNGBUF_t * print_rngbuf;

    switch (MYBUF(pvar)->id) {
        case 0:
            // First call to this function
        break;

        /*case 1:
        // Command MEAS, repeated call
        // Let's use as much of the buffer as possible
        while (len < buf_len-80) {
        len += (uint32_t)sprintf (buf+len, "\r\n%4d", MYBUF(pvar)->idx);
        for (val = 0; val < 8; val++) {
        len += (uint32_t)sprintf (buf+len, "%7d", AD_in(val));
        }
        if (++MYBUF(pvar)->idx >= MYBUF(pvar)->nmax) {
        // Requested number of measurements done
        return (len);
        }
        }
        // Set request for another callback
        return (len | (1u << 31));*/

        case 2:
            // Command TCPSTAT, repeated call
            if (MYBUF(pvar)->idx == 0) {
                len = (uint32_t)sprintf (buf, tcp_stat_header);
            }
            // Let's use as much of the buffer as possible
            while (len < buf_len-96) {
                socket = ++MYBUF(pvar)->idx;
                state  = netTCP_GetState (socket);

                if (state == netTCP_StateINVALID) {
                    // Invalid socket, we are done
                    len += (uint32_t)sprintf (buf+len, "\r\n");
                    // Reset index counter for next callback
                    MYBUF(pvar)->idx = 0;
                    // Repeat a command after 20 ticks (2 seconds)
                    netTELNETs_RepeatCommand (20);
                    break;
                }

                len += (uint32_t)sprintf (buf+len, "\r\n  %-6d%-13s", MYBUF(pvar)->idx, net_tcp_ntoa(state));

                if (state == netTCP_StateLISTEN) {
                    len += (uint32_t)sprintf (buf+len, "%-5d", netTCP_GetLocalPort (socket));
                }
                else if (state > netTCP_StateLISTEN) {
                    // Retrieve remote client IP address and port number
                    netTCP_GetPeer (socket, &client, sizeof(client));
                    // Convert client IP address to ASCII string
                    netIP_ntoa (client.addr_type, client.addr, ip_ascii, sizeof(ip_ascii));

                    len +=(uint32_t) sprintf (buf+len, "%-6d%-7d%-30s%-5d",
                    netTCP_GetLocalPort (socket), netTCP_GetTimer (socket),
                    ip_ascii, client.port);
                }
            }
            // Set request for another callback
            return (len | (1u << 31));

        case 3:
            telnet_server_set_delay (2);
#if 0
            len = 0;
            buf[0] = 0x0;
            if(!rng_is_empty(print_rngbuf))
            {
                len = rng_get_buf(print_rngbuf,buf,1024);
            }
            buf[len++] = 0x0;
#endif

            if(uart_get_dbg_outmode() == DBG_OUT_TELNET)
            {
							len = 0;
							buf[0] = 0x0;
							if(!rng_is_empty(print_rngbuf))
							{
									len = rng_get_buf(print_rngbuf,buf,1024);
							}
							buf[len++] = 0x0;

              len = (len | (1u << 31));
            }
						else
						{
							len = 0;
							len = (len | (1u << 31));
						}

						return len;
						
						#if 0
            if(uart_get_dbg_outmode() == DBG_OUT_USART)
            {
                return (len);
            }
            else
            {
                return (len | (1u << 31));
            }
						#endif
    }

    // Simple command line parser
	len = strlen (cmd);
    if (netTELNETs_CheckCommand (cmd, "BYE") == true) {
        // BYE command given, disconnect the client
        len = (uint32_t)sprintf (buf, "\r\nDisconnecting\r\n");
        // Bit-30 of return value is a disconnect flag
        return (len | (1u << 30));
    }
	if (netTELNETs_CheckCommand (cmd, "USERN") == true && netTELNETs_LoginActive()) {
		
        // PASSW command given, change password
        if (len > 6) {
            // Change password
            usernm = &cmd[6];
        }
        else { 
            usernm = NULL; 
        }

        if (netTELNETs_SetUsername (usernm) == netOK) {
            len = (uint32_t)sprintf (buf, "\r\n OK, New Username: \"%s\"", netTELNETs_GetUsername ());
        }
        else {
            len = (uint32_t)sprintf (buf, "\r\nFailed to change username!");
        }
        return (len);
    }
	if (netTELNETs_CheckCommand (cmd, "PASSW") == true && netTELNETs_LoginActive()) {
		
        // PASSW command given, change password
        if (len > 6) {
            // Change password
            passwd = &cmd[6];
        }
        else { 
            passwd = NULL; 
        }

        if (netTELNETs_SetPassword (passwd) == netOK) {
            len = (uint32_t)sprintf (buf, "\r\n OK, New Password: \"%s\"", netTELNETs_GetPassword ());
        }
        else {
            len = (uint32_t)sprintf (buf, "\r\nFailed to change password!");
        }
        return (len);
    }

	if (netTELNETs_CheckCommand (cmd, "USER") == true && netTELNETs_LoginActive()) {
        // PASSWD command given, print current username
        len = (uint32_t)sprintf (buf, "\r\n System Username: \"%s\"", netTELNETs_GetUsername());
        return (len);
    }
	
    if (netTELNETs_CheckCommand (cmd, "PASSWD") == true && netTELNETs_LoginActive()) {
        // PASSWD command given, print current password
        len = (uint32_t)sprintf (buf, "\r\n System Password: \"%s\"", netTELNETs_GetPassword());
        return (len);
    }

    if (netTELNETs_CheckCommand (cmd, "MEAS") == true) {
        // MEAS command given, monitor analog inputs
        MYBUF(pvar)->id = 1;
        if (len > 5) {
            GLOBAL_SSCANF (&cmd[5], "%u", &nmax);
            if (nmax > 255) { nmax = 255; }
            MYBUF(pvar)->nmax = (unsigned char)nmax; 
        }
        len = (uint32_t)sprintf (buf, meas_header);
        if (MYBUF(pvar)->nmax) {
            // Bit-31 is a repeat flag
            len |= (1u << 31);
        }
        return (len);
    }

    if (netTELNETs_CheckCommand (cmd, "TCPSTAT") == true) {
        // TCPSTAT command given, display TCP socket status
        MYBUF(pvar)->id = 2;
        len = (uint32_t)sprintf (buf, CLS);
        return (len | (1u << 31));
    }

    if (netTELNETs_CheckCommand (cmd, "RINFO") == true) {
        // Print IP address and port number of connected telnet client
        if (netTELNETs_GetClient (&client, sizeof(client)) == netOK) {
            // Convert client IP address to ASCII string
            netIP_ntoa (client.addr_type, client.addr, ip_ascii, sizeof(ip_ascii));

            len  = (uint32_t)sprintf (buf    ,"\r\n IP addr : %s", ip_ascii);
            len += (uint32_t)sprintf (buf+len,"\r\n TCP port: %d", client.port);
            return (len);
        }
        len = (uint32_t)sprintf (buf, "\r\n Error!");
        return (len);
    }


    uart_set_dbg_outmode(DBG_OUT_TELNET);

    if(sshell_excute_cmd((U8*)cmd) == TRUE)
    {
        MYBUF(pvar)->id = 3;
        len += sprintf (buf+len, "\r\n");
        len |= (1u << 31);
        return (len);
    }

    uart_set_dbg_outmode(DBG_OUT_USART);
    
    // Unknown command given
    len = (uint32_t)sprintf (buf, "\r\n "ANSI_BG_RED"ERROR "ANSI_NONE"Unknow cmd: %s\r\n", cmd);
    return (len);
}

