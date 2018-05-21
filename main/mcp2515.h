
#pragma once

#include "driver/spi_master.h"

//Prototypes
void mcp_init(uint32_t baud);
void mcp_send_can(uint32_t arb_id, const uint8_t * data, const uint8_t len);

//Module Prototypes
void mcp_write_byte(uint8_t val);
void mcp_write_reg(uint8_t addr, uint8_t val);
uint8_t mcp_read_reg(uint8_t addr);
void mcp_write_bit(uint8_t addr, uint8_t mask, uint8_t val);
uint8_t mcp_read_status();


//mcp2515 spi commands
enum spi_cmd_type
{
	WRITE_CMD = 2,
	READ_CMD = 3,
	BIT_MODIFY_CMD = 5,
	LOAD_TX_BUFFER_CMD = 0x40,
	RTS_CMD = 0x80,
	READ_RX_BUFFER_CMD = 0x90,
	READ_STATUS_CMD = 0xA0,
	RX_STATUS_CMD = 0xB0,
	RESET_CMD = 0xC0
};

/* MPC2515 registers */
#define CANSTAT	      0x0E
#define CANCTRL	      0x0F
#  define CANCTRL_REQOP_MASK	    0xE0
#  define CANCTRL_REQOP_CONF	    0x80
#  define CANCTRL_REQOP_LISTEN_ONLY 0x60
#  define CANCTRL_REQOP_LOOPBACK    0x40
#  define CANCTRL_REQOP_SLEEP	    0x20
#  define CANCTRL_REQOP_NORMAL	    0x00
#  define CANCTRL_OSM		    0x08
#  define CANCTRL_ABAT		    0x10
#define TEC	      0x1C
#define REC	      0x1D
#define CNF1	      0x2A
#  define CNF1_SJW_SHIFT   6
#define CNF2	      0x29
#  define CNF2_BTLMODE	   0x80
#  define CNF2_SAM         0x40
#  define CNF2_PS1_SHIFT   3
#define CNF3	      0x28
#  define CNF3_SOF	   0x08
#  define CNF3_WAKFIL	   0x04
#  define CNF3_PHSEG2_MASK 0x07
#define CANINTE	      0x2B
#  define CANINTE_MERRE 0x80
#  define CANINTE_WAKIE 0x40
#  define CANINTE_ERRIE 0x20
#  define CANINTE_TX2IE 0x10
#  define CANINTE_TX1IE 0x08
#  define CANINTE_TX0IE 0x04
#  define CANINTE_RX1IE 0x02
#  define CANINTE_RX0IE 0x01
#define CANINTF	      0x2C
#  define CANINTF_MERRF 0x80
#  define CANINTF_WAKIF 0x40
#  define CANINTF_ERRIF 0x20
#  define CANINTF_TX2IF 0x10
#  define CANINTF_TX1IF 0x08
#  define CANINTF_TX0IF 0x04
#  define CANINTF_RX1IF 0x02
#  define CANINTF_RX0IF 0x01
#  define CANINTF_RX (CANINTF_RX0IF | CANINTF_RX1IF)
#  define CANINTF_TX (CANINTF_TX2IF | CANINTF_TX1IF | CANINTF_TX0IF)
#  define CANINTF_ERR (CANINTF_ERRIF)
#define EFLG	      0x2D
#  define EFLG_EWARN	0x01
#  define EFLG_RXWAR	0x02
#  define EFLG_TXWAR	0x04
#  define EFLG_RXEP	0x08
#  define EFLG_TXEP	0x10
#  define EFLG_TXBO	0x20
#  define EFLG_RX0OVR	0x40
#  define EFLG_RX1OVR	0x80
#define TXBCTRL(N)  (((N) * 0x10) + 0x30 + TXBCTRL_OFF)
#  define TXBCTRL_ABTF	0x40
#  define TXBCTRL_MLOA	0x20
#  define TXBCTRL_TXERR 0x10
#  define TXBCTRL_TXREQ 0x08
#define TXBSIDH(N)  (((N) * 0x10) + 0x30 + TXBSIDH_OFF)
#  define SIDH_SHIFT    3
#define TXBSIDL(N)  (((N) * 0x10) + 0x30 + TXBSIDL_OFF)
#  define SIDL_SID_MASK    7
#  define SIDL_SID_SHIFT   5
#  define SIDL_EXIDE_SHIFT 3
#  define SIDL_EID_SHIFT   16
#  define SIDL_EID_MASK    3
#define TXBEID8(N)  (((N) * 0x10) + 0x30 + TXBEID8_OFF)
#define TXBEID0(N)  (((N) * 0x10) + 0x30 + TXBEID0_OFF)
#define TXBDLC(N)   (((N) * 0x10) + 0x30 + TXBDLC_OFF)
#  define DLC_RTR_SHIFT    6
#define TXBCTRL_OFF 0
#define TXBSIDH_OFF 1
#define TXBSIDL_OFF 2
#define TXBEID8_OFF 3
#define TXBEID0_OFF 4
#define TXBDLC_OFF  5
#define TXBDAT_OFF  6
#define TXBD(N) (((N) * 0x10) + 0x36)
#define RXBCTRL(N)  (((N) * 0x10) + 0x60 + RXBCTRL_OFF)
#  define RXBCTRL_BUKT	0x04
#  define RXBCTRL_RXM0	0x20
#  define RXBCTRL_RXM1	0x40
#define RXBSIDH(N)  (((N) * 0x10) + 0x60 + RXBSIDH_OFF)
#  define RXBSIDH_SHIFT 3
#define RXBSIDL(N)  (((N) * 0x10) + 0x60 + RXBSIDL_OFF)
#  define RXBSIDL_IDE   0x08
#  define RXBSIDL_SRR   0x10
#  define RXBSIDL_EID   3
#  define RXBSIDL_SHIFT 5
#define RXBEID8(N)  (((N) * 0x10) + 0x60 + RXBEID8_OFF)
#define RXBEID0(N)  (((N) * 0x10) + 0x60 + RXBEID0_OFF)
#define RXBDLC(N)   (((N) * 0x10) + 0x60 + RXBDLC_OFF)
#  define RXBDLC_LEN_MASK  0x0F
#  define RXBDLC_RTR       0x40
#define RXBD(N)   (((N) * 0x10) + 0x60 + RXBDAT_OFF)
#define RXBCTRL_OFF 0
#define RXBSIDH_OFF 1
#define RXBSIDL_OFF 2
#define RXBEID8_OFF 3
#define RXBEID0_OFF 4
#define RXBDLC_OFF  5
#define RXBDAT_OFF  6
#define RXFSID(N) ((N < 3) ? 0 : 4)
#define RXFSIDH(N) ((N) * 4 + RXFSID(N))
#define RXFSIDL(N) ((N) * 4 + 1 + RXFSID(N))
#define RXFEID8(N) ((N) * 4 + 2 + RXFSID(N))
#define RXFEID0(N) ((N) * 4 + 3 + RXFSID(N))
#define RXMSIDH(N) ((N) * 4 + 0x20)
#define RXMSIDL(N) ((N) * 4 + 0x21)
#define RXMEID8(N) ((N) * 4 + 0x22)
#define RXMEID0(N) ((N) * 4 + 0x23)
#define BFPCTRL 0x0C

//BRGCON
// format --CNFG1 CNFG2 CNFG3
#define BAUD_125 0x01B501
#define BAUD_250 0x00B501
#define BAUD_500 0x009101
#define BAUD_800 0x008001

