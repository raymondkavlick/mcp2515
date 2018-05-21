
#include "main.h"
#include "mcp2515.h"

#define PIN_NUM_MISO 23//IO23 on JP1 on wroverkit
#define PIN_NUM_MOSI 18//IO18 on JP1 on wroverkit
#define PIN_NUM_CLK  4//IO4 on JP1 on wroverkit
#define PIN_NUM_CS   26//IO26 on JP1 on wroverkit
#define PIN_NUM_INT  21//IO21 on JP1 on wroverkit



void handle_rx();
spi_device_handle_t spi;

void mcp_init(uint32_t baud)
{
	esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=100
    };
    spi_device_interface_config_t devcfg={

        .clock_speed_hz=8*1000*1000,
        .mode=0,          //SPI mode 0
        .spics_io_num=PIN_NUM_CS,
        .queue_size=40,
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the MPC2515 to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize CAN 
	 mcp_write_byte(RESET_CMD);
	 mcp_write_reg(CANCTRL,CANCTRL_REQOP_CONF);//set to Config mode

	 mcp_write_reg(CNF1,(baud & 0xFF0000) >> 0x10);
	 mcp_write_reg(CNF2,(baud & 0xFF00) >> 0x8);
	 mcp_write_reg(CNF3,baud & 0xFF);

#define all_msgs_ignore_masks (RXBCTRL_RXM0 | RXBCTRL_RXM1)
	 //setup RX filter
	 mcp_write_reg(RXBCTRL(0),all_msgs_ignore_masks);

	 mcp_write_reg(CANCTRL,CANCTRL_REQOP_NORMAL);//set to Normal mode
	 mcp_write_bit(CANINTE,CANINTE_TX0IE | CANINTE_TX1IE | CANINTE_TX2IE | CANINTE_RX0IE,
			 CANINTE_TX0IE | CANINTE_TX1IE | CANINTE_TX2IE | CANINTE_RX0IE);
    //setup

}

void mcp_send_can(uint32_t arb_id, const uint8_t * data, const uint8_t len)
{
#define INVALID_BUFFER 3
	int buffer_to_use = INVALID_BUFFER;
	//check if buffers are free (0,1,or 2),choose a free buffer, 10 retries before fail
	int timeout = 10;
	while(buffer_to_use > 2 && --timeout)
	{
		if(!(mcp_read_reg(TXBCTRL(0)) & TXBCTRL_TXREQ))
			buffer_to_use = 0;
		else if(!(mcp_read_reg(TXBCTRL(1)) & TXBCTRL_TXREQ))
			buffer_to_use = 1;
		else if(!(mcp_read_reg(TXBCTRL(2)) & TXBCTRL_TXREQ))
			buffer_to_use = 2;
	}

	if(!timeout && buffer_to_use == INVALID_BUFFER)
	{
		printf("tx overflow");
		return;
	}

	mcp_write_reg(TXBDLC(buffer_to_use),len);
	for(int i = 0; i < len; i++)
		mcp_write_reg(TXBD(buffer_to_use) + i, data[i]);
	mcp_write_reg(TXBSIDL(buffer_to_use),arb_id << SIDL_SID_SHIFT);
	mcp_write_reg(TXBSIDH(buffer_to_use),arb_id >> SIDH_SHIFT);
	mcp_write_byte(RTS_CMD | (1 << buffer_to_use));

}

void mcp_write_reg(uint8_t addr, uint8_t val)
{
    spi_transaction_t spi_packet;
    memset(&spi_packet, 0, sizeof(spi_packet));
    spi_packet.flags |= SPI_TRANS_USE_TXDATA;
    spi_packet.tx_data[0] = WRITE_CMD;
    spi_packet.tx_data[1] = addr;
	spi_packet.tx_data[2] = val;
    spi_packet.length = 8 * 3;//bits to send (3 bytes)
    if(spi_device_transmit(spi, &spi_packet) != ESP_OK)
    	printf("\r\nError--SpiTx");
}

uint8_t mcp_read_reg(uint8_t addr)
{
    spi_transaction_t spi_packet;
    memset(&spi_packet, 0, sizeof(spi_packet));
    spi_packet.flags |= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    spi_packet.tx_data[0] = READ_CMD;
    spi_packet.tx_data[1] = addr;
	spi_packet.tx_data[2] = 0xFF;//dummy
    spi_packet.length = 8 * 3;//bits to send (3 bytes)
    if(spi_device_transmit(spi, &spi_packet) != ESP_OK)
    	printf("\r\nError--SpiRx");

    return spi_packet.rx_data[2];
}


uint8_t mcp_read_status()
{
    spi_transaction_t spi_packet;
    memset(&spi_packet, 0, sizeof(spi_packet));
    spi_packet.flags |= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    spi_packet.tx_data[0] = READ_STATUS_CMD;
	spi_packet.tx_data[2] = 0xFF;//dummy
    spi_packet.length = 8 * 2;//bits to send (2 bytes)
    if(spi_device_transmit(spi, &spi_packet) != ESP_OK)
    	printf("\r\nError--SpiRx");

    return spi_packet.rx_data[1];
}

void mcp_write_byte(uint8_t val)
{
    spi_transaction_t spi_packet;
    memset(&spi_packet, 0, sizeof(spi_packet));
    spi_packet.flags |= SPI_TRANS_USE_TXDATA;
    spi_packet.tx_data[0] = val;
    spi_packet.length = 8 * 1;//bits to send (1 bytes)
    if(spi_device_transmit(spi, &spi_packet) != ESP_OK)
    	printf("\r\nError--SpiTx");
}

void mcp_write_bit(uint8_t addr, uint8_t mask, uint8_t val)
{
    spi_transaction_t spi_packet;
    memset(&spi_packet, 0, sizeof(spi_packet));
    spi_packet.flags |= SPI_TRANS_USE_TXDATA;
    spi_packet.tx_data[0] = BIT_MODIFY_CMD;
    spi_packet.tx_data[1] = addr;
	spi_packet.tx_data[2] = mask;
	spi_packet.tx_data[3] = val;
    spi_packet.length = 8 * 4;//bits to send (4 bytes)
    if(spi_device_transmit(spi, &spi_packet) != ESP_OK)
    	printf("\r\nError--SpiTx");
}

void handle_rx()
{
//use rx buffer command for faster reads
	WORD_ALIGNED_ATTR uint8_t tx_buff[0x10];
	WORD_ALIGNED_ATTR uint8_t rx_buff[0x10];
	spi_transaction_t spi_packet;
	memset(&spi_packet, 0, sizeof(spi_packet));
	spi_packet.tx_buffer = tx_buff;
	spi_packet.rx_buffer = rx_buff;
	tx_buff[0] = READ_RX_BUFFER_CMD | 0x0/*n m*/ | 0x0;
	spi_packet.length = 8 * 0x10;//bits to send (1 bytes)
	spi_packet.rxlength = 8 * 0x10;//bits to send (0xD bytes)
	if(spi_device_transmit(spi, &spi_packet) != ESP_OK)
		printf("\r\nError--SpiRx");

	uint32_t filt = (rx_buff[2] >> RXBSIDL_SHIFT)
			| ((uint16_t)rx_buff[1] << RXBSIDH_SHIFT);
	printf("\r\nFilter input = 0x%03X\r\n",filt);

	printf("TestData = %02X %02X %02X\r\n",
			rx_buff[1],
			(rx_buff[2]),

			(rx_buff[0]));

	printf("Data = %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
			rx_buff[6],
			(rx_buff[7]),
			(rx_buff[8]),
			(rx_buff[9]),
			(rx_buff[10]),
			(rx_buff[11]),
			(rx_buff[12]),
			(rx_buff[13])
			);

	//clear interrupt to open buffer
	mcp_write_bit(CANINTF,CANINTF_RX0IF,0);
}

bool rx_has_data()
{
	if((mcp_read_status() & 1))
		handle_rx();

	return 0;
//	return (mcp_read_status() & 1);
}

