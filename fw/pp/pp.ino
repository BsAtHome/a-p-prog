/*
 * pp programmer, for SW 0.99 and higher
 *
 *
 */


#include <avr/io.h>
#include <util/delay.h>

#define ISP_PORT  PORTC
#define ISP_DDR   DDRC
#define ISP_PIN   PINC
#define ISP_MCLR  3
#define ISP_DAT   1
#define ISP_CLK   0

#define  ISP_MCLR_1 ISP_PORT |= (1<<ISP_MCLR);
#define  ISP_MCLR_0 ISP_PORT &= ~(1<<ISP_MCLR);
#define  ISP_MCLR_D_I ISP_DDR &= ~(1<<ISP_MCLR);
#define  ISP_MCLR_D_0 ISP_DDR |= (1<<ISP_MCLR);

#define  ISP_DAT_1 ISP_PORT |= (1<<ISP_DAT);
#define  ISP_DAT_0 ISP_PORT &= ~(1<<ISP_DAT);
#define  ISP_DAT_V (ISP_PIN&(1<<ISP_DAT))
#define  ISP_DAT_D_I ISP_DDR &= ~(1<<ISP_DAT);
#define  ISP_DAT_D_0 ISP_DDR |= (1<<ISP_DAT);

#define  ISP_CLK_1 ISP_PORT |= (1<<ISP_CLK);
#define  ISP_CLK_0 ISP_PORT &= ~(1<<ISP_CLK);
#define  ISP_CLK_D_I ISP_DDR &= ~(1<<ISP_CLK);
#define  ISP_CLK_D_0 ISP_DDR |= (1<<ISP_CLK);

#define  ISP_CLK_DELAY  1
static void isp_send(uint16_t data, uint8_t n);
static uint16_t isp_read_16(void);
static void enter_progmode(void);
static void exit_progmode(void);
static void isp_read_pgm(uint16_t * data, uint8_t n);
//static void isp_write_pgm(uint16_t * data, uint8_t n);
static void isp_mass_erase(void);
static void isp_reset_pointer(void);
static void isp_send_8_msb(uint8_t data);
static uint16_t isp_read_8_msb(void);
static uint16_t isp_read_16_msb(void);
static void p16c_enter_progmode(void);
static void p16c_set_pc(uint32_t pc);
static void p16c_bulk_erase(void);
//static void p16c_load_nvm(uint8_t inc, uint16_t data);
static uint16_t p16c_read_data_nvm(uint8_t inc);
static void p16c_begin_prog(uint8_t cfg_bit);
//static void p16c_isp_write_cfg(uint16_t data, uint16_t addr);
static void p18q_isp_write_pgm(uint16_t * data, uint32_t addr, uint8_t n);
static void p18q_isp_write_cfg(uint16_t data, uint32_t addr);

static void p18_enter_progmode(void);
//static uint16_t p18_get_ID(void);
static void p18_send_cmd_payload(uint8_t cmd, uint16_t payload);
//static uint16_t p18_get_cmd_payload(uint8_t cmd);
static uint16_t isp_read_8(void);
static void p18_set_tblptr(uint32_t val);
static uint8_t p18_read_pgm_byte(void);
//static void p_18_modfied_nop(void);
static void p18_isp_mass_erase(void);
static void p18fk_isp_mass_erase(uint8_t data1, uint8_t data2, uint8_t data3);


static void usart_tx_b(uint8_t data);
static uint8_t usart_rx_rdy(void);
static uint8_t usart_rx_b(void);
//static void usart_tx_s(uint8_t * data);

#define _BAUD	57600	// Baud rate (9600 is default)
#define _UBRR	(F_CPU/16)/_BAUD - 1	// Used for UBRRL and UBRRH

#if 0
static uint16_t fmimg[32] = {
	0x1234,0x1234,0x1234,0x1234,
	0x1234,0x1234,0x1234,0x1234,
	0x1234,0x1234,0x1234,0x1234,
	0x1234,0x1234,0x1234,0x1234,
	0x1234,0x1234,0x1234,0x1234,
	0x1234,0x1234,0x1234,0x1234,
	0x1234,0x1234,0x1234,0x1234,
	0x1234,0x1234,0x1234,0x1234,
};
#endif


//static uint16_t dat;
static uint8_t rx;
//static uint8_t main_state;
static uint8_t bytes_to_receive;
static uint8_t rx_state;
static uint8_t rx_message[280];
static uint8_t rx_message_ptr;
static uint16_t flash_buffer[260];
//static uint16_t test;
static uint16_t cfg_val;
static uint32_t addr;

int main(void)
{
	UBRR0H = ((_UBRR) & 0xF00);
	UBRR0L = (uint8_t) ((_UBRR) & 0xFF);
	UCSR0B |= _BV(TXEN0);
	UCSR0B |= _BV(RXEN0);

	ISP_CLK_D_0
	ISP_DAT_D_0
	ISP_DAT_0
	ISP_CLK_0
	ISP_MCLR_D_0
	ISP_MCLR_1
	rx_state = 0;
	while (1) {
		if (usart_rx_rdy()) {
			rx = usart_rx_b();
			rx_state = rx_state_machine (rx_state,rx);
			if (rx_state==3) {
				if (rx_message[0]==0x01) {
					enter_progmode();
					usart_tx_b (0x81);
					rx_state = 0;
				}
				if (rx_message[0]==0x02) {
					exit_progmode();
					usart_tx_b (0x82);
					rx_state = 0;
				}
				if (rx_message[0]==0x03) {
					isp_reset_pointer();
					usart_tx_b (0x83);
					rx_state = 0;
				}
				if (rx_message[0]==0x04) {
					isp_send_config(0);
					usart_tx_b (0x84);
					rx_state = 0;
				}
				if (rx_message[0]==0x05) {
					for (uint8_t i = 0; i < rx_message[2]; i++) {
						isp_inc_pointer();
					}
					usart_tx_b (0x85);
					rx_state = 0;
				}
				if (rx_message[0]==0x06) {
					usart_tx_b (0x86);
					isp_read_pgm(flash_buffer,rx_message[2]);
					for (uint8_t i = 0; i < rx_message[2]; i++) {
						usart_tx_b (flash_buffer[i]&0xFF);
						usart_tx_b (flash_buffer[i]>>8);
					}
					rx_state = 0;
				}
				if (rx_message[0]==0x07) {
					isp_mass_erase();
					usart_tx_b (0x87);
					rx_state = 0;
				}

				if (rx_message[0]==0x08) {
					for (uint8_t i = 0; i < rx_message[2]/2; i++) {
						flash_buffer[i] = (((uint16_t)(rx_message[(2*i)+1+4]))<<8) + (((uint16_t)(rx_message[(2*i)+0+4]))<<0);
					}
					isp_write_pgm(flash_buffer,rx_message[2]/2,rx_message[3]);
					usart_tx_b (0x88);
					rx_state = 0;
				}
				if (rx_message[0]==0x09) {
					isp_reset_pointer_16d();
					usart_tx_b (0x89);
					rx_state = 0;
				}
				if (rx_message[0]==0x10) {
					p18_enter_progmode();
					usart_tx_b (0x90);
					rx_state = 0;
				}
				if (rx_message[0]==0x11) {
					usart_tx_b (0x91);
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					p_18_isp_read_pgm (flash_buffer, addr, rx_message[2]);
					for (uint8_t i = 0; i < rx_message[2]; i++) {
						usart_tx_b (flash_buffer[i]&0xFF);
						usart_tx_b (flash_buffer[i]>>8);
					}
					rx_state = 0;
				}
				if (rx_message[0]==0x12) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					for (uint8_t i = 0; i < rx_message[2]/2; i++) {
						flash_buffer[i] = (((uint16_t)(rx_message[(2*i)+1+6]))<<8) + (((uint16_t)(rx_message[(2*i)+0+6]))<<0);
					}
					p18_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
					usart_tx_b (0x92);
					rx_state = 0;
				}
				if (rx_message[0]==0x13) {
					p18_isp_mass_erase();
					usart_tx_b (0x93);
					rx_state = 0;
				}
				if (rx_message[0]==0x14) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					p18_isp_write_cfg (rx_message[6],rx_message[7], addr);
					usart_tx_b (0x94);
					rx_state = 0;
				}
				if (rx_message[0]==0x23) {
					p18fj_isp_mass_erase();
					usart_tx_b (0xA3);
					rx_state = 0;
				}
				if (rx_message[0]==0x30) {
					p18fk_isp_mass_erase (rx_message[2], rx_message[3], rx_message[4]);
					usart_tx_b (0xB0);
					rx_state = 0;
				}
				if (rx_message[0]==0x31) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					for (uint8_t i = 0; i < rx_message[2]/2; i++) {
						flash_buffer[i] = (((uint16_t)(rx_message[(2*i)+1+6]))<<8) + (((uint16_t)(rx_message[(2*i)+0+6]))<<0);
					}
					p18fk_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
					usart_tx_b (0xB1);
					rx_state = 0;
				}
				if (rx_message[0]==0x32) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					p18fk_isp_write_cfg (rx_message[6],rx_message[7], addr);
					usart_tx_b (0xB2);
					rx_state = 0;
				}
				if (rx_message[0]==0x40) {
					p16c_enter_progmode();
					usart_tx_b (0xC0);
					rx_state = 0;
				}
				if (rx_message[0]==0x41) {
					usart_tx_b (0xC1);
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					p16c_isp_read_pgm (flash_buffer, addr, rx_message[2]);
					for (uint8_t i = 0; i < rx_message[2]; i++) {
						usart_tx_b (flash_buffer[i]&0xFF);
						usart_tx_b (flash_buffer[i]>>8);
					}
					rx_state = 0;
				}
				if (rx_message[0]==0x42) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					for (uint8_t i = 0; i < rx_message[2]/2; i++) {
						flash_buffer[i] = (((uint16_t)(rx_message[(2*i)+1+6]))<<8) + (((uint16_t)(rx_message[(2*i)+0+6]))<<0);
					}
					p16c_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
					usart_tx_b (0xC2);
					rx_state = 0;
				}
				if (rx_message[0]==0x43) {
					p16c_set_pc (0x8000);
					p16c_bulk_erase ();
					usart_tx_b (0xC3);
					rx_state = 0;
				}
				if (rx_message[0]==0x44) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					cfg_val = rx_message[6];
					cfg_val = (cfg_val<<8) + rx_message[7];
					p16c_isp_write_cfg (cfg_val, addr);
					usart_tx_b (0xC4);
					rx_state = 0;
				}
				if (rx_message[0]==0x45) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					cfg_val = rx_message[6];
					cfg_val = (cfg_val<<8) + rx_message[7];
					p18q_isp_write_cfg (cfg_val, addr);
					usart_tx_b (0xC5);
					rx_state = 0;
				}
				if (rx_message[0]==0x46) {
					addr = (((uint32_t)(rx_message[3]))<<16) + (((uint32_t)(rx_message[4]))<<8) + (((uint32_t)(rx_message[5]))<<0);
					for (uint8_t i = 0; i < rx_message[2]/2; i++) {
						flash_buffer[i] = (((uint16_t)(rx_message[(2*i)+1+6]))<<8) + (((uint16_t)(rx_message[(2*i)+0+6]))<<0);
					}
					p18q_isp_write_pgm (flash_buffer, addr, rx_message[2]/2);
					usart_tx_b (0xC6);
					rx_state = 0;
				}
			}
		}
	}
}

static uint8_t rx_state_machine(uint8_t state, uint8_t rx_char)
{
	if (state==0) {
		rx_message_ptr = 0;
		rx_message[rx_message_ptr++] = rx_char;
		return 1;
	}
	if (state==1) {
		bytes_to_receive = rx_char;
		rx_message[rx_message_ptr++] = rx_char;
		if (bytes_to_receive==0) return 3;
		return 2;
	}
	if (state==2) {
		rx_message[rx_message_ptr++] = rx_char;
		bytes_to_receive--;
		if (bytes_to_receive==0) return 3;
	}
	return state;
}


static void isp_read_pgm(uint16_t * data, uint8_t n)
{
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i =0; i < n; i++) {
		isp_send(0x04,6);
		data[i] = isp_read_14s();
		isp_send(0x06,6);
	}
}




static void isp_write_pgm(uint16_t * data, uint8_t n, uint8_t slow)
{
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i =0; i < n; i++) {
		isp_send(0x02,6);
		isp_send(data[i]<<1,16);
		if (i!=(n-1))
			isp_send(0x06,6);
	}
	isp_send(0x08,6);
	if (slow==1)
		_delay_ms(5);
	else
		_delay_ms(3);
	isp_send(0x06,6);
}

static void isp_send_config(uint16_t data)
{
	isp_send(0x00,6);
	isp_send(data,16);
}

static void isp_mass_erase(void)
{
	//_delay_ms(10);
	//_delay_us(3*ISP_CLK_DELAY);
	//isp_send(0x11,6);
	isp_send_config(0);
	isp_send(0x09,6);
	_delay_ms(10);
	//isp_send(0x0B,6);
	//_delay_ms(10);
}



static void isp_reset_pointer(void)
{
	//_delay_us(3*ISP_CLK_DELAY);
	isp_send(0x16,6);
}

static void isp_reset_pointer_16d(void)
{
	//_delay_us(3*ISP_CLK_DELAY);
	isp_send(0x1D,6);
	isp_send(0x0,8);
	isp_send(0x0,8);
	isp_send(0x0,8);
}

static void isp_inc_pointer(void)
{
	//_delay_us(3*ISP_CLK_DELAY);
	isp_send(0x06,6);
}


static uint16_t isp_read_16(void)
{
	uint16_t out;
	out = 0;
	ISP_DAT_D_I
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i = 0; i < 16; i++) {
		ISP_CLK_1
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_0
		_delay_us(ISP_CLK_DELAY);
		out = out >> 1;
		if (ISP_DAT_V)
			out = out | 0x8000;
	}
	return out;
}

static uint16_t isp_read_8(void)
{
	uint16_t out;
	out = 0;
	ISP_DAT_D_I
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i = 0; i < 8; i++) {
		ISP_CLK_1
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_0
		_delay_us(ISP_CLK_DELAY);
		out = out >> 1;
		if (ISP_DAT_V)
			out = out | 0x80;
	}
	return out;
}

static uint16_t isp_read_14s(void)
{
	uint16_t out;
	out = isp_read_16();
	out = out &0x7FFE;
	out = out >> 1;
	return out;
}



static void isp_send(uint16_t data, uint8_t n)
{
	ISP_DAT_D_0
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i = 0; i < n; i++) {
		if (data & 0x01) {
			ISP_DAT_1
		} else {
			ISP_DAT_0
		}
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_1
		//  _delay_us(ISP_CLK_DELAY);
		data = data >> 1;
		ISP_CLK_0
		ISP_DAT_0
		//  _delay_us(ISP_CLK_DELAY);
	}
}


static void isp_send_24_msb(uint32_t data)
{
	ISP_DAT_D_0
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i = 0; i < 23; i++) {
		if (data&0x400000) {
			ISP_DAT_1
		} else {
			ISP_DAT_0
		}
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_1
		_delay_us(ISP_CLK_DELAY);
		data = data << 1;
		ISP_CLK_0
		//  _delay_us(ISP_CLK_DELAY);
	}
	ISP_DAT_0
	_delay_us(ISP_CLK_DELAY);
	ISP_CLK_1
	_delay_us(ISP_CLK_DELAY);
	ISP_CLK_0
}

static void isp_send_8_msb(uint8_t data)
{
	ISP_DAT_D_0
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i = 0; i < 8; i++) {
		if (data&0x80) {
			ISP_DAT_1
		} else {
			ISP_DAT_0
		}
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_1
		_delay_us(ISP_CLK_DELAY);
		data = data << 1;
		ISP_CLK_0
		ISP_DAT_0
		//  _delay_us(ISP_CLK_DELAY);
	}
}


static uint16_t isp_read_8_msb(void)
{
	uint16_t out;
	out = 0;
	ISP_DAT_D_I
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i = 0; i < 8; i++) {
		ISP_CLK_1
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_0
		_delay_us(ISP_CLK_DELAY);
		out = out << 1;
		if (ISP_DAT_V)
			out = out | 0x1;
	}
	return out;
}

static uint16_t isp_read_16_msb(void)
{
	uint16_t out;
	out = 0;
	ISP_DAT_D_I
	//_delay_us(3*ISP_CLK_DELAY);
	for (uint8_t i = 0; i < 16; i++) {
		ISP_CLK_1
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_0
		_delay_us(ISP_CLK_DELAY);
		out = out << 1;
		if (ISP_DAT_V)
			out = out | 0x1;
	}
	return out;
}



static void enter_progmode(void)
{
	ISP_MCLR_0
	_delay_us(300);
	isp_send(0b01010000,8);
	isp_send(0b01001000,8);
	isp_send(0b01000011,8);
	isp_send(0b01001101,8);
	isp_send(0,1);
}

/**************************************************************************************************************************/

static void p18_enter_progmode(void)
{
	ISP_MCLR_0
	_delay_us(300);
	isp_send(0b10110010,8);
	isp_send(0b11000010,8);
	isp_send(0b00010010,8);
	isp_send(0b00001010,8);
	_delay_us(300);
	ISP_MCLR_1
}


static void p18_isp_mass_erase(void)
{
	p18_set_tblptr(0x3C0005);
	p18_send_cmd_payload(0x0C,0x0F0F);
	p18_set_tblptr(0x3C0004);
	p18_send_cmd_payload(0x0C,0x8F8F);
	p18_send_cmd_payload(0,0x0000);
	isp_send(0x00,4);
	_delay_ms(20);
	isp_send(0x00,16);
}

static void p18fj_isp_mass_erase(void)
{
	p18_set_tblptr(0x3C0005);
	p18_send_cmd_payload(0x0C,0x0101);
	p18_set_tblptr(0x3C0004);
	p18_send_cmd_payload(0x0C,0x8080);
	p18_send_cmd_payload(0,0x0000);
	isp_send(0x00,4);
	_delay_ms(600);
	isp_send(0x00,16);
}


static void p18fk_isp_mass_erase(uint8_t data1, uint8_t data2, uint8_t data3)
{
	uint16_t tmp1, tmp2, tmp3;
	tmp1 = data1;
	tmp1 = (tmp1<<8)|data1;
	tmp2 = data2;
	tmp2 = (tmp2<<8)|data2;
	tmp3 = data3;
	tmp3 = (tmp3<<8)|data3;
	p18_set_tblptr(0x3C0004);
	p18_send_cmd_payload(0x0C,tmp3);
	p18_set_tblptr(0x3C0005);
	p18_send_cmd_payload(0x0C,tmp2);
	p18_set_tblptr(0x3C0006);
	p18_send_cmd_payload(0x0C,tmp1);
	p18_send_cmd_payload(0x00,0);
	isp_send(0x00,4);
	_delay_ms(5);
	isp_send(0x00,16);
}

static void p18fk_isp_write_pgm(uint16_t * data, uint32_t addr, uint8_t n)
{
	//_delay_us(3*ISP_CLK_DELAY);
	p18_send_cmd_payload(0,0x8E7F);
	p18_send_cmd_payload(0,0x9C7F);
	p18_send_cmd_payload(0,0x847F);
	p18_set_tblptr(addr);
	for (uint8_t i = 0; i < n-1; i++)
		p18_send_cmd_payload(0x0D,data[i]);
	p18_send_cmd_payload(0x0F,data[n-1]);
	p_18_modfied_nop(0);
}

static void p18_isp_write_pgm(uint16_t * data, uint32_t addr, uint8_t n)
{
	//_delay_us(3*ISP_CLK_DELAY);
	p18_send_cmd_payload(0,0x8EA6);
	p18_send_cmd_payload(0,0x9CA6);
	p18_send_cmd_payload(0,0x84A6);
	p18_set_tblptr(addr);
	for (uint8_t i = 0; i < n-1; i++)
		p18_send_cmd_payload(0x0D,data[i]);
	p18_send_cmd_payload(0x0F,data[n-1]);
	p_18_modfied_nop(1);
}

static void p18_isp_write_cfg(uint8_t data1, uint8_t data2, uint32_t addr)
{
	uint16_t i;
	//_delay_us(3*ISP_CLK_DELAY);
	p18_send_cmd_payload(0,0x8EA6);
	p18_send_cmd_payload(0,0x8CA6);
	p18_send_cmd_payload(0,0x84A6);
	p18_set_tblptr(addr);
	p18_send_cmd_payload(0x0F,data1);
	p_18_modfied_nop(1);
	_delay_ms(5);
	p18_set_tblptr(addr+1);
	i = data2;
	i = i << 8;
	p18_send_cmd_payload(0x0F,i);
	p_18_modfied_nop(1);
	_delay_ms(5);
}

static void p18fk_isp_write_cfg(uint8_t data1, uint8_t data2, uint32_t addr)
{
	uint16_t i;
	//_delay_us(3*ISP_CLK_DELAY);
	p18_send_cmd_payload(0,0x8E7F);
	p18_send_cmd_payload(0,0x8C7F);
	p18_set_tblptr(addr);
	p18_send_cmd_payload(0x0F,data1);
	p_18_modfied_nop(1);
	_delay_ms(5);
	p18_set_tblptr(addr+1);
	i = data2;
	i = i << 8;
	p18_send_cmd_payload(0x0F,i);
	p_18_modfied_nop(1);
	_delay_ms(5);
}

static void p_18_modfied_nop(uint8_t nop_long)
{
	ISP_DAT_D_0
	ISP_DAT_0
	for (uint8_t i = 0; i < 3; i++) {
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_1
		_delay_us(ISP_CLK_DELAY);
		ISP_CLK_0
	}
	_delay_us(ISP_CLK_DELAY);
	ISP_CLK_1
	if (nop_long==1) _delay_ms(4);
	_delay_ms(1);
	ISP_CLK_0
	_delay_us(ISP_CLK_DELAY);
	isp_send(0x00,16);
}

static void p_18_isp_read_pgm(uint16_t * data, uint32_t addr, uint8_t n)
{
	uint16_t tmp1,tmp2;
	//_delay_us(3*ISP_CLK_DELAY);
	p18_set_tblptr(addr);
	for (uint8_t i = 0; i < n; i++) {
		tmp1 =  p18_read_pgm_byte();
		tmp2 =  p18_read_pgm_byte();
		tmp2 = tmp2<<8;
		data[i] = tmp1|tmp2;
	}
}


static void p18_set_tblptr(uint32_t val)
{
	p18_send_cmd_payload(0,0x0E00|((val>>16)&0xFF));
	p18_send_cmd_payload(0,0x6EF8);
	p18_send_cmd_payload(0,0x0E00|((val>>8)&0xFF));
	p18_send_cmd_payload(0,0x6EF7);
	p18_send_cmd_payload(0,0x0E00|((val>>0)&0xFF));
	p18_send_cmd_payload(0,0x6EF6);
}


static uint8_t p18_read_pgm_byte(void)
{
	isp_send(0x09,4);
	isp_send(0x00,8);
	return isp_read_8();
}

#if 0
static uint16_t p18_get_ID(void)
{
	uint16_t temp;

	p18_set_tblptr(0x3FFFFE);
	temp = p18_read_pgm_byte();
	temp = temp << 8;
	temp = temp | p18_read_pgm_byte();
	return temp;
}
#endif

static void p18_send_cmd_payload(uint8_t cmd, uint16_t payload)
{
	isp_send(cmd,4);
	isp_send(payload,16);
	_delay_us(30);
}

#if 0
static uint16_t p18_get_cmd_payload(uint8_t cmd)
{
	isp_send(cmd,4);
	return isp_read_16();
}
#endif

static void exit_progmode(void)
{
	ISP_MCLR_1
	_delay_ms(30);
	ISP_MCLR_0
	_delay_ms(30);
	ISP_MCLR_1
}

//***********************************************************************************//

static void p16c_enter_progmode(void)
{
	ISP_MCLR_0
	_delay_us(300);
	isp_send_8_msb(0x4d);
	isp_send_8_msb(0x43);
	isp_send_8_msb(0x48);
	isp_send_8_msb(0x50);
	_delay_us(300);
}

static void p16c_set_pc(uint32_t pc)
{
	isp_send_8_msb(0x80);
	_delay_us(2);
	isp_send_24_msb(pc);
}

static void p16c_bulk_erase(void)
{
	isp_send_8_msb(0x18);
	_delay_ms(100);
}

static void p16c_load_nvm(uint16_t data, uint8_t inc)
{
	if (inc==0) isp_send_8_msb(0x00);
	else isp_send_8_msb(0x02);
	_delay_us(2);
	isp_send_24_msb(data);
	_delay_us(2);
}

static uint16_t p16c_read_data_nvm(uint8_t inc)
{
	uint16_t retval;
	uint8_t tmp;
	if (inc==0)
		isp_send_8_msb(0xFC);
	else
		isp_send_8_msb(0xFE);
	_delay_us(2);
	tmp = isp_read_8_msb();
	retval = isp_read_16_msb();
	retval = retval >> 1;
	if (tmp&0x01) retval = retval | 0x8000;
	return retval;
}

static void p16c_begin_prog(uint8_t cfg_bit)
{
	isp_send_8_msb(0xE0);
	_delay_ms(3);
	if (cfg_bit!=0) _delay_ms(3);
}

#if 0
static uint16_t p16c_get_ID(void)
{
	p16c_set_pc(0x8006);
	return p16c_read_data_nvm(1);
}
#endif

static void p16c_isp_write_pgm(uint16_t * data, uint32_t addr, uint8_t n)
{
	uint8_t i;
	//_delay_us(3*ISP_CLK_DELAY);
	p16c_set_pc(addr);
	for (i=0;i<n;i++)
		p16c_load_nvm(data[i],1);
	p16c_set_pc(addr);
	p16c_begin_prog(0);
}

static void p16c_isp_read_pgm(uint16_t * data, uint32_t addr, uint8_t n)
{
	uint8_t i;
	//_delay_us(3*ISP_CLK_DELAY);
	p16c_set_pc(addr);
	for (i=0;i<n;i++)
		data[i] = p16c_read_data_nvm(1);
}

static void p16c_isp_write_cfg(uint16_t data, uint32_t addr)
{
	//_delay_us(3*ISP_CLK_DELAY);
	p16c_set_pc(addr);
	p16c_load_nvm(data,0);
	p16c_begin_prog(1);
}

static void p18q_isp_write_pgm(uint16_t * data, uint32_t addr, uint8_t n)
{
	uint8_t i;
	//_delay_us(3*ISP_CLK_DELAY);
	p16c_set_pc(addr);
	for (i=0;i<n;i++) {
		isp_send_8_msb(0xE0);
		isp_send_24_msb(data[i]);
		_delay_us(65);
	}
}

static void p18q_isp_write_cfg(uint16_t data, uint32_t addr)
{
	//_delay_us(3*ISP_CLK_DELAY);
	p16c_set_pc(addr);
	isp_send_8_msb(0xE0);
	isp_send_24_msb(data);
	_delay_us(65);
}


static void usart_tx_b(uint8_t data)
{
	while (!(UCSR0A & _BV(UDRE0)));
		UDR0 = data;
}

#if 0
static void usart_tx_s(uint8_t * data)
{
	while (*data!=0)
		usart_tx_b(*data++);
}
#endif

static uint8_t usart_rx_rdy(void)
{
	if (UCSR0A & _BV(RXC0))
		return 1;
	else
		return 0;
}

static uint8_t usart_rx_b(void)
{
	return (uint8_t) UDR0;
}

#if 0
static void usart_tx_hexa_8(uint8_t value)
{
	usart_tx_b('0');
	usart_tx_b('x');
	usart_tx_hexa_8b(value);
	usart_tx_b(' ');
}

static void usart_tx_hexa_8b(uint8_t value)
{
	uint8_t temp;
	temp = value;
	temp = ((temp>>4)&0x0F);
	if (temp<10) temp = temp + '0';
	else temp = temp + 'A'- 10;
	usart_tx_b(temp);
	temp = value;
	temp = ((temp>>0)&0x0F);
	if (temp<10) temp = temp + '0';
	else temp = temp + 'A' - 10;
	usart_tx_b(temp);
}

static void usart_tx_hexa_16(uint16_t value)
{
	usart_tx_b('0');
	usart_tx_b('x');
	usart_tx_hexa_8b((value>>8)&0xFF);
	usart_tx_hexa_8b(value&0xFF);
	usart_tx_b(' ');
}
#endif
