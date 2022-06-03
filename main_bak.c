//#define SIM908type
#define L50type
//#define ZTR1type
//#define SPImem
#define I2Cmem
//#define OZUmem
//#define ZTR_BTN
//#define CAN
//#define L80type
#define RFM73

#define Version		30

void    START(void);

#if defined(RFM73)
void	RFM73T(void);
#endif
#include "p24Fxxxx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <i2c.h>
#include <spi.h>
#include <osa.h>
#include <rfm73.h>

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_OFF & ICS_PGx1 & FWDTEN_OFF & WINDIS_ON & FWPSA_PR128	 & WDTPS_PS16384);
_CONFIG2( IESO_OFF & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_HS)

#define GSM_ON		_LATC3
#define GSM_ON_TRIS	_TRISC3
#define OFF900		_LATB9
#define RTS			_LATC4
#define RTS_TRIS	_TRISC4
#define GSM_TX_TRIS	_TRISC7
#define GPS_TX_TRIS	_TRISC2
#define GSM_STS		_RB10

#ifndef ZTR1type
#define LED_ER 		_LATC8
#define LED_FX 		_LATC9
#define ALLOFF		_LATB8
#define INZ			_RA10
#define OUT1		_LATA4
#define SPI_CS1		_LATB4
#endif

#if defined(ZTR1type)
#define LED_ER 		_LATC9
#define LED_FX 		_LATC9
#endif

#if defined(SIM908type)
#define DTR			_LATC5
#define DTR_TRIS	_TRISC5
#define OUT2		_LATA7
#define SPI_CS2		_LATC0
#define MMA_INT1	_RB11
#define SUART		_RB13
#define IN1			_RA8
#define IN2			_RA9
#endif

#if defined(L50type) || defined(ZTR_BTN)
#define IN1			_RC0
#define IN2			_RC5
#endif

#if defined(L50type) || defined(ZTR1type)
#define SUART		_RB14
#define GPS_R		_LATB11
#define GPS_R_TRIS	_TRISB11
#define	GPS_ON		_LATB12//питание
#define GPS_OFFB	_LATB13//кнопка
#define GPS_OFFB_TRIS	_TRISB13
#define OUT2			_LATA8
#endif

#if defined(RFM73)
#define SPI_CS _RB4
#define SPI_CE _RB14
#define SPI_CLK _RB6
#define SPI_OUT _RB5
#define SPI_IN _RB7
#endif

#define EEPROM_ADR	0b10100000

char ExIP1[16] = "194.28.172.136", ExIP2[16] = "194.28.172.136", ExIP3[16] = "194.28.172.136";
unsigned int ExPort1 = 10028, ExPort2 = 10028, ExPort3 = 10028;
char APN[31] = "www.kyivstar.net", APN_user[31] = "", APN_pass[31] = "";
unsigned char PortPr = 0b00000000;

unsigned int Rsindp = 1;
unsigned int Sync = 0;
volatile unsigned int Rsindp_cnt = 0;
#if defined(SIM908type)  || defined(L50type)
unsigned int Period1 = 10, Period2 = 600, Period3 = 2, prm_per = 300, gsmloc_per = 0, money_per = 0, trip_per = 0, debug1_per = 0, debug2_per = 0;
unsigned int can1_per = 0, can2_per = 0, ADC_per = 0, INx_per = 0, freq_per = 0, ident_per = 0, offsim_per = 14400;
unsigned int Send_per1 = 300, Send_per2 = 1800, Stealth_ontm = 0;
volatile unsigned int Stealth_per = 0;
unsigned int Vbt_off = 3500;
char SMS_pass[5] = "0000";
char MoneyUSSD[11] = "*111#";
unsigned int Settings = 0b0000000011001001;
//bit0 - (0)пер.отпр.-0-движ;1-заж.;2-пост.;3-адапт.		1
//bit1 - (1)пер.отпр.-0-движ;1-заж.;2-пост.;3-адапт.		2
//bit2 - опред. роуминг по карте							4
//bit3 - опред. роуминг по GSM				 				8
//bit4 - reserved											16
//bit5 - отправлять данные в роуминге						32
//bit6 - отвечать на СМС  в роуминге			 			64
//bit7 - (0)фильтр на стоянкак-0-нет;1-по прогр.движ.		128
//bit8 - (1)2-по заж.3-reserved								256
//bit9 - reserved											512
//bit10 - reserve											1024
//bit11 - Не разрывать соединение							2048
//bit12 - reserve											4096
//bit13 - reserve											8192
//bit14 - reserve											16384
//bit15 - reserve											32768

unsigned int Settings1 = 0b0000000000001111;
//bit0 - при изм.заж.зап.коорд.								1
//bit1 - при изм.заж.начинать отпр.							2
//bit2 - при откл/вкл.вн.напр.зап.коорд.					4
//bit3 - при откл/вкл.вн.напр.начинать отпр.				8
//bit4 - IN1 - тревога										16
//bit5 - IN2 - тревога										32
//bit6 - Тревога1 акт.выс.						 			64
//bit7 - Тревога2 акт.выс.									128
//bit8 - отправл.gsmloc при опред. по gps					256
//bit9 - при изм.движ.зап.									512
//bit10 - при изм.движ.отпр.								1024
//bit11 - при изм. IN1 отпр.								2048
//bit12 - при изм. IN2 отпр.								4096
//bit13 - reserve											8192
//bit14 - reserve											16384
//bit15 - reserve											32768
#endif

#if defined(ZTR1type)
unsigned int Period1 = 10, Period2 = 600, Period3 = 2, prm_per = 300, gsmloc_per = 0, money_per = 0, trip_per = 0, debug1_per = 0, debug2_per = 0;
unsigned int can1_per = 0, can2_per = 0, ADC_per = 0, INx_per = 0, freq_per = 0, ident_per = 0, offsim_per = 0;
unsigned int Send_per1 = 300, Send_per2 = 1800, Stealth_ontm = 30;
volatile unsigned int Stealth_per = 5;
unsigned int Vbt_off = 0;
char SMS_pass[5] = "0000";
char MoneyUSSD[11] = "*111#";
unsigned int Settings = 0b0000000001001010;
//bit0 - (0)пер.отпр.-0-движ;1-заж.;2-пост.;3-адапт.		1
//bit1 - (1)пер.отпр.-0-движ;1-заж.;2-пост.;3-адапт.		2
//bit2 - опред. роуминг по карте							4
//bit3 - опред. роуминг по GSM				 				8
//bit4 - reserved											16
//bit5 - отправлять данные в роуминге						32
//bit6 - отвечать на СМС  в роуминге			 			64
//bit7 - (0)фильтр на стоянкак-0-нет;1-по прогр.движ.		128
//bit8 - (1)2-по заж.3-reserved								256
//bit9 - reserved											512
//bit10 - reserve											1024
//bit11 - Не разрывать соединение							2048
//bit12 - reserve											4096
//bit13 - reserve											8192
//bit14 - reserve											16384
//bit15 - reserve											32768

unsigned int Settings1 = 0b0000000000000000;
//bit0 - при изм.заж.зап.коорд.								1
//bit1 - при изм.заж.начинать отпр.							2
//bit2 - при откл/вкл.вн.напр.зап.коорд.					4
//bit3 - при откл/вкл.вн.напр.начинать отпр.				8
//bit4 - IN1 - тревога										16
//bit5 - IN2 - тревога										32
//bit6 - Тревога1 акт.выс.						 			64
//bit7 - Тревога2 акт.выс.									128
//bit8 - отправл.gsmloc при опред. по gps					256
//bit9 - при изм.движ.зап.									512
//bit10 - при изм.движ.отпр.								1024
//bit11 - при изм. IN1 отпр.								2048
//bit12 - при изм. IN2 отпр.								4096
//bit13 - reserve											8192
//bit14 - reserve											16384
//bit15 - reserve											32768
#endif

unsigned long Trip = 0;

unsigned int Period, send_per;
unsigned char gsm_busy = 0, gps_ok = 0, gsm_reg = 0, gprs_reg = 0, gsm_fun = 0, RoumingSIM = 0, RoumingMAP = 0, Rouming = 0, Move = 0, Ignition = 0, Jamming = 0;
unsigned char gps_state = 0, gprs_state = 0, out_send = 0, out_send1 = 0, send_prgrs = 0, send_imm = 0, alert = 0, cpureset = 0, dis_simon = 0, dis_simon1 = 0, dis_gpson = 0;
volatile unsigned char insms_f = 0, incall_prgrs = 0;
unsigned char sendsms_f = 0, cmd_ret = 0, simcom_on = 0, prm_f = 0, gsmloc_f = 0, money_f = 0, trip_f = 0, offsim_f = 0, gpsreset = 0, ident_f = 0;
const char *cmd;
char in_byte1, in_byte2;
char * ibp1, * ibp2, * ibpt1, * tmpp1, * tmpp2;
unsigned int outb1_col = 0;
unsigned int Vbt = 0, Money = 0;
unsigned int Vcc = 0;
unsigned char rssi = 0;
signed char Tm = 0;
volatile unsigned int send_cnt = 0, prm_cnt = 0, gsmloc_cnt = 0, money_cnt = 0, trip_cnt = 0, debug1_cnt = 0, debug2_cnt = 0, can1_cnt = 0, can2_cnt = 0;
volatile unsigned int ADC_cnt = 0, INx_cnt = 0, freq_cnt = 0, ident_cnt = 0, offsim_cnt = 0, tripsv_cnt = 0, smsread_cnt = 0, netchk_cnt = 0; 
volatile unsigned int stealth_ontmc = 0, gprsregt_cnt = 0, senderrt_cnt = 0;
unsigned int Stealth_per1 = 0;
unsigned int per_cnt = 0, col_cnt = 0;
unsigned int stealth_cnt = 0;
unsigned int senderror_cnt = 0, gprsregerr_cnt = 0;
unsigned char move_d = 0, CRC = 0, uv_f = 0;
char IMEI[19] = {19, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0, 0};
char ICC[6] = {0,0,0,0,0,0};
unsigned char myIP[4] = {0,0,0,0};
unsigned char IPtemp[4];
unsigned int cnt1;
unsigned char cnt73;
unsigned int out_cnt = 0;
volatile unsigned int Speed = 0;
volatile unsigned long Latitude = 0, Longitude = 0;
unsigned long Lat_old = 0, Lon_old = 0, Latf = 0, Lonf = 0;
volatile unsigned char pps1 = 0, SatUs = 0, Year = 0, Month = 0, Day = 0, Hour = 0, Minute = 0, Second = 0, Turn = 0;
struct tm time1;
time_t time2, time2_old = 0;
volatile time_t timec;
unsigned long Lat_old1 = 0, Lon_old1 = 0, timeD = 0;
signed long LatD = 0, LonD = 0;
unsigned int Sync_cnt = 0xFFFE;

unsigned int Course_old = 0;
volatile unsigned int Course = 0;
unsigned int Sts_d = 0;
//bit0 - Speed_lim				1
//bit1 - RoumingMAP				2
//bit2 - gprs_reg				4
//bit3 - Ignition				8
//bit4 - Move					16
//bit5 - gsm_reg0				32
//bit6 - gsm_reg1				64
//bit7 - gsm_reg2				128
//bit8 - Jamming				256
//bit9 - Вн.напр.откл.			512
//bit10 - Авт.заблокирован		1024
//bit11 - Vcc_lim				2048
//bit12 - Vbt_lim				4096
//bit13 - Geo-fence				8192
//bit14 - IN1					16384
//bit15 - IN2					32768
unsigned int deg, cnt2, cnt3, cnt5, IPnbr = 0, sms_nbr = 0, cpurst_cnt = 0;
const char * cmd;
unsigned int MCC = 0, MNC = 0, LAC = 0, CID = 0;
unsigned char SPI_ret = 0, I2C_ret = 0, eng_block = 0, Event_nbr, jd = 0;
unsigned int pkt, do_req = 0;
//bit0 - Send debug1			1
//bit1 - Send debug2			2
//bit2 - Send ident				4
//bit3 - Send prm				8
//bit4 - Send gsmloc			16
//bit5 - Send money				32
//bit6 - Send trip				64
//bit7 - Simcom offon			128
//bit8 - CPU reset				256
//bit9 - GPS reset				512
//bit10 - reserve				1024
//bit11 - reserve				2048
//bit12 - reserve				4096
//bit13 - eng block				8192
//bit14 - eng soft block		16384
//bit15 - eng unblock			32768
long double lat1a, lat2a, lon1a, lon2a, dlat, dlon, tmpdst, dst;
double Lat, Lon;
unsigned int NbrWrCnt = 0, simof_cnt1 = 0, simof_cnt2 = 0, simof_cnt3 = 0;
unsigned char CurPageNbr = 0;

unsigned char CRC_can;
volatile unsigned char speed_can = 0xFF, accel_can = 0xFF, fuel_can = 0xFF, temp_can = 0xFF;
volatile unsigned int rpm_can = 0xFFFF;
volatile unsigned long fuela_can = 0xFFFFFFFF, mth_can = 0xFFFFFFFF, dist_can = 0xFFFFFFFF;
volatile unsigned char speed_cant = 0, accel_cant = 0, fuel_cant = 0, temp_cant = 0;
volatile unsigned int rpm_cant = 0;
volatile unsigned long fuela_cant = 0, mth_cant = 0, dist_cant = 0;

unsigned char SU_state = 0;
unsigned char SU_recb = 0;
unsigned char SU_recbc = 0, SU_recbc1 = 0;
unsigned char SU_recf = 0;

unsigned long debug1;
unsigned int debug2;

unsigned char ADC1 = 0, ADC2 = 0, INx = 0;
unsigned int ADCtmp, freq1 = 0, freq2 = 0;
unsigned int in_tmp1 = 0, in_tmp2 = 0, cntw1, cntw2;

unsigned long eeprom_p1 = 0, eeprom_p2 = 0, eeprom_p2tmp = 0, out_buf_col = 0, eeprom_p1tmp;

unsigned int Speed_lim = 250, Vcc_lim = 0, Vbt_lim = 0;
unsigned int Speed_limf = 0, Vcc_limf = 0, Vbt_limf = 0;
unsigned char reindent = 0;

volatile unsigned char can_read = 0, t3_f = 0;

#if defined(OZUmem)
const unsigned int eeprom_size = 3000;
#endif
#if defined(I2Cmem)
const unsigned int eeprom_size = 65530;
#endif
#if defined(SPImem)
const unsigned long eeprom_size = 4194300;
#endif

const unsigned long LatU[46] = {483099800,486404600,489821200,489826800,495251400,503543000,503714600,505805900,
					   507706500,508603300,513327100,516147200,515512300,517306700,518505800,518671900,
					   519327100,517614200,516962200,515770800,514456600,513522300,514316000,511813300,
					   518992900,518418000,510943800,509612300,502033900,500809400,496193600,493384200,
					   480592900,479271100,474860700,469745900,455968700,442571800,442533700,456164700,
					   461790500,463153300,470505200,481451500,487127500,480220100};

const unsigned long LonU[46] = {225811100,222473400,226263400,229715200,227025200,237872600,240329400,241554200,
					   240777600,241748000,237086800,237072100,239780900,243520900,243978600,249966200,
					   252444700,265569000,271554300,272255100,277780200,293333900,300734400,306432300,
					   317425800,338364000,341669900,349155400,355267100,374049300,382629000,396482000,
					   393893800,386502800,380137400,378935000,365752500,358517900,322803900,294967400,
					   293474400,303050000,302208900,292945400,275216100,250169700};
const unsigned char day_tab[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
unsigned int ret_err = 0;
void fun1(void);

char * CMD2, * ANS2;
unsigned int TM2;
unsigned char WaitAnsTf = 0, SendCmdTf = 0;

#if defined(RFM73)
const unsigned char Bank0_Reg[ BANK0_ENTRIES ][ 2 ]={
   {  0, 0x0B }, // receive, enabled, CRC 1, enable interupts
   {  1, 0x02 }, // auto-ack on all pipes enabled
   {  2, 0x03 }, // Enable pipes 0,1
   {  3, 0x01 }, // 3 bytes addresses
   {  4, 0xff }, // auto retransmission delay 4000 ms, 15 times
   {  5, 0x01 }, // channel 1
   {  6 ,0x07 }, // data rate 1Mbit, power 5dbm, LNA gain high
   {  7, 0x07 }, // why write this at all?? but seems required to work...
   {  8, 0x00 }, // clear Tx packet counters
   { 23, 0x00 }, // fifo status
};

const unsigned long Bank1_Reg0_13[] = {
   0xE2014B40,
   0x00004BC0,
   0x028CFCD0,
   0x41390099,
   0x1B8296D9, 
   0xA67F0624,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00000000,
   0x00127300,
   0x36B48000
};   
   
const unsigned char Bank1_Reg14[] = {
   0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF };   

unsigned char RX0_Address[]={ 0x34, 0x43, 0xFF };
unsigned char RX1_Address[]={ 0x34, 0x43, 0x00 };
unsigned char rfm73tmp, rfm73cmd;
unsigned char rfm73buf[10];
#endif

#define SendCmd(CMD, ANS, TM) {SendCmd2(CMD, ANS, TM);OS_Wait(!SendCmdTf);}
#define WaitAns(ANS, TM) {WaitAns2(ANS, TM);OS_Wait(!WaitAnsTf);}
#define SendCmd1(CMD) {cmd = CMD;while(*cmd){U1TXREG = *cmd++;OS_Wait_TO(U1STAbits.TRMT, 1);}}
#define WaitAns1(TM) {ret_err = 0;ibp1 = ib1;RTS = 0;OS_Wait_TO(RTS, TM);if(OS_IsTimeout()){RTS = 1;ret_err = 1;}}
#define SIMCOM_OFF {OS_Wait_TO((!gsm_busy && !send_prgrs), 200);if(OS_IsTimeout()){save_pt();asm("RESET");}dis_simon1 = 1;gsm_busy = 1;if(GSM_STS){GSM_ON = 1;OS_Delay(40);GSM_ON = 0;}fun1();}

unsigned char GetCurrentPageNbr(void);
unsigned int GetCurrentPageSts(void);
void wr_pkt(void);

#if defined(RFM73)
unsigned char
rfm73_SPI_RW(unsigned char value)
{
	unsigned char i;
	for(i =0; i < 8; i++)
	{
		if(value & 0x80)
			SPI_OUT = 1;
		else
			SPI_OUT = 0;
		value = (value << 1);
		SPI_CLK = 1;   
		if(SPI_IN)
			value |= 1;      
		SPI_CLK = 0;
	}
	return value;
}

void
rfm73_register_write(unsigned char reg, unsigned char value)
{
	if(reg < RFM73_CMD_WRITE_REG)
	{
		reg |= RFM73_CMD_WRITE_REG;
	}
	SPI_CS = 0;
	(void)rfm73_SPI_RW(reg);
	(void)rfm73_SPI_RW(value);
	SPI_CS = 1;
}

unsigned char
rfm73_register_read(unsigned char reg)
{
	unsigned char value;
	if(reg < RFM73_CMD_WRITE_REG)
	{
		reg |= RFM73_CMD_READ_REG;
	}
	SPI_CS = 0;
	(void)rfm73_SPI_RW(reg);
	value = rfm73_SPI_RW(0);
	SPI_CS = 1;
	return value;
}

void
rfm73_buffer_read(unsigned char reg, unsigned char pBuf[], unsigned char length)
{
	unsigned char i;
	if(reg < RFM73_CMD_WRITE_REG)
	{
		reg |= RFM73_CMD_READ_REG;
	}
	SPI_CS = 0;
	(void)rfm73_SPI_RW(reg);
	for(i = 0; i < length; i++)
	{
		pBuf[i] = rfm73_SPI_RW(0);
	}
	SPI_CS = 1;
}

void
rfm73_buffer_write(char reg, const unsigned char pBuf[], unsigned char length)
{
	unsigned char i;
	if(reg < RFM73_CMD_WRITE_REG)
	{
		reg |= RFM73_CMD_WRITE_REG;
	}
	SPI_CS = 0;
	(void)rfm73_SPI_RW(reg);
	for(i = 0; i < length; i++)
	{
		(void)rfm73_SPI_RW(pBuf[ i ]);
	}
	SPI_CS = 1;
}

void
rfm73_bank(unsigned char b)
{
	unsigned char st = 0x80 & rfm73_register_read(RFM73_REG_STATUS);
	if((st && (b == 0)) || ((st == 0 ) && b))
	{
		rfm73_register_write(RFM73_CMD_ACTIVATE, 0x53);
	}
}

void
rfm73_init_bank1(void)
{
	unsigned char i, j;
	unsigned char WriteArr[ 12 ];

	rfm73_bank(1);

	for(i = 0; i <= 8; i++)
	{
		for(j = 0; j < 4; j++)
		{
			WriteArr[j] = (Bank1_Reg0_13[i] >> (8 * (j))) & 0xFF;
		}
		rfm73_buffer_write(i, WriteArr, 4);
	}

	for(i = 9; i <= 13; i++)
	{
		for(j = 0; j < 4; j++)
		{
			WriteArr[j]=(Bank1_Reg0_13[i] >> (8 * (3 - j))) & 0xFF;
		}
		rfm73_buffer_write(i, WriteArr, 4);
	}

	rfm73_buffer_write(14, Bank1_Reg14, 11);

	for(j = 0; j < 4; j++)
	{
		WriteArr[j] = (Bank1_Reg0_13[4] >> (8 * (j))) & 0xFF;
	}

	WriteArr[0] = WriteArr[0] | 0x06;
	rfm73_buffer_write(4, WriteArr, 4);

	WriteArr[0] = WriteArr[0] & 0xF9;
	rfm73_buffer_write(4, WriteArr, 4);

	rfm73_bank(0);
}

unsigned char
rfm73_is_present(void)
{
	unsigned char st1, st2;
	st1 = rfm73_register_read(RFM73_REG_STATUS);
	rfm73_register_write(RFM73_CMD_ACTIVATE, 0x53);
	st2 = rfm73_register_read(RFM73_REG_STATUS);
	rfm73_register_write(RFM73_CMD_ACTIVATE, 0x53);
	return (st1 ^ st2) == 0x80;
}

void
rfm73_mode_receive(void)
{
	unsigned char value;

	rfm73_register_write(RFM73_CMD_FLUSH_RX, 0);
	value = rfm73_register_read(RFM73_REG_STATUS);
	rfm73_register_write(RFM73_REG_STATUS ,value);
	SPI_CE = 0;
	value = rfm73_register_read(RFM73_REG_CONFIG);
	value |= 0x01;
	value |= 0x02;
	rfm73_register_write(RFM73_REG_CONFIG, value);
	SPI_CE = 1;
}

void
rfm73_mode_transmit(void)
{
	unsigned char value;

	rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
	value = rfm73_register_read(RFM73_REG_STATUS);
	rfm73_register_write(RFM73_REG_STATUS, value);
	SPI_CE = 0;
	value = rfm73_register_read(RFM73_REG_CONFIG);
	value &= 0xFE;
	value |= 0x02;
	rfm73_register_write(RFM73_REG_CONFIG, value);
	SPI_CE = 1;
}

void
rfm73_mode_standby(void)
{
	unsigned char value;
	SPI_CE = 0;
	value = rfm73_register_read(RFM73_REG_CONFIG);
	value |= 0x02;
	rfm73_register_write(RFM73_REG_CONFIG, value);
}

void
rfm73_mode_powerdown(void)
{
	unsigned char value;
	SPI_CE = 0;
	value = rfm73_register_read(RFM73_REG_CONFIG);
	value &= 0xFD;
	rfm73_register_write(RFM73_REG_CONFIG, value);
}

void
rfm73_channel(unsigned char ch)
{
	rfm73_register_write(RFM73_REG_RF_CH, ch & 0x7E);
}

void
rfm73_air_data_rate(unsigned char rate)
{
	unsigned char value;
	SPI_CE = 0;
	value = rfm73_register_read(RFM73_REG_RF_SETUP);
	value &= 0x07;
	if( rate == 0 )
	{
		value |= 0x20;
	}
	if(rate > 1 )
	{
		value |= 0x08;
	}
	rfm73_register_write(RFM73_REG_RF_SETUP, value);
	SPI_CE = 1;
}

void
rfm73_crc_length(unsigned char len)
{
	unsigned char val;
	if(len > 2)
	{
		len = 2;
	}
	val = rfm73_register_read(RFM73_REG_CONFIG);
	if(len == 0)
	{
		val &= 0xF3;
		rfm73_register_write(RFM73_REG_EN_AA, 0);
		rfm73_register_write(RFM73_REG_CONFIG, val);
	}
	else
	{
		rfm73_register_write(RFM73_REG_EN_AA, 0x3F);
		val &= 0xFB;
		if(val == 2)
		{
			val |= 0x04;
		}
		rfm73_register_write(RFM73_REG_CONFIG, val);
	}
}

void
rfm73_address_length(unsigned char len)
{
	if(len > 5)
	{
		len = 5;
	}
	if(len < 3)
	{
		len = 3;
	}
	rfm73_register_write(RFM73_REG_SETUP_AW, len - 2);
}

unsigned char
rfm73_transmit_fifo_full(void)
{
	unsigned char s;
	s = rfm73_register_read(RFM73_REG_FIFO_STATUS);
	return (s & FIFO_STATUS_TX_FULL) != 0;
}
    
unsigned char
rfm73_receive_fifo_empty(void)
{
	unsigned char s;
	s = rfm73_register_read(RFM73_REG_FIFO_STATUS);
	return (s & FIFO_STATUS_RX_EMPTY) != 0;
}

void
rfm73_receive_address_p0(const unsigned char address[3])
{
	rfm73_buffer_write(RFM73_REG_RX_ADDR_P0, address, 3);
}

void
rfm73_receive_address_p1(const unsigned char address[3])
{
	rfm73_buffer_write(RFM73_REG_RX_ADDR_P1, address, 3);
}

void
rfm73_receive_address_pn(unsigned char channel, unsigned char address)
{
	rfm73_register_write(RFM73_REG_RX_ADDR_P0 + channel, address);
}

void
rfm73_transmit_address(const unsigned char address[])
{
	rfm73_buffer_write(RFM73_REG_TX_ADDR, address, 3);
} 

unsigned char
rfm73_retransmit_count(void)
{
	return rfm73_register_read(RFM73_REG_OBSERVE_TX) & 0x0F;
}

unsigned char
rfm73_lost_packets_count(void)
{
	return (rfm73_register_read(RFM73_REG_OBSERVE_TX) >> 4) & 0x0F;
}

void
rfm73_pipe_autoack(unsigned char pipe, unsigned char enabled )
{
	unsigned char val = rfm73_register_read(RFM73_REG_EN_AA);
	if(pipe > 5)
	{ 
      pipe = 5; 
   }   
   if( enabled ){
      val |= 1 << pipe;
   } else {
      val &= ~ ( 1 << pipe );
   }
   rfm73_register_write( RFM73_REG_EN_AA, val ); 
}

void
rfm73_pipe_enable(unsigned char pipe, unsigned char enabled)
{
	unsigned char val = rfm73_register_read(RFM73_REG_EN_RXADDR);
	if(pipe > 5)
	{
		pipe = 5;
	}
	if(enabled)
	{
		val |= 1 << pipe;
	}
	else
	{
		val &= ~ (1 << pipe);
	}
	rfm73_register_write(RFM73_REG_EN_RXADDR, val);
}

void
rfm73_lost_packets_reset(void)
{
	unsigned char val = rfm73_register_read(RFM73_REG_RF_CH);
	rfm73_register_write(RFM73_REG_RF_CH, val);
}

void
rfm73_retransmit_delay_attempts(unsigned char d, unsigned char n)
{
	rfm73_register_write(RFM73_REG_SETUP_RETR, (n & 0x0F) | ((d & 0x0F) << 4));
}

void
rfm73_lna_high(void)
{
	unsigned char val = rfm73_register_read(RFM73_REG_RF_SETUP);
	val |= 0x01;
	rfm73_register_write(RFM73_REG_RF_SETUP, val);
}

void
rfm73_lna_low(void)
{
	unsigned char val = rfm73_register_read(RFM73_REG_RF_SETUP);
	val &= 0xFE;
	rfm73_register_write(RFM73_REG_RF_SETUP, val);
}

void
rfm73_power(unsigned char level)
{
	if(level > 3)
	{
		level = 3;
	}
	SPI_CE = 0;
	unsigned char val = rfm73_register_read(RFM73_REG_RF_SETUP);
	val &= 0x09;
	val |= 0x30;
	val |= (level << 1);
	rfm73_register_write(RFM73_REG_RF_SETUP, val);
	SPI_CE = 1;
}

void
rfm73_channel_payload_size(unsigned char channel, unsigned char size)
{
	unsigned char val;
	if(size > 32)
	{
		size = 32;
	}
	val = rfm73_register_read(RFM73_REG_DYNPD);
	if(size == 0)
	{
		val |= 1 << channel;
	}
	else
	{
		val &= ~ (1 << channel);
	}
	rfm73_register_write(RFM73_REG_DYNPD, val);
	rfm73_register_write(RFM73_REG_RX_PW_P0 + channel, size);
}
/*
unsigned char
rfm73_transmit_message(const unsigned char buf[], unsigned char length)
{
	unsigned char tmp;
	if(length > 32)
		length = 32;
	rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
	tmp = rfm73_register_read(RFM73_REG_STATUS);
	rfm73_register_write(RFM73_REG_STATUS, tmp);
	rfm73_buffer_write(RFM73_CMD_W_TX_PAYLOAD, buf, length);
	do{
		tmp = rfm73_register_read(RFM73_REG_STATUS);
		while(SPI_IRQ_IN);
	}while(!(tmp & (STATUS_MAX_RT | STATUS_TX_DS)));
	if((tmp & STATUS_MAX_RT))
	{
		rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
		rfm73_register_write(RFM73_REG_STATUS, tmp);
		return 0;
	}
	else
	{
		rfm73_register_write(RFM73_REG_STATUS, tmp);
		return 1;
	}
}

void
rfm73_transmit_message_once(const unsigned char buf[], unsigned char length)
{
	unsigned char tmp;
	if(length > 32)
	{
		length = 32;
	}
	rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
	tmp = rfm73_register_read(RFM73_REG_STATUS);
	rfm73_register_write(RFM73_REG_STATUS, tmp);
	rfm73_buffer_write(RFM73_CMD_W_TX_PAYLOAD_NOACK, buf, length);
	do{
		tmp = rfm73_register_read(RFM73_REG_STATUS);
		while(SPI_IRQ_IN);
	}while(!(tmp & (STATUS_MAX_RT | STATUS_TX_DS)));
	rfm73_register_write(RFM73_REG_STATUS, tmp);
}
*/
unsigned char
rfm73_receive_next_pipe(void)
{
	unsigned char status = rfm73_register_read(RFM73_REG_STATUS);
	return (status >> 1) & 0x07;
}

unsigned char
rfm73_receive_next_length(void)
{
	return rfm73_register_read(RFM73_CMD_R_RX_PL_WID);
}

unsigned char
rfm73_receive(unsigned char * pipe, unsigned char buf[], unsigned char * length)
{
	unsigned char tmp;
	unsigned char p = rfm73_receive_next_pipe();
	if(p == 0x07)
	{
		tmp = rfm73_register_read(RFM73_REG_STATUS);
		rfm73_register_write(RFM73_REG_STATUS, tmp);
		return 0;
	}
	* pipe = p;
	* length = rfm73_receive_next_length();
	rfm73_buffer_read(RFM73_CMD_R_RX_PAYLOAD, buf, * length);
	tmp = rfm73_register_read(RFM73_REG_STATUS);
	rfm73_register_write(RFM73_REG_STATUS, tmp);
	return 1;
}
#endif

#if defined(RFM73)
void RFM73T(void)
{
SPI_CS = 1;
SPI_CE = 0;
SPI_CLK = 0;
SPI_OUT = 0;
	LED_ER = 1;

	OS_Delay(2);
	rfm73_bank(0);
	for(cnt73 = 0; cnt73 < BANK0_ENTRIES; cnt73++)
	{
		rfm73_register_write(Bank0_Reg[cnt73][0], Bank0_Reg[cnt73][1]);
	}
	cnt73 = rfm73_register_read(29);
	if(cnt73 == 0)
	{
		rfm73_register_write(RFM73_CMD_ACTIVATE, 0x73);
	}
	rfm73_register_write(29, 0x07);
	rfm73_register_write(28, 0x3F);
	rfm73_channel_payload_size(0, 0);
	rfm73_init_bank1();
	OS_Delay(2);

	rfm73_power(3);
	rfm73_channel(0);
	rfm73_air_data_rate(1);
	rfm73_lna_high();
	rfm73_retransmit_delay_attempts(5, 15);
	rfm73_mode_transmit();

rfm73cmd = 0xAA;
	for (;;)
	{
//		if(rfm73cmd != 0)
		{
			RX0_Address[2] = 0x00;
			rfm73_transmit_address(RX0_Address);
			rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
			rfm73tmp = rfm73_register_read(RFM73_REG_STATUS);
			rfm73_register_write(RFM73_REG_STATUS, rfm73tmp);
			rfm73buf[0] = 0x00;
			rfm73buf[1] = 0x00;
			rfm73buf[2] = rfm73cmd;
			rfm73_buffer_write(RFM73_CMD_W_TX_PAYLOAD, rfm73buf, 3);
			do{
				rfm73tmp = rfm73_register_read(RFM73_REG_STATUS);
			}while(!(rfm73tmp & (STATUS_MAX_RT | STATUS_TX_DS)));
			if((rfm73tmp & STATUS_MAX_RT))
			{
				rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
				rfm73_register_write(RFM73_REG_STATUS, rfm73tmp);
			}
			else
			{
				rfm73_register_write(RFM73_REG_STATUS, rfm73tmp);
			}
			rfm73cmd = 0;
		}
LED_ER = 1;
OS_Delay(5);
LED_ER = 0;
	OS_Delay(20);
//		OS_Yield();
	}
}
#endif

void
T1set(void)//секундный таймер событий
{
	T1CON = 0x00; //Stops the Timer1 and reset control reg.
	TMR1 = 0x00; //Clear contents of the timer register
	PR1 = 0xF424; //Load the Period register with the value 0xFFFF
	IPC0bits.T1IP = 0x01; //Setup Timer1 interrupt for desired priority level
	IFS0bits.T1IF = 0; //Clear the Timer1 interrupt status flag
	IEC0bits.T1IE = 1; //Enable Timer1 interrupts
	T1CONbits.TCKPS1 = 1;
	T1CONbits.TCKPS0 = 1;
	T1CONbits.TON = 1; //Start Timer1 with prescaler settings at 1:1 and
	//clock source set to the internal instruction cycle
}

void
T2set(void)//100мс OStimer
{
	T2CON = 0x00; //Stops the Timer2 and reset control reg.
	TMR2 = 0x00; //Clear contents of the timer register
	PR2 = 0x61A7; //Load the Period register with the value 0xFFFF
	IPC1bits.T2IP = 0x01; //Setup Timer2 interrupt for desired priority level
	IFS0bits.T2IF = 0; //Clear the Timer2 interrupt status flag
	IEC0bits.T2IE = 1; //Enable Timer2 interrupts
	T2CONbits.TCKPS1 = 1;
	T2CONbits.TCKPS0 = 0;
	T2CONbits.TON = 1; //Start Timer2 with prescaler settings at 1:256 and
	//clock source set to the internal instruction cycle
}

void
T3set(void)//50мс таймер
{
	T3CON = 0x00; //Stops the Timer3 and reset control reg.
	TMR3 = 0x00; //Clear contents of the timer register
//	PR3 = 0x30D3; //Load the Period register with the value 0xFFFF
	PR3 = 0xF424; //Load the Period register with the value 0xFFFF
	IPC2bits.T3IP = 0x01; //Setup Timer3 interrupt for desired priority level
	IFS0bits.T3IF = 0; //Clear the Timer3 interrupt status flag
	IEC0bits.T3IE = 1; //Enable Timer3 interrupts
//	T3CONbits.TCKPS1 = 1;
//	T3CONbits.TCKPS0 = 0;
	T3CONbits.TCKPS1 = 1;
	T3CONbits.TCKPS0 = 1;
	T3CONbits.TON = 1; //Start Timer3 with prescaler settings at 1:256 and
	//clock source set to the internal instruction cycle
}

void
T4set(void)//833.33мкс таймер UART
{
	T4CON = 0x00; //Stops the Timer4 and reset control reg.
	TMR4 = 0x00; //Clear contents of the timer register
	PR4 = 13332; //Load the Period register with the value 0xFFFF
	IPC6bits.T4IP = 0x01; //Setup Timer4 interrupt for desired priority level
	IFS1bits.T4IF = 0; //Clear the Timer4 interrupt status flag
	IEC1bits.T4IE = 1; //Enable Timer4 interrupts
	T4CONbits.TCKPS1 = 0;
	T4CONbits.TCKPS0 = 0;
	T4CONbits.TON = 1; //Start Timer4 with prescaler settings at 1:256 and
	//clock source set to the internal instruction cycle
}

void
U1set2400(void)
{
	RPINR18 = 0x1F16;			//GSM_TX - RP22
	RPOR11 = 0x0300;			//GSM_RX - RP23
	U1MODE = 0b1100000000000000;
	U1STA = 0b0010010100010000;
	U1BRG = 416;				//скорость по GSM
 	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 0;
}

void
U1off(void)
{
	U1MODE = 0;//hi
}

void
U1set(void)
{
	RPINR18 = 0x1F16;			//GSM_TX - RP22
	RPOR11 = 0x0300;			//GSM_RX - RP23
	U1MODE = 0b1100000000001000;//hi
	U1STA = 0b0010010100010000;
	U1BRG = 34;				//скорость по GSM 115200
 	IFS0bits.U1RXIF = 0;
	IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 0;
}

void
U2set(void)
{
	RPINR19 = 0x1F11;			//GPS_TX - RP17
	RPOR9 = 0x0005;				//GPS_RX - RP18
	U2MODE = 0b1100000000000000;//low
	U2STA = 0b0010010100010000;
#if defined(SIM908type)
	U2BRG = 103;				//скорость по GPS 9600
#endif
#if defined(L50type) || defined(ZTR1type)
	U2BRG = 207;				//скорость по GPS 4800L50
#endif
#if defined(L80type)
	U2BRG = 103;				//скорость по GPS 9600L80
#endif
	IFS1bits.U2TXIF = 0;
	IEC1bits.U2TXIE = 0;
	IFS1bits.U2RXIF = 0;
	IEC1bits.U2RXIE = 1;
}

void
ADCset(void)
{
#if defined(SIM908type)
	AD1PCFG = 0b0001100111111101; 	// AN1 as analog, all other pins are digital
#endif
#if defined(L50type)
	AD1PCFG = 0b0001110111111101; 	// AN1 as analog, all other pins are digital
#endif
#if defined(L50type) || defined(SIM908type)
	AD1CON1 = 0x0000; 				// SAMP bit = 0 ends sampling
									// and starts converting
	AD1CON2 = 0b0000000000000000;	//Vref = Vcc
	AD1CHS = 0x0001; 				// Connect AN1 as CH0 input
									// in this example AN1 is the input
	AD1CSSL = 0;
	AD1CON3 = 0x0002; 				// Manual Sample, Tad = 2 Tcy
	AD1CON1bits.ADON = 1; 			// turn ADC ON
#endif
}

void
I2Cset(void)
{
	CloseI2C2();
	ConfigIntI2C2(MI2C_INT_OFF);
	OpenI2C2(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD &
		I2C_IPMI_DIS & I2C_7BIT_ADD &
		I2C_SLW_DIS & I2C_SM_DIS &
		I2C_GCALL_DIS & I2C_STR_DIS &
		I2C_NACK & I2C_ACK_DIS & I2C_RCV_DIS &
		I2C_STOP_DIS & I2C_RESTART_DIS &
		I2C_START_DIS, 0x4F);
}

int main (void)
{
#if defined(SIM908type)
	TRISA = 0b0000011100001110;
	TRISB = 0b1110110010001100;
	TRISC = 0b0000000001000010;
	DTR = 0;
	SPI_CS2 = 1;
#endif

#if defined(L50type)
	TRISA = 0b0000010000001110;
	TRISB = 0b1000010010001100;
	TRISC = 0b0000000001100011;
#endif

#if defined(ZTR1type)
	TRISA = 0b0000000000001100;
	TRISB = 0b0000010000000000;
	TRISC = 0b0000000001000010;
#endif

#if defined(ZTR_BTN)
	TRISA = 0b0000000000001100;
	TRISB = 0b0000010000000000;
	TRISC = 0b0000000001100011;
#endif

#if defined(L50type) || defined(ZTR1type)
	GPS_R = 0;
	GPS_OFFB = 1;
	GPS_ON = 0;
#endif

#ifndef ZTR1type
	ALLOFF = 0;
	SPI_CS1 = 1;
#endif

	GSM_ON = 0;
	OFF900 = 0;

	AD1PCFG = 0b0001111111111111;

	T1set();
	T2set();
	T3set();
#if defined(CAN)
	T4set();
#endif
//	U1set2400();
	U1set();
	U2set();
#ifndef ZTR1type
	I2Cset();
	ADCset();
#endif

	OS_Init();

	OS_Task_Create(0, START);

	for(;;)
		OS_Run();
}

void START(void)
{
	OS_Delay(1);
#if defined(RFM73)
	OS_Task_Create(0, RFM73T);
#endif

	while(1)
	{
		OS_Delay(100);
	}
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag and Return from ISR

}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS0bits.T2IF = 0; //Reset Timer1 interrupt flag and Return from ISR
	OS_Timer();
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS0bits.T3IF = 0; //Reset Timer1 interrupt flag and Return from ISR
	t3_f = 0;
}

#if defined(CAN)
void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T4Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS1bits.T4IF = 0; //Reset Timer1 interrupt flag and Return from ISR
}
#endif

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void)
{
	IFS1bits.U2RXIF = 0;
}
