//#define SIM908type
#define L50type
//#define ZTR1type
//#define SPImem
//#define I2Cmem
#define OZUmem
//#define ZTR_BTN
//#define CAN
//#define L80type
//#define RFM73

#define Version		37

void    START(void);
void    GPS_1(void);
void    GPS_2(void);
void    GSM_1(void);
void    GSM_2(void);
void    GSM_3(void);
void    GSM_4(void);
void    GSM_5(void);
void    GSM_6(void);
void    GSM_7(void);
void    GSM_8(void);
void	LED1(void);
void	LED2(void);
void	WaitAnsT(void);
void	SendCmdT(void);
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
#include "DEE Emulation 16-bit.h"
#include <i2c.h>
#include <spi.h>
#include <osa.h>
#if defined(RFM73)
#include <rfm73.h>
#endif

_CONFIG1( JTAGEN_OFF & GCP_ON & GWRP_OFF & BKBUG_OFF & ICS_PGx1 & FWDTEN_ON & WINDIS_ON & FWPSA_PR128	 & WDTPS_PS16384);
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

char ExIP1[16] = "185.25.117.35", ExIP2[16] = "185.25.117.35", ExIP3[16] = "185.25.117.35";
unsigned int ExPort1 = 10033, ExPort2 = 10033, ExPort3 = 10033;
char APN[31] = "www.kyivstar.net", APN_user[31] = "", APN_pass[31] = "";
unsigned char PortPr = 0b00000111;

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
//bit4 - вести ReseCnt										16
//bit5 - отправлять данные в роуминге						32
//bit6 - отвечать на СМС  в роуминге			 			64
//bit7 - (0)фильтр на стоянкак-0-нет;1-по прогр.движ.		128
//bit8 - (1)2-по заж.3-reserved								256
//bit9 - Выключить GPRS										512
//bit10 - reserve											1024
//bit11 - Не разрывать соединение							2048
//bit12 - reserve											4096
//bit13 - save pointers & trip								8192
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
//bit9 - Выключить GPRS										512
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
//bit13 - режим иммобилайзера								8192
//bit14 - reserve											16384
//bit15 - reserve											32768
#endif

unsigned long Trip = 0;

unsigned int Period, send_per;
unsigned char gsm_busy = 0, gps_ok = 0, gsm_reg = 0, gprs_reg = 0, gsm_fun = 0, RoumingSIM = 0, RoumingMAP = 0, Rouming = 0, Move = 0, Ignition = 0, Jamming = 0;
unsigned char gps_state = 0, gprs_state = 0, out_send = 0, send_prgrs = 0, send_imm = 0, alert = 0, cpureset = 0, dis_simon = 0, dis_simon1 = 0, dis_gpson = 0;
volatile unsigned char insms_f = 0, incall_prgrs = 0;
unsigned char sendsms_f = 0, cmd_ret = 0, simcom_on = 0, prm_f = 0, gsmloc_f = 0, money_f = 0, trip_f = 0, offsim_f = 0, gpsreset = 0, ident_f = 0;
const char *cmd;
char in_byte1, in_byte2;
char * ibp1, * ibp2, * ibpt1, * tmpp1, * tmpp2;
char ib1[300], ib2[256], tmp_buf[256], insms_nbr[13], outsms_txt[180], simrev[30];
unsigned char outb[3001], outb1[1000];
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
unsigned int cnt1;
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
//bit13 - Перегрев				8192
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

unsigned char TX_Address[3]={ 0x34, 0x43, 0xFF };
unsigned char rfm73tmp, rfm73cmd, rfm73ret, rfm73snd_f = 0, rfm73snd_cnt;
unsigned int rfm73addr;
unsigned char rfm73buf[5];
unsigned char cnt73, RF_command = 0, rfm73once = 0, Imm_period = 0;
unsigned int RFmod_nbr = 0;
volatile unsigned int imm_cnt = 0;
#endif

void fun1(void);

char * CMD2, * ANS2;
unsigned int TM2;
unsigned char WaitAnsTf = 0, SendCmdTf = 0;

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

void
rfm73_receive_address_p0(const unsigned char address[3])
{
	rfm73_buffer_write(RFM73_REG_RX_ADDR_P0, address, 3);
}

void
rfm73_transmit_address(const unsigned char address[])
{
	rfm73_buffer_write(RFM73_REG_TX_ADDR, address, 3);
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
#endif

void SendCmd2(char * CMD1, char * ANS1, unsigned int TM1)
{
	CMD2 = CMD1;
	ANS2 = ANS1;
	TM2 = TM1;
	SendCmdTf = 1;
}

void
WaitAns2(char * ANS1, unsigned int TM1)
{
	ANS2 = ANS1;
	TM2 = TM1;
	WaitAnsTf = 1;
}

void WaitAnsT(void)
{
	for (;;)
	{
		if(WaitAnsTf)
		{
			ret_err = 0;
			do{
				ibp1 = ib1;
				RTS = 0;
				OS_Wait_TO(RTS, TM2);
				if(OS_IsTimeout())
				{
					RTS = 1;
					ret_err = 1;
					break;
				}
			}while(memcmp(ibp1, ANS2, strlen(ANS2)));
			WaitAnsTf = 0;
		}
		OS_Yield();
	}
}

void SendCmdT(void)
{
	for (;;)
	{
		if(SendCmdTf)
		{
			ret_err = 0;
			while(*CMD2)
			{
				U1TXREG = *CMD2++;
				while(!U1STAbits.TRMT);
			}
			do{
				ibp1 = ib1;
				RTS = 0;
				OS_Wait_TO(RTS, TM2);
				if(OS_IsTimeout())
				{
					RTS = 1;
					ret_err= 1;
					break;
				}
			}while(memcmp(ibp1, ANS2, strlen(ANS2)));
			SendCmdTf = 0;
		}
		OS_Yield();
	}
}

void
fun1(void)
{
	OFF900 = 0;
	simcom_on = 0;
 	gsm_reg = 0;
 	gprs_reg = 0;
	gsm_fun = 0;
	dis_simon1 = 0;
	gsm_busy = 0;
}

int pnpoly(long x, long y)
{
	int c = 0, i, j;
	for (i = 0, j = 45; i < 46; j = i++) 
	{
		if ((((LonU[i]<=y) && (y<LonU[j])) || ((LonU[j]<=y) && (y<LonU[i]))) &&	(x > (LatU[j] - LatU[i]) * (y - LonU[i]) / (LonU[j] - LonU[i]) + LatU[i]))
			c = !c;
	}
	return c;
}

unsigned char
dir_r()
{
	unsigned int r1, r2, a1, a2;

	if(Course_old > Course)
	{
		r1 = Course_old;
		r2 = Course;
	}
	else
	{
		r1 = Course;
		r2 = Course_old;
	}
	a1 = r1 - r2;
	a2 = (360 - r1) + r2;
	if(a1 < a2)
		return a1;
	else
		return a2;
}

void
save_pt(void)
{
	if(Settings & 8192)
	{
		DataEEWrite((unsigned int)(eeprom_p1 >> 16), 0x02);
		DataEEWrite((unsigned int)eeprom_p1, 0x03);
		DataEEWrite((unsigned int)(eeprom_p2 >> 16), 0x04);
		DataEEWrite((unsigned int)eeprom_p2, 0x05);

		DataEEWrite((unsigned int)(Trip >> 16), 0x06);
		DataEEWrite((unsigned int)Trip, 0x07);
	}
}

void
write_param(void)
{

	DataEEWrite(ExPort1, 0x09);
	DataEEWrite(ExPort2, 0x0A);
	DataEEWrite(ExPort3, 0x0B);
	DataEEWrite(Period1, 0x0C);
	DataEEWrite(Period2, 0x0D);
	DataEEWrite(prm_per, 0x0E);
	DataEEWrite(gsmloc_per, 0x0F);
	DataEEWrite(money_per, 0x10);
	DataEEWrite(trip_per, 0x11);
	DataEEWrite(debug1_per, 0x12);
	DataEEWrite(debug2_per, 0x13);
	DataEEWrite(can1_per, 0x14);
	DataEEWrite(can2_per, 0x15);
	DataEEWrite(ADC_per, 0x16);
	DataEEWrite(INx_per, 0x17);
	DataEEWrite(freq_per, 0x18);

	DataEEWrite(Send_per1, 0x1A);
	DataEEWrite(Send_per2, 0x1B);
	DataEEWrite(Vbt_off, 0x1C);
	DataEEWrite(Settings, 0x1E);
	DataEEWrite(Settings1, 0x1F);
	DataEEWrite(((unsigned int)SMS_pass[1] << 8) + SMS_pass[0], 0x20);
	DataEEWrite(((unsigned int)SMS_pass[3] << 8) | SMS_pass[2], 0x21);
	DataEEWrite(Stealth_per, 0x22);
	DataEEWrite(Stealth_ontm, 0x23);
	DataEEWrite(ident_per, 0x24);
	DataEEWrite(offsim_per, 0x25);
	DataEEWrite(Speed_lim, 0x26);
	DataEEWrite(Vcc_lim, 0x27);
	DataEEWrite(Vbt_lim, 0x28);
	DataEEWrite(Period3, 0x29);
	DataEEWrite(Rsindp, 0x2A);
	DataEEWrite(Sync, 0x2B);
	DataEEWrite((unsigned int)PortPr, 0x2C);

	cntw1 = 0x2D;
	for(cntw2 = 0; cntw2 < 16; cntw2 = cntw2 + 2)
	{
		DataEEWrite(((unsigned int)ExIP1[cntw2+1] << 8) + ExIP1[cntw2], cntw1);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 16; cntw2 = cntw2 + 2)
	{
		DataEEWrite(((unsigned int)ExIP2[cntw2+1] << 8) + ExIP2[cntw2], cntw1);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 16; cntw2 = cntw2 + 2)
	{
		DataEEWrite(((unsigned int)ExIP3[cntw2+1] << 8) + ExIP3[cntw2], cntw1);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 30; cntw2 = cntw2 + 2)
	{
		DataEEWrite(((unsigned int)APN[cntw2+1] << 8) + APN[cntw2], cntw1);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 30; cntw2 = cntw2 + 2)
	{
		DataEEWrite(((unsigned int)APN_user[cntw2+1] << 8) + APN_user[cntw2], cntw1);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 30; cntw2 = cntw2 + 2)
	{
		DataEEWrite(((unsigned int)APN_pass[cntw2+1] << 8) + APN_pass[cntw2], cntw1);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 10; cntw2 = cntw2 + 2)
	{
		DataEEWrite(((unsigned int)MoneyUSSD[cntw2+1] << 8) + MoneyUSSD[cntw2], cntw1);
		cntw1++;
	}
}

void
read_param(void)
{
	ExPort1 = DataEERead(0x09);
	ExPort2 = DataEERead(0x0A);
	ExPort3 = DataEERead(0x0B);
	Period1 = DataEERead(0x0C);
	Period2 = DataEERead(0x0D);
	prm_per = DataEERead(0x0E);
	gsmloc_per = DataEERead(0x0F);
	money_per = DataEERead(0x10);
	trip_per = DataEERead(0x11);
	debug1_per = DataEERead(0x12);
	debug2_per = DataEERead(0x13);
	can1_per = DataEERead(0x14);
	can2_per = DataEERead(0x15);
	ADC_per = DataEERead(0x16);
	INx_per = DataEERead(0x17);
	freq_per = DataEERead(0x18);

	Send_per1 = DataEERead(0x1A);
	Send_per2 = DataEERead(0x1B);
	Vbt_off = DataEERead(0x1C);
	Settings = DataEERead(0x1E);
	Settings1 = DataEERead(0x1F);
	SMS_pass[0] = (unsigned char)DataEERead(0x20);
	SMS_pass[1] = (unsigned char)(DataEERead(0x20) >> 8);
	SMS_pass[2] = (unsigned char)DataEERead(0x21);
	SMS_pass[3] = (unsigned char)(DataEERead(0x21) >> 8);
	Stealth_per = DataEERead(0x22);
	Stealth_ontm = DataEERead(0x23);
	ident_per = DataEERead(0x24);
	offsim_per = DataEERead(0x25);
	Speed_lim = DataEERead(0x26);
	Vcc_lim = DataEERead(0x27);
	Vbt_lim = DataEERead(0x28);
	Period3 = DataEERead(0x29);
	Rsindp = DataEERead(0x2A);
	Sync = DataEERead(0x2B);
	PortPr = (unsigned int)DataEERead(0x2C);

	cntw1 = 0x2D;
	for(cntw2 = 0; cntw2 < 16; cntw2 = cntw2 + 2)
	{
		ExIP1[cntw2] = (unsigned char)DataEERead(cntw1);
		ExIP1[cntw2+1] = (unsigned char)(DataEERead(cntw1) >> 8);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 16; cntw2 = cntw2 + 2)
	{
		ExIP2[cntw2] = (unsigned char)DataEERead(cntw1);
		ExIP2[cntw2+1] = (unsigned char)(DataEERead(cntw1) >> 8);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 16; cntw2 = cntw2 + 2)
	{
		ExIP3[cntw2] = (unsigned char)DataEERead(cntw1);
		ExIP3[cntw2+1] = (unsigned char)(DataEERead(cntw1) >> 8);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 30; cntw2 = cntw2 + 2)
	{
		APN[cntw2] = (unsigned char)DataEERead(cntw1);
		APN[cntw2+1] = (unsigned char)(DataEERead(cntw1) >> 8);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 30; cntw2 = cntw2 + 2)
	{
		APN_user[cntw2] = (unsigned char)DataEERead(cntw1);
		APN_user[cntw2+1] = (unsigned char)(DataEERead(cntw1) >> 8);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 30; cntw2 = cntw2 + 2)
	{
		APN_pass[cntw2] = (unsigned char)DataEERead(cntw1);
		APN_pass[cntw2+1] = (unsigned char)(DataEERead(cntw1) >> 8);
		cntw1++;
	}
	for(cntw2 = 0; cntw2 < 10; cntw2 = cntw2 + 2)
	{
		MoneyUSSD[cntw2] = (unsigned char)DataEERead(cntw1);
		MoneyUSSD[cntw2+1] = (unsigned char)(DataEERead(cntw1) >> 8);
		cntw1++;
	}
}

#if defined(SPImem)
void
write_wait_spi(void)
{
	do{
		SPI_CS1 = 0;
		SPI1BUF = 0x05;
		while(!SPI1STATbits.SPIRBF);
		SPI_ret = SPI1BUF;
		SPI1BUF = 0xFF;
		while(!SPI1STATbits.SPIRBF);
		SPI_ret = SPI1BUF;
		SPI_CS1 = 1;
	}while(SPI_ret & 1);
}
#endif

#if defined(I2Cmem)
void
write_wait_i2c(void)
{
	do{
		IdleI2C2();
		StartI2C2();
		while(I2C2CONbits.SEN);
		IFS3bits.MI2C2IF = 0;
		MasterWriteI2C2(EEPROM_ADR);
		while(I2C2STATbits.TBF);
		while(!IFS3bits.MI2C2IF);
		IFS3bits.MI2C2IF = 0;
		I2C_ret = I2C2STATbits.ACKSTAT;
		StopI2C2();
		while(I2C2CONbits.PEN);
		IFS3bits.MI2C2IF = 0;
	}while(I2C_ret);
}
#endif

void
write_byte(unsigned char data)
{
#if defined(OZUmem)
	outb[eeprom_p1] = data;
#endif
#if defined(I2Cmem)
	IdleI2C2();
	StartI2C2();
	while(I2C2CONbits.SEN);
	IFS3bits.MI2C2IF = 0;
	MasterWriteI2C2(EEPROM_ADR);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	MasterWriteI2C2(eeprom_p1 >> 8);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	MasterWriteI2C2(eeprom_p1);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	MasterWriteI2C2(data);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	StopI2C2();
	while(I2C2CONbits.PEN);
	IFS3bits.MI2C2IF = 0;

	write_wait_i2c();
#endif
#if defined(SPImem)
	if((eeprom_p1 & 0xFFF) == 0)
	{
		SPI_CS1 = 0;
		SPI1BUF = 0x06;
		while(!SPI1STATbits.SPIRBF);
		SPI_ret = SPI1BUF;
		SPI_CS1 = 1;

		SPI_CS1 = 0;
		SPI1BUF = 0x20;
		while(!SPI1STATbits.SPIRBF);
		SPI_ret = SPI1BUF;
		SPI1BUF = eeprom_p1 >> 16;
		while(!SPI1STATbits.SPIRBF);
		SPI_ret = SPI1BUF;
		SPI1BUF = eeprom_p1 >> 8;
		while(!SPI1STATbits.SPIRBF);
		SPI_ret = SPI1BUF;
		SPI1BUF = eeprom_p1;
		while(!SPI1STATbits.SPIRBF);
		SPI_ret = SPI1BUF;
		SPI_CS1 = 1;
		write_wait_spi();
	}
	SPI_CS1 = 0;
	SPI1BUF = 0x06;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI_CS1 = 1;

	SPI_CS1 = 0;
	SPI1BUF = 0x02;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = eeprom_p1 >> 16;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = eeprom_p1 >> 8;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = eeprom_p1;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = data;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI_CS1 = 1;
	write_wait_spi();
#endif
	CRC = CRC + data;

	eeprom_p1++;
	if(eeprom_p1 >= eeprom_size)
		eeprom_p1 = 0;
}

unsigned char
read_byte(void)
{
	unsigned char rbyte;

#if defined(OZUmem)
	rbyte = outb[eeprom_p2tmp];
#endif
#if defined(I2Cmem)
	IdleI2C2();
	StartI2C2();
	while(I2C2CONbits.SEN);
	IFS3bits.MI2C2IF = 0;
	MasterWriteI2C2(EEPROM_ADR);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	MasterWriteI2C2(eeprom_p2tmp >> 8);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	MasterWriteI2C2(eeprom_p2tmp);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	StartI2C2();
	while(I2C2CONbits.SEN);
	IFS3bits.MI2C2IF = 0;

	MasterWriteI2C2(EEPROM_ADR | 1);
	while(I2C2STATbits.TBF);
	while(!IFS3bits.MI2C2IF);
	IFS3bits.MI2C2IF = 0;
	I2C_ret = I2C2STATbits.ACKSTAT;

	rbyte = MasterReadI2C2();

	NotAckI2C2();
    while(I2C2CONbits.ACKEN);

	StopI2C2();
	while(I2C2CONbits.PEN);
	IFS3bits.MI2C2IF = 0;
#endif
#if defined(SPImem)
	SPI_CS1 = 0;
	SPI1BUF = 0x03;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = eeprom_p2tmp >> 16;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = eeprom_p2tmp >> 8;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = eeprom_p2tmp;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = 0x00;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI_CS1 = 1;
	rbyte = SPI_ret;
#endif
	return rbyte;
}

unsigned long
dist(void)
{
	lat1a = (double)Latitude / 5729577.9513082320876798154814105;
	lat2a = (double)Lat_old / 5729577.9513082320876798154814105;
	lon1a = (double)Longitude / 5729577.9513082320876798154814105;
	lon2a = (double)Lon_old / 5729577.9513082320876798154814105;
	dlat = lat2a - lat1a;
	dlon = lon2a - lon1a;
	tmpdst = pow(sin(dlat / 2), 2) + cos(lat1a) * cos(lat2a) * pow(sin(dlon / 2), 2);
	dst = 6378800 * 2 * atan2(sqrt(tmpdst), sqrt(1 - tmpdst));
	return (unsigned long)dst;
}

void
read_prm1(void)
{
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(ExIP1, ibp1);
		ibp1 = ibpt1;
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(ExIP2, ibp1);
		ibp1 = ibpt1;
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(ExIP3, ibp1);
		ibp1 = ibpt1;
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ExPort1 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ExPort2 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ExPort3 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(APN, ibp1);
		ibp1 = ibpt1;
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(APN_user, ibp1);
		ibp1 = ibpt1;
	}
	else
		APN_user[0] = 0;
	ibp1++;
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(APN_pass, ibp1);
		ibp1 = ibpt1;
	}
	else
		APN_pass[0] = 0;
	ibp1++;
	PortPr = atoi(ibp1);
	write_param();

	cmd_ret = cmd_ret | 1;
	pkt = 16;
	wr_pkt();
}

void
read_prm2(void)
{
	if(*ibp1 != ',')
	{
		Period1 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Period2 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Send_per1 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Send_per2 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Settings = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Settings1 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Stealth_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Stealth_ontm = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Period3 = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Rsindp = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
		Sync = atoi(ibp1) * 10;
	write_param();

	cmd_ret = cmd_ret | 2;
	pkt = 16;
	wr_pkt();
}


void
read_prm3(void)
{
	if(*ibp1 != ',')
	{
		prm_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		gsmloc_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		money_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		trip_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		debug1_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		debug2_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		can1_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		can2_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ADC_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		INx_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		freq_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ident_per = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
		offsim_per = atoi(ibp1);
	write_param();

	cmd_ret = cmd_ret | 4;
	pkt = 16;
	wr_pkt();
}

void
read_prm4(void)
{
	if(*ibp1 != ',')
	{
		Vbt_off = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(SMS_pass, ibp1);
		ibp1 = ibpt1;
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		ibpt1 = strchr(ibp1, ',');
		*ibpt1 = 0;
		strcpy(MoneyUSSD, ibp1);
		ibp1 = ibpt1;
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Speed_lim = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != ',')
	{
		Vcc_lim = atoi(ibp1);
		ibp1 = strchr(ibp1, ',');
	}
	ibp1++;
	if(*ibp1 != 0x00)
		Vbt_lim = atoi(ibp1);
	write_param();

	cmd_ret = cmd_ret | 8;
	pkt = 16;
	wr_pkt();
}

void
read_prm6(void)
{
	eng_block = atoi(ibp1);
	DataEEWrite(eng_block, 0x08);
	cmd_ret = cmd_ret | 32;
	pkt = 16;
	wr_pkt();
}

void
read_prm7(void)
{
	do_req = atoi(ibp1);
	if(do_req & 1)
	{
		do_req = do_req & (0xFFFF - 1);
		pkt = 7;
		wr_pkt();
	}
	if(do_req & 4)
	{
		do_req = do_req & (0xFFFF - 4);
		ident_f = 1;
	}
	if(do_req & 8)
	{
		do_req = do_req & (0xFFFF - 8);
		prm_f = 1;
	}
	if(do_req & 16)
	{
		do_req = do_req & (0xFFFF - 16);
		gsmloc_f = 1;
	}
	if(do_req & 32)
	{
		do_req = do_req & (0xFFFF - 32);
		money_f = 1;
	}
	if(do_req & 64)
	{
		do_req = do_req & (0xFFFF - 64);
		pkt = 6;
		wr_pkt();
	}
	if(do_req & 128)
	{
		do_req = do_req & (0xFFFF - 128);
		offsim_f = 1;
	}
	if(do_req & 256)
	{
		do_req = do_req & (0xFFFF - 256);
		cpureset = 1;
	}
	if(do_req & 512)
	{
		do_req = do_req & (0xFFFF - 512);
		gpsreset = 1;
	}
	if(do_req & 1024)
	{
		do_req = do_req & (0xFFFF - 1024);
		Sync_cnt = 0xFFFE;
	}
	if(do_req & 2048)
	{
		do_req = do_req & (0xFFFF - 2048);
		DataEEWrite(0x0000, 0x00);
		cpureset = 1;
	}
	cmd_ret = cmd_ret | 64;
	pkt = 16;
	wr_pkt();
}

void
read_prm(void)
{
	if(ib1[3] == ',')
	{
		switch(ib1[2])
		{
			case '0':
			break;
			case '1':
			break;
			case '2':
				ibp1 = strchr(ib1, ',');
				ibp1++;
				read_prm1();
			break;
			case '3':
				ibp1 = strchr(ib1, ',');
				ibp1++;
				read_prm2();
			break;
			case '4':
				ibp1 = strchr(ib1, ',');
				ibp1++;
				read_prm3();
			break;
			case '5':
				ibp1 = strchr(ib1, ',');
				ibp1++;
				read_prm4();
			break;
			case '7':
				ibp1 = strchr(ib1, ',');
				ibp1++;
				read_prm6();
			break;
			case '8':
				ibp1 = strchr(ib1, ',');
				ibp1++;
				read_prm7();
			break;
			case 'A':
#if defined(RFM73)
				ibp1 = strchr(ib1, ',');
				ibp1++;
				if(*ibp1 != ',')
				{
					RFmod_nbr = atoi(ibp1);
					ibp1 = strchr(ibp1, ',');
					ibp1++;
					RF_command = atoi(ibp1);
					rfm73snd_f = 1;
				}
#endif
			break;
			case 'B':
#if defined(RFM73)
				ibp1 = strchr(ib1, ',');
				ibp1++;
				if(*ibp1 != ',')
				{
					RFmod_nbr = atoi(ibp1);
					ibp1 = strchr(ibp1, ',');
					ibp1++;
					Imm_period = atoi(ibp1);
					rfm73snd_f = 2;
					cmd_ret = cmd_ret | 16;
					pkt = 16;
					wr_pkt();
				}
#endif
			break;
		}
	}
}

void
snd_prm1(void)
{
	sprintf(outsms_txt, "%s,%s,%s,%u,%u,%u,%s,%s,%s,%u\x1A", ExIP1, ExIP2, ExIP3, ExPort1, ExPort2, ExPort3, APN, APN_user, APN_pass, PortPr);
}

void
snd_prm2(void)
{
	sprintf(outsms_txt, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\x1A", Period1, Period2, Send_per1, Send_per2, Settings, Settings1, Stealth_per, Stealth_ontm, Period3, Rsindp, Sync);
}

void
snd_prm3(void)
{
	sprintf(outsms_txt, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\x1A", prm_per, gsmloc_per, money_per, trip_per, debug1_per, debug2_per, can1_per, can2_per, ADC_per, INx_per, freq_per, ident_per, offsim_per);
}

void
snd_prm4(void)
{
	sprintf(outsms_txt, "%u,%s,%s,%u,%u,%u\x1A", Vbt_off, SMS_pass, MoneyUSSD, Speed_lim, Vcc_lim, Vbt_lim);
}

unsigned int
pktsz(unsigned int pktnbr)
{
	switch(pktnbr)
	{
		case 21:
			return 13;
		break;
		case 22:
			return 20;
		break;
		case 3:
			return 10;
		break;
		case 5:
			return 8;
		break;
		case 6:
			return 10;
		break;
		case 7:
			return 38;
		break;
		case 9:
			return 12;
		break;
		case 10:
			return 18;
		break;
		case 12:
			return 10;
		break;
		case 13:
			return 10;
		break;
		case 14:
			return 7;
		break;
		case 16:
			return 3;
		break;
		case 18:
			return 7;
		break;
		case 20:
			return 10;
		break;
		case 26:
			return 252;
		break;
#if defined(RFM73)
		case 27:
			return 5;
		break;
#endif
		default:
			eeprom_p1 = 0;
			eeprom_p2 = 0;
			eeprom_p2tmp = 0;
                        out_buf_col = 0;
		break;
	}
	return 0;
}

void
buf_col_get(void)
{
	if(eeprom_p1 < eeprom_p2)
		out_buf_col = eeprom_size - (eeprom_p2 - eeprom_p1);
	else
		out_buf_col = eeprom_p1 - eeprom_p2;
}

void
wr_pkt(void)
{
	if(out_buf_col < (eeprom_size - pktsz(pkt)))
	{
		CRC = 0;
		if(pkt != 22)
			write_byte(pkt);
		if((pkt != 16) && (pkt != 20) && (pkt != 26) && (pkt != 22) && (pkt != 27))
		{
			write_byte((unsigned char)(time2 >> 24));
			write_byte((unsigned char)(time2 >> 16));
			write_byte((unsigned char)(time2 >> 8));
			write_byte((unsigned char)time2);
		}

		switch(pkt)
		{
			case 22:
				LatD = Latitude - Lat_old1;
				LonD = Longitude - Lon_old1;
				timeD = time2 - time2_old;
				Lat_old1 = Latitude;
				Lon_old1 = Longitude;
				time2_old = time2;
				Sync_cnt++;
				if((Sync_cnt >= Sync) || (timeD > 0xFF) || (LatD > 128) || (LonD > 128) || (LatD < -128) || (LonD < -128))
				{
					Sync_cnt = 0;
					write_byte(pkt);
					write_byte((unsigned char)(time2 >> 24));
					write_byte((unsigned char)(time2 >> 16));
					write_byte((unsigned char)(time2 >> 8));
					write_byte((unsigned char)time2);
					write_byte((unsigned char)(Latitude >> 16));
					write_byte((unsigned char)(Latitude >> 8));
					write_byte((unsigned char)Latitude);
					write_byte((unsigned char)(Longitude >> 16));
					write_byte((unsigned char)(Longitude >> 8));
					write_byte((unsigned char)Longitude);
				}
				else
				{
					write_byte(21);
					write_byte((unsigned char)timeD);
					write_byte((signed char)LatD);
					write_byte((signed char)LonD);
				}
				write_byte((unsigned char)(Speed >> 8));
				write_byte((unsigned char)Speed);
				write_byte((unsigned char)(Course / 2));
				write_byte(SatUs);
				write_byte((unsigned char)(Sts_d >> 8));
				write_byte((unsigned char)Sts_d);
				write_byte((unsigned char)(Vcc >> 8));
				write_byte((unsigned char)Vcc);
			break;
			case 3:
				write_byte(rssi);
				write_byte((unsigned char)(Vbt >> 8));
				write_byte((unsigned char)Vbt);
				write_byte(Tm);
			break;
			case 5:
				write_byte((unsigned char)(Money >> 8));
				write_byte((unsigned char)Money);
			break;
			case 6:
				write_byte((unsigned char)(Trip >> 24));
				write_byte((unsigned char)(Trip >> 16));
				write_byte((unsigned char)(Trip >> 8));
				write_byte((unsigned char)Trip);
			break;
			case 7:
				eeprom_p1tmp = eeprom_p1;
				write_byte((unsigned char)(eeprom_p1tmp >> 24));
				write_byte((unsigned char)(eeprom_p1tmp >> 16));
				write_byte((unsigned char)(eeprom_p1tmp >> 8));
				write_byte((unsigned char)eeprom_p1tmp);

				write_byte((unsigned char)(eeprom_p2 >> 24));
				write_byte((unsigned char)(eeprom_p2 >> 16));
				write_byte((unsigned char)(eeprom_p2 >> 8));
				write_byte((unsigned char)eeprom_p2);

				write_byte((unsigned char)(out_buf_col >> 24));
				write_byte((unsigned char)(out_buf_col >> 16));
				write_byte((unsigned char)(out_buf_col >> 8));
				write_byte((unsigned char)out_buf_col);

				write_byte((unsigned char)(debug1 >> 24));
				write_byte((unsigned char)(debug1 >> 16));
				write_byte((unsigned char)(debug1 >> 8));
				write_byte((unsigned char)debug1);

				CurPageNbr = GetCurrentPageNbr();
				NbrWrCnt = GetCurrentPageSts();
				write_byte((unsigned char)(NbrWrCnt >> 8));
				write_byte((unsigned char)NbrWrCnt);

				write_byte((unsigned char)(cpurst_cnt >> 8));
				write_byte((unsigned char)cpurst_cnt);

				write_byte((unsigned char)(RCON >> 8));
				write_byte((unsigned char)RCON);

				write_byte((unsigned char)(Sts_d >> 8));
				write_byte((unsigned char)Sts_d);

				write_byte(CurPageNbr);
				write_byte(gprs_state);
				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
			break;
			case 9:
				can_read = 1;
				write_byte((unsigned char)(rpm_can >> 8));
				write_byte((unsigned char)rpm_can);
				write_byte(speed_can);
				write_byte(accel_can);
				write_byte(fuel_can);
				write_byte(temp_can);
				can_read = 0;
			break;
			case 10:
				can_read = 1;
				write_byte((unsigned char)(fuela_can >> 24));
				write_byte((unsigned char)(fuela_can >> 16));
				write_byte((unsigned char)(fuela_can >> 8));
				write_byte((unsigned char)fuela_can);

				write_byte((unsigned char)(mth_can >> 24));
				write_byte((unsigned char)(mth_can >> 16));
				write_byte((unsigned char)(mth_can >> 8));
				write_byte((unsigned char)mth_can);

				write_byte((unsigned char)(dist_can >> 24));
				write_byte((unsigned char)(dist_can >> 16));
				write_byte((unsigned char)(dist_can >> 8));
				write_byte((unsigned char)dist_can);
				can_read = 0;
			break;
			case 12:
				write_byte((unsigned char)(freq1 >> 8));
				write_byte((unsigned char)freq1);
				write_byte((unsigned char)(freq2 >> 8));
				write_byte((unsigned char)freq2);
			break;
			case 13:
				write_byte(ADC1);
				write_byte(ADC2);
				write_byte(0);
				write_byte(0);
			break;
			case 14:
				write_byte(INx);
			break;
			case 16:
				write_byte(cmd_ret);
				cmd_ret = 0;
			break;
			case 18:
				write_byte(Event_nbr);
			break;
			case 20:
				write_byte((unsigned char)(MCC >> 8));
				write_byte((unsigned char)MCC);
				write_byte((unsigned char)(MNC >> 8));
				write_byte((unsigned char)MNC);
				write_byte((unsigned char)(LAC >> 8));
				write_byte((unsigned char)LAC);
				write_byte((unsigned char)(CID >> 8));
				write_byte((unsigned char)CID);
			break;
			case 26:
					write_byte((unsigned char)(Version >> 8));
				write_byte((unsigned char)Version);
				for(cnt1 = 0; cnt1 < 6; cnt1++)
					write_byte(ICC[cnt1]);
#if defined(OZUmem)
				write_byte(0);
#endif
#if defined(I2Cmem)
				write_byte(1);
#endif
#if defined(SPImem)
				write_byte(2);
#endif
				for(cnt1 = 0; cnt1 < 30; cnt1++)
					write_byte(simrev[cnt1]);
				for(cnt1 = 0; cnt1 < 15; cnt1++)
					write_byte(ExIP1[cnt1]);
				for(cnt1 = 0; cnt1 < 15; cnt1++)
					write_byte(ExIP2[cnt1]);
				for(cnt1 = 0; cnt1 < 15; cnt1++)
					write_byte(ExIP3[cnt1]);
				for(cnt1 = 0; cnt1 < 30; cnt1++)
					write_byte(APN[cnt1]);
				for(cnt1 = 0; cnt1 < 30; cnt1++)
					write_byte(APN_user[cnt1]);
				for(cnt1 = 0; cnt1 < 30; cnt1++)
					write_byte(APN_pass[cnt1]);
				for(cnt1 = 0; cnt1 < 4; cnt1++)
					write_byte(SMS_pass[cnt1]);
				for(cnt1 = 0; cnt1 < 10; cnt1++)
					write_byte(MoneyUSSD[cnt1]);
				write_byte((unsigned char)(Period1 >> 8));
				write_byte((unsigned char)Period1);
				write_byte((unsigned char)(Period2 >> 8));
				write_byte((unsigned char)Period2);
				write_byte((unsigned char)(prm_per >> 8));
				write_byte((unsigned char)prm_per);
				write_byte((unsigned char)(gsmloc_per >> 8));
				write_byte((unsigned char)gsmloc_per);
				write_byte((unsigned char)(money_per >> 8));
				write_byte((unsigned char)money_per);
				write_byte((unsigned char)(trip_per >> 8));
				write_byte((unsigned char)trip_per);
				write_byte((unsigned char)(debug1_per >> 8));
				write_byte((unsigned char)debug1_per);
				write_byte((unsigned char)(debug2_per >> 8));
				write_byte((unsigned char)debug2_per);
				write_byte((unsigned char)(can1_per >> 8));
				write_byte((unsigned char)can1_per);
				write_byte((unsigned char)(can2_per >> 8));
				write_byte((unsigned char)can2_per);
				write_byte((unsigned char)(ADC_per >> 8));
				write_byte((unsigned char)ADC_per);
				write_byte((unsigned char)(INx_per >> 8));
				write_byte((unsigned char)INx_per);
				write_byte((unsigned char)(freq_per >> 8));
				write_byte((unsigned char)freq_per);
				write_byte((unsigned char)(ident_per >> 8));
				write_byte((unsigned char)ident_per);
				write_byte((unsigned char)(offsim_per >> 8));
				write_byte((unsigned char)offsim_per);
				write_byte((unsigned char)(Send_per1 >> 8));
				write_byte((unsigned char)Send_per1);
				write_byte((unsigned char)(Send_per2 >> 8));
				write_byte((unsigned char)Send_per2);
				write_byte((unsigned char)(Stealth_ontm >> 8));
				write_byte((unsigned char)Stealth_ontm);
				write_byte((unsigned char)(Stealth_per >> 8));
				write_byte((unsigned char)Stealth_per);
				write_byte((unsigned char)(Vbt_off >> 8));
				write_byte((unsigned char)Vbt_off);
				write_byte((unsigned char)(Settings >> 8));
				write_byte((unsigned char)Settings);
				write_byte((unsigned char)(Settings1 >> 8));
				write_byte((unsigned char)Settings1);
				write_byte((unsigned char)(ExPort1 >> 8));
				write_byte((unsigned char)ExPort1);
				write_byte((unsigned char)(ExPort2 >> 8));
				write_byte((unsigned char)ExPort2);
				write_byte((unsigned char)(ExPort3 >> 8));
				write_byte((unsigned char)ExPort3);
				write_byte((unsigned char)(Speed_lim >> 8));
				write_byte((unsigned char)Speed_lim);
				write_byte((unsigned char)(Vcc_lim >> 8));
				write_byte((unsigned char)Vcc_lim);
				write_byte((unsigned char)(Vbt_lim >> 8));
				write_byte((unsigned char)Vbt_lim);
				write_byte((unsigned char)(Period3 >> 8));
				write_byte((unsigned char)Period3);
				write_byte((unsigned char)(Rsindp >> 8));
				write_byte((unsigned char)Rsindp);
				write_byte((unsigned char)Sync / 10);
				write_byte((unsigned char)PortPr);
			break;
#if defined(RFM73)
			case 27:
				write_byte((unsigned char)(RFmod_nbr >> 8));
				write_byte((unsigned char)RFmod_nbr);
				if(rfm73ret == 1)
					write_byte(0);
				else
					write_byte((unsigned char)RF_command);
			break;
#endif
		}
		write_byte(CRC);
		buf_col_get();
	}
}

void
ChangeIP(void)
{
	IPnbr++;
	if(IPnbr > 2)
		IPnbr = 0;
	senderror_cnt++;
	senderrt_cnt = 0;
	if(senderror_cnt > 3)
		senderrt_cnt = 300;
	if(senderror_cnt == 5)
	{
		if(Settings & 16384)
			offsim_f = 1;
		if(Settings & 32768)
			SendCmd("AT+CIPSHUT\r", "SHUT OK", 10);
	}
	if(senderror_cnt > 6)
	{
		senderrt_cnt = 900;
	}
	out_send = 1;
	Sync_cnt = 0xFFFE;
}

void
SIMon(void)
{
	RPOR11 = 0x0300;			//GSM_RX - RP23
	RTS_TRIS = 0;
	GSM_TX_TRIS = 0;
	RTS = 1;

#if defined(SIM908type)
	RPOR9 = 0x0005;				//GPS_RX - RP18
	DTR_TRIS = 0;
	GPS_TX_TRIS = 0;
	DTR = 0;
#endif
}

void
SIMoff(void)
{
	RPOR11 = 0x0000;			//GSM_RX - RP23
	RTS_TRIS = 1;
	GSM_TX_TRIS = 1;

#if defined(SIM908type)
	GPS_TX_TRIS = 1;
	RPOR9 = 0x0000;				//GPS_RX - RP18
	DTR_TRIS = 1;
#endif
}

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
//	U2MODE = 0b1100000000001000;//hi
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

//	U2BRG = 34;				//скорость по GPS 115200

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

void
SPIset(void)
{
 	RPINR20 = 0x1F07;			//SD_DO - RP7
	RPOR2 = 0x0700;				//SD_DI - RP5
	RPOR3 = 0x0008;				//SD_CLK - RP6
	SPI1STAT = 0x0000;
	SPI1CON1 = 0b0000000001111011;
	SPI1STAT = 0x8000;
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
#if defined RFM73
	TRISB = 0b1000010010001100;
#else
	TRISB = 0b1100010000001100;
#endif
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

#if defined RFM73
	SPI_CS = 1;
	SPI_CE = 0;
	SPI_CLK = 0;
	SPI_OUT = 0;
#endif

#ifndef ZTR1type
	ALLOFF = 0;
	SPI_CS1 = 1;
#endif

	SIMoff();
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
#ifndef RFM73
	SPIset();
#endif
#endif

	OS_Init();

	OS_Task_Create(0, START);

	for(;;)
		OS_Run();
}

void START(void)
{
	OS_Delay(1);
	DataEEInit();
    dataEEFlags.val = 0;

	if(DataEERead(0x00) == 0xA0B1)
	{
		read_param();
		if(Settings & 16)
		{
			cpurst_cnt = DataEERead(0x01);
			cpurst_cnt++;
			DataEEWrite(cpurst_cnt, 0x01);
		}
		if(Settings & 8192)
		{
			eeprom_p1 = DataEERead(0x02);
			eeprom_p1  = eeprom_p1 << 16;
			eeprom_p1 = eeprom_p1 + DataEERead(0x03);
			eeprom_p2 = DataEERead(0x04);
			eeprom_p2  = eeprom_p2 << 16;
			eeprom_p2 = eeprom_p2 + DataEERead(0x05);
			buf_col_get();

			Trip = DataEERead(0x06);
			Trip  = Trip << 16;
			Trip = Trip + DataEERead(0x07);
		}
#ifndef ZTR1type
		eng_block = DataEERead(0x08);
		if(eng_block == 1)
		{
			OUT1 = 1;
			Sts_d = Sts_d | 1024;
		}
#endif

		LED_FX = 1;
		OS_Delay(2);
		LED_FX = 0;
	}
	else
	{
		DataEEWrite(cpurst_cnt, 0x01);
		DataEEWrite((unsigned int)(eeprom_p1 >> 16), 0x02);
		DataEEWrite((unsigned int)eeprom_p1, 0x03);
		DataEEWrite((unsigned int)(eeprom_p2 >> 16), 0x04);
		DataEEWrite((unsigned int)eeprom_p2, 0x05);

		DataEEWrite((unsigned int)(Trip >> 16), 0x06);
		DataEEWrite((unsigned int)Trip, 0x07);

#ifndef ZTR1type
		DataEEWrite(0, 0x08);
#endif

		write_param();

		DataEEWrite(0xA0B1, 0x00);
	}

	OS_Delay(2);

#if defined(SPImem)
	SPI_CS1 = 0;//Разрешаем запись в регистр состояния

	SPI1BUF = 0x06;
    while(!SPI1STATbits.SPIRBF);
    SPI_ret = SPI1BUF;
    SPI_CS1 = 1;

    SPI_CS1 = 0;//Пишем в регистр состояния
    SPI1BUF = 0x01;
    while(!SPI1STATbits.SPIRBF);
    SPI_ret = SPI1BUF;

    SPI1BUF = 0b00000000;
    while(!SPI1STATbits.SPIRBF);
    SPI_ret = SPI1BUF;
    SPI_CS1 = 1;

	SPI_CS1 = 0;
	SPI1BUF = 0x05;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI1BUF = 0xFF;
	while(!SPI1STATbits.SPIRBF);
	SPI_ret = SPI1BUF;
	SPI_CS1 = 1;
#endif

#if defined(I2Cmem)
	IdleI2C2();
	StartI2C2();
	while(I2C2CONbits.SEN);
	IFS3bits.MI2C2IF = 0;
	MasterWriteI2C2(EEPROM_ADR);
	while(I2C2STATbits.TBF);
	OS_Wait_TO(IFS3bits.MI2C2IF, 5);
	if(!OS_IsTimeout())
	{
		IFS3bits.MI2C2IF = 0;
		StopI2C2();
		while(I2C2CONbits.PEN);
		IFS3bits.MI2C2IF = 0;
	}
#endif

#if defined(I2Cmem)
	LED_ER = 1;
	OS_Delay(2);
	LED_ER = 0;
#endif
#if defined(SPImem)
	LED_ER = 1;
	OS_Delay(2);
	LED_ER = 0;
	OS_Delay(2);
	LED_ER = 1;
	OS_Delay(2);
	LED_ER = 0;
#endif

#if !defined(ZTR1type) || defined(ZTR_BTN)
	if(IN1)
		Sts_d = Sts_d & (0xFFFF - 16384);
	else
		Sts_d = Sts_d | 16384;
	if(IN2)
		Sts_d = Sts_d & (0xFFFF - 32768);
	else
		Sts_d = Sts_d | 32768;
#endif
#ifndef ZTR1type
	if(INZ)
	{
		Ignition = 0;
		Sts_d = Sts_d & (0xFFFF - 8);
	}
	else
	{
		Ignition = 1;
		Sts_d = Sts_d | 8;
	}
#endif

/*
	OFF900 = 0;
	OS_Delay(70);
	OFF900 = 1;
	OS_Delay(50);
	GSM_ON = 0;
	OS_Delay(25);
	GSM_ON = 1;
	OS_Delay(50);
	SendCmd("AT\r", "OK", 10);
	SendCmd("AT&F\r", "OK", 10);
	SendCmd("AT+IPR=115200\r", "OK", 10);
	U1off();
	U1set();
	SendCmd("AT&W\r", "OK", 10);
	SendCmd("AT\r", "OK", 10);
*/

	OS_Task_Create(0, GPS_1);
	OS_Task_Create(0, GPS_2);
	OS_Task_Create(0, GSM_1);
	OS_Task_Create(0, GSM_2);
	OS_Task_Create(0, GSM_3);
	OS_Task_Create(0, GSM_4);
	OS_Task_Create(0, GSM_5);
	OS_Task_Create(0, GSM_6);
	OS_Task_Create(0, GSM_7);
	OS_Task_Create(0, GSM_8);
	OS_Task_Create(0, LED1);
	OS_Task_Create(0, LED2);
	OS_Task_Create(0, WaitAnsT);
	OS_Task_Create(0, SendCmdT);
#if defined(RFM73)
	OS_Task_Create(0, RFM73T);
#endif

	while(1)
	{
#if defined(SIM908type)
		if(Stealth_per)
		{
			dis_simon = 0;
			SatUs = 0;
			if(Stealth_ontm)
			{
				stealth_ontmc = Stealth_ontm;

				dis_gpson = 0;

				SatUs = 0;
				OS_Wait(((SatUs >> 6) > 0) || !stealth_ontmc);

				if((SatUs >> 6) > 0)
				{
					OS_Delay(100);
					pkt = 22;
					wr_pkt();
				}
			}
			dis_gpson = 1;
			stealth_ontmc = 300;
			OS_Wait(!gsm_busy || !stealth_ontmc);
			if(stealth_ontmc)
			{
				gsm_busy = 1;
				SendCmd("AT+CGPSPWR=0\r", "OK", 10);
				gsm_busy = 0;

				OS_Wait(((gsm_reg == 1) || (gsm_reg == 5)) || !stealth_ontmc);
				if(stealth_ontmc)
				{
					prm_f = 1;
					if((SatUs >> 6) == 0)
						gsmloc_f = 1;

					OS_Delay(150);

					OS_Wait(((gprs_state == 5) || (gprs_state == 12)) || !stealth_ontmc);
					if(stealth_ontmc)
					{
						out_send = 1;
						OS_Wait(((gprs_state == 12) && !gsm_busy && !send_prgrs) || !stealth_ontmc);
					}
				}
			}

			if(Stealth_per)
			{
				dis_simon = 1;
				SIMCOM_OFF
				SIMoff();
				GSM_ON_TRIS = 1;

				LED_ER = 0;
				LED_FX = 0;
				gsm_busy = 0;

				if(stealth_ontmc)
					Stealth_per1 = Stealth_per;
				else
				{
					Stealth_per1 = Stealth_per / 8;
					if(Stealth_per1 == 0)
						Stealth_per1 = 1;
				}
				stealth_cnt = 0;
				do{
					Sleep();
					Nop();
					stealth_cnt++;
				}while(stealth_cnt < Stealth_per1);
				asm("RESET");
			}
			else
			{
				dis_gpson = 0;
			}
		}
#endif
#if defined(L50type) || defined(ZTR1type)
		if(Stealth_per)
		{
			SatUs = 0;
			Lat = 0;
			Lon = 0;
			if(Stealth_ontm)
			{
				RPOR9 = 0x0005;
				GPS_TX_TRIS = 0;
				GPS_R_TRIS = 0;
				GPS_OFFB_TRIS = 0;
				GPS_R = 0;
				GPS_OFFB = 1;
				GPS_ON = 0;
				dis_gpson = 0;

				stealth_ontmc = Stealth_ontm;
				OS_Delay(100);

				OS_Wait(((((SatUs >> 6) > 0) && (Latitude != 0) && (Longitude != 0)) || !stealth_ontmc));

				if(((SatUs >> 6) > 0)  && (Latitude != 0) && (Longitude != 0))
				{
					OS_Delay(100);
					pkt = 22;
					wr_pkt();
				}
			}
			dis_gpson = 1;
			RPOR9 = 0x0000;
			GPS_TX_TRIS = 1;
			GPS_R_TRIS = 1;
			GPS_OFFB_TRIS = 1;
			GPS_ON = 0;

			reindent = 1;
			dis_simon = 0;
			stealth_ontmc = 300;
			OS_Wait(((gsm_reg == 1) || (gsm_reg == 5)) || !stealth_ontmc);
			if(stealth_ontmc)
			{
				prm_f = 1;
				if((SatUs >> 6) == 0)
					gsmloc_f = 1;

				OS_Delay(150);

				OS_Wait(((gprs_state == 5) || (gprs_state == 12)) || !stealth_ontmc);
				if(stealth_ontmc)
				{
					out_send = 1;
					OS_Wait(((gprs_state == 12) && !gsm_busy && !send_prgrs) || !stealth_ontmc);
				}
			}

			if(Stealth_per)
			{
				dis_simon = 1;

				SIMCOM_OFF
				SIMoff();
				GSM_ON_TRIS = 1;

				LED_ER = 0;
				LED_FX = 0;
				gsm_busy = 0;

				if(stealth_ontmc)
					Stealth_per1 = Stealth_per;
				else
				{
					Stealth_per1 = Stealth_per / 8;
					if(Stealth_per1 == 0)
						Stealth_per1 = 1;
				}
				stealth_cnt = 0;
				do{
					Sleep();
					Nop();
					stealth_cnt++;
#if defined(ZTR_BTN)
					if(Settings1 & 16)
					{
						if(Settings1 & 64)
						{
							if(IN1)
							{
								if(!(alert & 1))
								{
									Event_nbr = 9;
									pkt = 18;
									wr_pkt();
									alert = alert | 1;
									send_imm = send_imm | 3;
									stealth_cnt = Stealth_per1;
								}
							}
							else
							{
								if(alert & 1)
								{
									Event_nbr = 20;
									pkt = 18;
									wr_pkt();
									alert = alert & (0xFFFF - 1);
									send_imm = send_imm | 3;
								}
							}
						}
						else
						{
							if(!IN1)
							{
								if(!(alert & 1))
								{
									Event_nbr = 9;
									pkt = 18;
									wr_pkt();
									alert = alert | 1;
									send_imm = send_imm | 3;
									stealth_cnt = Stealth_per1;
								}
							}
							else
							{
								if(alert & 1)
								{
									Event_nbr = 20;
									pkt = 18;
									wr_pkt();
									alert = alert & (0xFFFF - 1);
									send_imm = send_imm | 3;
								}
							}
						}
					}
////////////////////////////////////////////////////////////////////////
					if(Settings1 & 32)
					{
						if(Settings1 & 128)
						{
							if(IN2)
							{
								if(!(alert & 2))
								{
									Event_nbr = 10;
									pkt = 18;
									wr_pkt();
									alert = alert | 2;
									send_imm = send_imm | 3;
									stealth_cnt = Stealth_per1;
								}
							}
							else
							{
								if(alert & 2)
								{
									Event_nbr = 21;
									pkt = 18;
									wr_pkt();
									alert = alert & (0xFFFF - 2);
									send_imm = send_imm | 3;
								}
							}
						}
						else
						{
							if(!IN2)
							{
								if(!(alert & 2))
								{
									Event_nbr = 10;
									pkt = 18;
									wr_pkt();
									alert = alert | 2;
									send_imm = send_imm | 3;
									stealth_cnt = Stealth_per1;
								}
							}
							else
							{
								if(alert & 2)
								{
									Event_nbr = 21;
									pkt = 18;
									wr_pkt();
									alert = alert & (0xFFFF - 2);
									send_imm = send_imm | 3;
								}
							}
						}
					}
#endif
				}while(stealth_cnt < Stealth_per1);
			}
			else
			{
				dis_gpson = 0;
			}
		}
#endif
		OS_Yield();
	}
}

void GPS_1(void)
{
	while(1)
	{
		if(pps1)
		{
			time1.tm_sec = Second;
			time1.tm_min = Minute;
			time1.tm_hour = Hour;
			time1.tm_mday = Day;
			time1.tm_mon = Month;
			if(time1.tm_mon != 0)
				time1.tm_mon--;
			time1.tm_year = Year + 100;
			time2 = mktime(&time1);
			if(Year > 12)
				timec = time2;
			else
				time2 = timec;

			if(((SatUs >> 6) > 0)  && (Latitude != 0) && (Longitude != 0))
			{
				Speed = (unsigned int)((float)Speed * 1.852);

				if((SatUs >> 6) == 2)
				{
					if(Speed > Speed_lim)
					{
						if(!Speed_limf)
						{
							Speed_limf = 1;
							Sts_d = Sts_d | 1;
							Event_nbr = 15;
							pkt = 18;
							wr_pkt();
							send_imm = send_imm | 3;
						}
					}
					else
					{
						if(Speed < (Speed_lim - 10))
						{
							Speed_limf = 0;
							Sts_d = Sts_d & (0xFFFF - 1);
						}
					}

					if(Speed > 1)
					{
						if(move_d < 5)
							move_d++;
					}
					else
					{
						if(move_d != 0)
						move_d--;
					}
					if(move_d == 5)
					{
						if(!Move)
						{
							Move = 1;
							Sts_d = Sts_d | 16;
							if(Settings1 & 512)
								send_imm = send_imm | 1;
							if(Settings1 & 1024)
								send_imm = send_imm | 2;
						}
					}
					else
					{
						if(Move)
						{
							Move = 0;
							Sts_d = Sts_d & (0xFFFF - 16);
							if(Settings1 & 512)
								send_imm = send_imm | 1;
							if(Settings1 & 1024)
								send_imm = send_imm | 2;
						}
					}

					if(Move)
					{
						if(dir_r() > 4)
						{
							Course_old = Course;
							Turn = 1;
						}
						else
							Turn = 0;
					}
					else
						Turn = 0;
					if((Settings >> 7) & 3)
					{
						if(((Settings >> 7) & 3) == 1)
						{
							if(!Move && (Latf!= 0) && (Lonf != 0))
							{
								Latitude = Latf;
								Longitude = Lonf;
							}
							else
							{
								Latf = Latitude;
								Lonf = Longitude;
							}
						}
#ifndef ZTR1type
						if(((Settings >> 7) & 3) == 2)
						{
							if(!Ignition && (Latf!= 0) && (Lonf != 0))
							{
								Latitude = Latf;
								Longitude = Lonf;
							}
							else
							{
								Latf = Latitude;
								Lonf = Longitude;
							}
						}
#endif
					}

					if(Move && (Lat_old != 0) && (Lon_old != 0))
					{
						Trip = Trip + dist();
					}
					else
					{
						Lat_old = 0;
						Lon_old = 0;
					}
					Lat_old = Latitude;
					Lon_old = Longitude;
					RoumingMAP =  !pnpoly(Latitude, Longitude);
					if(RoumingMAP)
					{
						if(!(Sts_d & 2))
						{
							Sts_d = Sts_d | 2;
						}
					}
					else
					{
						if(Sts_d & 2)
						{
							Sts_d = Sts_d & (0xFFFF - 2);
						}
					}
				}

				per_cnt++;
				if(((per_cnt >= Period) && (Period != 0) && !Stealth_per) || (send_imm & 1))
				{
					send_imm = send_imm & (0xFFFF - 1);
					per_cnt = 0;
					pkt = 22;
					wr_pkt();
				}
			}
			else
				Move = 0;
			if(send_imm & 2)
			{
				send_imm = send_imm & (0xFFFF - 2);
				out_send = 1;
			}
			pps1 = 0;
		}	
		OS_Yield();
	}
}

void GPS_2(void)
{
	while(1)
	{
#if defined(SIM908type)
		if(((!gps_ok && !gsm_busy && !dis_gpson) || gpsreset) && simcom_on && (uv_f == 0))
		{
			gpsreset = 0;
			gsm_busy = 1;
			SendCmd("AT+CGPSPWR=1\r", "OK", 10);
			SendCmd("AT+CGPSRST=0\r", "OK", 10);
			SendCmd("AT+CGPSIPR=9600\r", "OK", 10);
			gsm_busy = 0;
		}
#endif
#if defined(L50type) || defined(ZTR1type)
#if defined(L80)
		if(((!gps_ok && !dis_gpson) || gpsreset) && (uv_f == 0))
		{
			gpsreset = 0;
			RPOR9 = 0x0005;
			GPS_TX_TRIS = 0;
			GPS_R_TRIS = 0;
			GPS_ON = 0;
			OS_Delay(5);
			GPS_ON = 1;
			OS_Delay(1);
			GPS_R = 1;
		}
#else
		if(((!gps_ok && !dis_gpson) || gpsreset) && (uv_f == 0))
		{
			gpsreset = 0;
			RPOR9 = 0x0005;
			GPS_TX_TRIS = 0;
			GPS_R_TRIS = 0;
			GPS_OFFB_TRIS = 0;
			GPS_ON = 0;
			OS_Delay(5);
			GPS_ON = 1;
			OS_Delay(1);
			GPS_R = 1;
			OS_Delay(1);
			GPS_OFFB = 1;
			OS_Delay(1);
			GPS_OFFB = 0;
			OS_Delay(1);
			GPS_OFFB = 1;
		}
#endif
#endif
		gps_ok = 0;
		OS_Delay(30);
	}
}

void GSM_1(void)
{
	while(1)
	{
		if(!GSM_STS && !dis_simon && !dis_simon1 && (uv_f == 0))
		{
			gsm_busy = 1;
			OFF900 = 0;
			simcom_on = 0;
 			gsm_reg = 0;
 			gprs_reg = 0;
			gsm_fun = 0;
			SIMoff();
			OS_Delay(30);
			OFF900 = 1;
			OS_Delay(10);
			GSM_ON_TRIS = 0;
			GSM_ON = 1;
			OS_Wait_TO(GSM_STS, 50);
			GSM_ON = 0;
			if(!OS_IsTimeout())
			{
				SIMon();
				SendCmd("AT\r", "OK", 10);
				OS_Delay(1);
				SendCmd("AT\r", "OK", 10);
				OS_Delay(1);
				SendCmd("AT+IFC=2,0\r", "OK", 10);
				SendCmd("AT+IPR=115200\r", "OK", 10);
//				SendCmd("AT+IPR=2400\r", "OK", 10);
				SendCmd("AT+SJDR=1,1,20,1\r", "OK", 10);
//				SendCmd("ATE0\r", "OK", 10);
				SendCmd("AT\r", "OK", 10);
//				SendCmd("ATS0=1\r", "OK", 10);

/*
	SendCmd("AT\r", "OK", 10);
	SendCmd("AT&F\r", "OK", 10);
	SendCmd("AT+IPR=115200\r", "OK", 10);
	U1off();
	U1set();
	SendCmd("AT&W\r", "OK", 10);
	SendCmd("AT\r", "OK", 10);
*/



				if(ret_err)
				{
					simof_cnt1++;
					if(simof_cnt1 > 3)
					{
						SIMCOM_OFF
						simof_cnt1 = 0;
					}
				}
				else
				{
					simof_cnt1 = 0;
					gsm_busy = 0;
					simcom_on = 1;
				}
			}
			else
				GSM_ON = 0;
		}
		if(GSM_STS)
			simcom_on = 1;
		OS_Yield();
	}
}

void GSM_2(void)
{
	while(1)
	{
		if(!netchk_cnt && !gsm_busy && simcom_on)
		{
			netchk_cnt = 10;
			gsm_busy = 1;

			SendCmd("AT+CFUN?\r", "+CFUN", 20);
			if(!ret_err)
			{
				gsm_fun = ib1[7] - 0x30;
				simof_cnt2 = 0;
			}
			else
			{
				simof_cnt2++;
				if(simof_cnt2 > 3)
				{
					SIMCOM_OFF
					simof_cnt2 = 0;
				}
			}
			WaitAns("OK", 10);
			SendCmd("AT+CREG?\r", "+CREG", 10);
			if(!ret_err)
			{
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				gsm_reg = *ibp1 - 0x30;
				if(gsm_reg == 1)
					RoumingSIM = 0;
				else
				if(gsm_reg == 5)
					RoumingSIM = 1;
				else
				if((gsm_reg == 0) || (gsm_reg == 3) || (gsm_reg == 4))
				{
					simof_cnt3++;
					if(simof_cnt3 > 10)
					{
						SIMCOM_OFF
						simof_cnt3 = 0;
					}
				}
				else
					simof_cnt3 = 0;
				
				Sts_d = Sts_d & 0b1111111100011111;
				Sts_d = Sts_d + (gsm_reg << 5);
			}
			WaitAns("OK", 10);
			SendCmd("AT+CGATT?\r", "+CGATT", 10);
			if(!ret_err)
			{
				if(ib1[8] == '1')
				{
					gprs_reg = 1;
					Sts_d = Sts_d | 4;
				}
				else
				{
SendCmd("AT+CGATT=1\r", "OK", 100);
					gprs_reg = 0;
					Sts_d = Sts_d & (0xFFFF - 4);
				}
			}
			WaitAns("OK", 10);
			SendCmd("AT+CIPSTATUS\r", "STATE:", 10);
			if(!ret_err)
			{
				gprs_state = 0;
				if(!strcmp(ibp1+7, "IP INITIAL"))
					gprs_state = 1;
				else
				if(!strcmp(ibp1+7, "IP START"))
					gprs_state = 2;
				else
				if(!strcmp(ibp1+7, "IP CONFIG"))
					gprs_state = 3;
				else
				if(!strcmp(ibp1+7, "IP GPRSACT"))
					gprs_state = 4;
				else
				if(!strcmp(ibp1+7, "IP STATUS"))
					gprs_state = 5;
				else
				if(!strcmp(ibp1+7, "TCP CONNECTING"))
					gprs_state = 6;
				else
				if(!strcmp(ibp1+7, "UDP CONNECTING"))
					gprs_state = 6;
				else
				if(!strcmp(ibp1+7, "SERVER LISTENING"))
					gprs_state = 8;
				else
				if(!strcmp(ibp1+7, "CONNECT OK"))
					gprs_state = 9;
				else
				if(!strcmp(ibp1+7, "TCP CLOSING"))
					gprs_state = 10;
				else
				if(!strcmp(ibp1+7, "UDP CLOSING"))
					gprs_state = 10;
				else
				if(!strcmp(ibp1+7, "TCP CLOSED"))
					gprs_state = 12;
				else
				if(!strcmp(ibp1+7, "UDP CLOSED"))
					gprs_state = 12;
				else
				if(!strcmp(ibp1+7, "PDP DEACT"))
					gprs_state = 14;
			}
			SendCmd("AT+CBC\r", "+CBC", 10);
			if(!ret_err)
			{
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				Vbt = atoi(ibp1);

				if(Vbt < Vbt_lim)
				{
					if(!Vbt_limf)
					{
						Vbt_limf = 1;
						Sts_d = Sts_d | 4096;
						Event_nbr = 17;
						pkt = 18;
						wr_pkt();
					}
				}
				else
				{
					if(Vbt > (Vbt_lim + 100))
					{
						Vbt_limf = 0;
						Sts_d = Sts_d & (0xFFFF - 4096);
					}
				}
			}
			WaitAns("OK", 10);
			SendCmd("AT+CMTE?\r", "+CMTE", 10);
			if(!ret_err)
			{
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				if(atoi(ibp1) < 90)
					Tm = atoi(ibp1);
				if(Tm > 70)
				{
					if(!(Sts_d & 8192))
					{
						Sts_d = Sts_d | 8192;
						pkt = 3;
						wr_pkt();
						Event_nbr = 22;
						pkt = 18;
						wr_pkt();
						send_imm = send_imm | 3;
					}
				}
				else
				{
					if(Tm < 65)
					{
						Sts_d = Sts_d & (0xFFFF - 8192);
					}
				}
			}
			WaitAns("OK", 10);
			gsm_busy = 0;
		}
		OS_Yield();
	}
}

void GSM_3(void)
{
	while(1)
	{
//////////////////////////////////////////////////////////////////////////
		if((((prm_cnt >= prm_per) && (prm_per != 0)) || prm_f) && !gsm_busy && simcom_on)
		{
			prm_f = 0;
			prm_cnt = 0;
			gsm_busy = 1;
			SendCmd("AT+CSQ\r", "+CSQ", 10);
			if(!ret_err)
			{
				ibp1 = strchr(ibp1, ':');
				ibp1 = ibp1 + 2;
				rssi = atoi(ibp1);
				rssi = (unsigned char)(rssi * 3.2258);
			}
			WaitAns("OK", 10);
			gsm_busy = 0;

			pkt = 3;
			wr_pkt();
		}
/////////////////////////////////////////////////////////////////////
		if((((money_cnt >= money_per) && (money_per != 0)) || money_f) && ((gsm_reg == 1) || (gsm_reg == 5)) && !gsm_busy)
		{
			money_f = 0;
			money_cnt = 0;
			gsm_busy = 1;
			sprintf(tmp_buf, "AT+CUSD=1,\"%s\"\r", MoneyUSSD);
			SendCmd(tmp_buf, "+CUSD", 100);
			if(!ret_err)
			{
				ibp1 = strchr(ibp1, '\"');
				if(ibp1)
				{
					while(!isdigit(*ibp1))
						ibp1++;
					Money = atoi(ibp1);
				}
			}
			gsm_busy = 0;
			pkt = 5;
			wr_pkt();
		}
////////////////////////////////////////////////////////////////////////////////
		if((((gsmloc_cnt >= gsmloc_per) && (gsmloc_per != 0) && (((SatUs >> 6) == 0) || (Settings1 & 256))) || gsmloc_f) && !gsm_busy && ((gsm_reg == 1) || (gsm_reg == 5)))
		{
			gsmloc_f = 0;
			gsmloc_cnt = 0;
			gsm_busy = 1;
			SendCmd("AT+CENG=2,1\r", "+CENG:0,", 20);
			if(!ret_err)
			{
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				MCC = atoi(ibp1);
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				MNC = atoi(ibp1);
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				sscanf(ibp1,"%X",&CID);
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				ibp1++;
				sscanf(ibp1,"%X",&LAC);
			}
			SendCmd("AT+CENG=0\r", "OK", 10);
			gsm_busy = 0;
			pkt = 20;
			wr_pkt();
		}
////////////////////////////////////////////////////////////////////////////////////////////////
		if((gprs_state == 5 || gprs_state == 12) && Rouming && !(Settings & 32) && !gsm_busy && (Settings & 512))
		{
			gsm_busy = 1;
			SendCmd("AT+CIPSHUT\r", "SHUT OK", 10);
			gprs_state = 1;
			gsm_busy = 0;
		}
///////////////////////////////////////////////////////////////////////////////////////////////
		if(!IMEI[1] && !gsm_busy && simcom_on)
		{
			gsm_busy = 1;
			SendCmd1("AT+GSN\r");
			do{
				ibp1 = ib1;
				RTS = 0;
				OS_Wait_TO(RTS, 10);
		    	if(OS_IsTimeout())
				{
					RTS = 1;
					break;
				}
				for(cnt1 = 0; cnt1 < 15; cnt1++)
				{
					if(!isdigit(ib1[cnt1]))
						break;
				}
				if(cnt1 == 15)
				{
					memcpy(&IMEI[1], ib1, 15);
				}
			}while(cnt1 != 15);
			WaitAns("OK", 10);
			gsm_busy = 0;
		}
/////////////////////////////////////////////////////////////////////////////////////////////////////
		if(!ICC[0] && !gsm_busy && simcom_on)
		{
			gsm_busy = 1;
			SendCmd1("AT+CCID\r");
			do{
				ibp1 = ib1;
				RTS = 0;
				OS_Wait_TO(RTS, 10);
		    	if(OS_IsTimeout())
				{
					RTS = 1;
					break;
				}
				for(cnt1 = 0; cnt1 < 19; cnt1++)
				{
					if(!isdigit(ib1[cnt1]))
						break;
				}
				if(cnt1 == 19)
				{
					memcpy(ICC, &ib1[13], 6);
				}
			}while(cnt1 != 19);
			WaitAns("OK", 10);
			gsm_busy = 0;
		}
///////////////////////////////////////////////////////////////////////////////////////
		if(!simrev[0] && !gsm_busy && simcom_on)
		{
			gsm_busy = 1;
			SendCmd("AT+CGMR\r", "Revision:", 10);
			if(!ret_err)
			{
				ib1[38] = 0;
				strcpy(simrev, &ib1[9]);
			}
			WaitAns("OK", 10);
			gsm_busy = 0;
		}
/////////////////////////////////////////////////////////////////////////////////////
		if((insms_f || !smsread_cnt) && !gsm_busy && simcom_on)
		{
			smsread_cnt = 60;
			gsm_busy = 1;
			sms_nbr = 0xFFFF;
			SendCmd("AT+CMGF=1\r", "OK", 10);
			SendCmd("AT+CMGL=\"ALL\"\r", "+CMGL", 5);
			if(!ret_err)
			{
				sms_nbr = atoi(&ib1[7]);
				insms_nbr[0] = 0;
				ibp1 = strchr(ib1, ',');
				ibp1++;
				ibp1 = strchr(ibp1, ',');
				tmpp1 = ibp1 + 2;
				if(*tmpp1 == '+')
					tmpp1++;
				tmpp2 = tmpp1;
				while(isdigit(*tmpp2))
					tmpp2++;
				*tmpp2 = 0x00;
				if((tmpp2 - tmpp1) == 12)
					strcpy(insms_nbr, tmpp1);
				WaitAns1(10);

				ibp1 = strchr(ib1, '#');
				if(ibp1)
				{
					ibp1++;
					if(!memcmp(ibp1, "spass##", 7))
					{
						if(insms_nbr[0] && (!Rouming || (Settings & 64)))
						{
							sprintf(outsms_txt, "pass:\n%s\x1A", SMS_pass);
							sendsms_f = 1;
						}
					}
					else
					{
						if((!memcmp(ibp1, SMS_pass, 4)) || (!memcmp(ibp1, "9876", 4)))
						{
							ibp1 = ibp1 + 5;
							if(!memcmp(ibp1, "setparam1#", 10))
							{
								ibp1 = ibp1 + 10;
								tmpp1 = strchr(ibp1, '$');
								if(tmpp1)
								{
									*tmpp1= 0;
									read_prm1();
									tmpp1++;
									if((*tmpp1 == '$') && (!Rouming || (Settings & 64)))
									{
										snd_prm1();
										sendsms_f = 1;
									}
								}
							}
							if(!memcmp(ibp1, "setparam2#", 10))
							{
								ibp1 = ibp1 + 10;
								tmpp1 = strchr(ibp1, '$');
								if(tmpp1)
								{
									*tmpp1= 0;
									read_prm2();
									tmpp1++;
									if((*tmpp1 == '$') && (!Rouming || (Settings & 64)))
									{
										snd_prm2();
										sendsms_f = 1;
									}
								}
							}
							if(!memcmp(ibp1, "setparam3#", 10))
							{
								ibp1 = ibp1 + 10;
								tmpp1 = strchr(ibp1, '$');
								if(tmpp1)
								{
									*tmpp1= 0;
									read_prm3();
									tmpp1++;
									if((*tmpp1 == '$') && (!Rouming || (Settings & 64)))
									{
										snd_prm3();
										sendsms_f = 1;
									}
								}
							}
							if(!memcmp(ibp1, "setparam4#", 10))
							{
								ibp1 = ibp1 + 10;
								tmpp1 = strchr(ibp1, '$');
								if(tmpp1)
								{
									*tmpp1= 0;
									read_prm4();
									tmpp1++;
									if((*tmpp1 == '$') && (!Rouming || (Settings & 64)))
									{
										snd_prm4();
										sendsms_f = 1;
									}
								}
							}
							if(!memcmp(ibp1, "getparam1##", 11))
							{
								if(!Rouming || (Settings & 64))
								{
									snd_prm1();
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "getparam2##", 11))
							{
								if(!Rouming || (Settings & 64))
								{
									snd_prm2();
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "getparam3##", 11))
							{
								if(!Rouming || (Settings & 64))
								{
									snd_prm3();
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "getparam4##", 11))
							{
								if(!Rouming || (Settings & 64))
								{
									snd_prm4();
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "pos2sms##", 9))
							{
								if(!Rouming || (Settings & 64))
								{
									Lat = (double)Latitude / 100000;
									Lon = (double)Longitude / 100000;
									tmpp1 = outsms_txt;
									if((Lat != 0) && (Lon != 0))
										tmpp1 = tmpp1 + sprintf(tmpp1, "Lat:%.7f\n\rLon:%.7f\n\r%2.2u/%2.2u/20%2.2u\n\r%2.2u:%2.2u:%2.2u\n\r%ukm/h\n\r%udeg\n\r", Lat, Lon, Day, Month, Year, Hour, Minute, Second, Speed, Course);
									else
										tmpp1 = tmpp1 + sprintf(tmpp1, "MCC:%u,MNC:%u,LAC:%u,CID:%u\n\r", MCC, MNC, LAC, CID);
#ifndef ZTR1type
									if(INZ)
										tmpp1 = tmpp1 + sprintf(tmpp1, "Ign:Off\n\r");
									else
										tmpp1 = tmpp1 + sprintf(tmpp1, "Ign:On\n\r");
#endif
#if defined(ZTR1type)
									tmpp1 = tmpp1 + sprintf(tmpp1, "Vbt:%2.2fV\x1A", (double)Vbt/1000);
#else
									tmpp1 = tmpp1 + sprintf(tmpp1, "Vcc:%2.2fV\x1A", (double)Vcc/1000);
#endif
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "block##", 7))
							{
								eng_block = 1;
								DataEEWrite(eng_block, 0x08);
								if(!Rouming || (Settings & 64))
								{
									strcpy(outsms_txt, "Command \"block\" accepted\x1A");
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "sblock##", 8))
							{
								eng_block = 2;
								DataEEWrite(eng_block, 0x08);
								if(!Rouming || (Settings & 64))
								{
									strcpy(outsms_txt, "Command \"sblock\" accepted\x1A");
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "unblock##", 9))
							{
								eng_block = 0;
								DataEEWrite(eng_block, 0x08);
								if(!Rouming || (Settings & 64))
								{
									strcpy(outsms_txt, "Command \"unblock\" accepted\x1A");
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "status##", 8))
							{
								if(!Rouming || (Settings & 64))
								{
									CurPageNbr = GetCurrentPageNbr();
									NbrWrCnt = GetCurrentPageSts();
									memcpy(tmp_buf, &IMEI[1], 15);
									tmp_buf[15] = 0;
									sprintf(outsms_txt, "IMEI:%s\n\rVer:%u\n\rsimrev:%s\n\rSts_d:%u\n\rgprs_sts:%u\n\rCPN:%u,NWC:%u\x1A", tmp_buf, Version, simrev, Sts_d, gprs_state, CurPageNbr, NbrWrCnt);
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "cpureset##", 10))
							{
								cpureset = 1;
								if(!Rouming || (Settings & 64))
								{
									strcpy(outsms_txt, "Command \"cpureset\" accepted\x1A");
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "stsreset##", 10))
							{
								DataEEWrite(0x0000, 0x00);
								cpureset = 1;
								if(!Rouming || (Settings & 64))
								{
									strcpy(outsms_txt, "Command \"stsreset\" accepted\x1A");
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "gsmoffon##", 10))
							{
								offsim_f = 1;
								if(!Rouming || (Settings & 64))
								{
									strcpy(outsms_txt, "Command \"gsmoffon\" accepted\x1A");
									sendsms_f = 1;
								}
							}
							if(!memcmp(ibp1, "gpsoffon##", 10))
							{
								gpsreset = 1;
								if(!Rouming || (Settings & 64))
								{
									strcpy(outsms_txt, "Command \"gpsoffon\" accepted\x1A");
									sendsms_f = 1;
								}
							}
#if defined(RFM73)
							if(!memcmp(ibp1, "rfblock#", 8))
							{
								ibp1 = ibp1 + 8;
								tmpp1 = strchr(ibp1, '#');
								if(tmpp1)
								{
									tmpp1++;
									if(*tmpp1 == '#')
									{
										while(rfm73addr != 0)
											OS_Yield();
										rfm73addr = atoi(ibp1);
										rfm73cmd = 0xAA;
										rfm73ret = 0;
										RFmod_nbr = rfm73addr;
										RF_command = 1;
										while(rfm73addr != 0)
											OS_Yield();
										if(rfm73ret == 1)
										{
											if((!Rouming || (Settings & 64)))
											{
												strcpy(outsms_txt, "RF rele not answer\x1A");
												sendsms_f = 1;
											}
										}
										else
										{
											if((!Rouming || (Settings & 64)))
											{
												strcpy(outsms_txt, "Command \"rfblock\" accepted\x1A");
												sendsms_f = 1;
											}
										}
										pkt = 27;
										wr_pkt();
									}
								}
							}
							if(!memcmp(ibp1, "rfunblock#", 10))
							{
								ibp1 = ibp1 + 10;
								tmpp1 = strchr(ibp1, '#');
								if(tmpp1)
								{
									tmpp1++;
									if(*tmpp1 == '#')
									{
										while(rfm73addr != 0)
											OS_Yield();
										rfm73addr = atoi(ibp1);
										rfm73cmd = 0xBB;
										rfm73ret = 0;
										RFmod_nbr = rfm73addr;
										RF_command = 2;
										while(rfm73addr != 0)
											OS_Yield();
										if(rfm73ret == 1)
										{
											if((!Rouming || (Settings & 64)))
											{
												strcpy(outsms_txt, "RF rele not answer\x1A");
												sendsms_f = 1;
											}
										}
										else
										{
											if((!Rouming || (Settings & 64)))
											{
												strcpy(outsms_txt, "Command \"rfunblock\" accepted\x1A");
												sendsms_f = 1;
											}
										}
										pkt = 27;
										wr_pkt();
									}
								}
							}
							if(!memcmp(ibp1, "rfimmper#", 9))
							{
								ibp1 = ibp1 + 9;
								tmpp1 = strchr(ibp1, '#');
								if(tmpp1)
								{
									tmpp1++;
									if(*tmpp1 == '#')
									{
										while(rfm73addr != 0)
											OS_Yield();
										rfm73addr = atoi(ibp1);
										ibp1 = strchr(ibp1, ',');
										ibp1++;
										Imm_period = atoi(ibp1);
										rfm73cmd = 0xDD;
										rfm73ret = 0;
										while(rfm73addr != 0)
											OS_Yield();
										if(rfm73ret == 1)
										{
											if((!Rouming || (Settings & 64)))
											{
												strcpy(outsms_txt, "RF rele not answer\x1A");
												sendsms_f = 1;
											}
										}
										else
										{
											if((!Rouming || (Settings & 64)))
											{
												strcpy(outsms_txt, "Command \"rfimmper\" accepted\x1A");
												sendsms_f = 1;
											}
										}
									}
								}
							}
#endif
						}
						else
						{
							if(insms_nbr[0] && (!Rouming || (Settings & 64)))
							{
								strcpy(outsms_txt, "Wrong pass\x1A");
								sendsms_f = 1;
							}
						}
					}
				}
			}
			WaitAns("+CMGL", 10);
			if(!ret_err)
				smsread_cnt = 60;
			WaitAns("OK", 10);
			if(sms_nbr != 0xFFFF)
			{
				sprintf(tmp_buf, "AT+CMGD=%u\r", sms_nbr);
				SendCmd(tmp_buf, "OK", 10);
			}
			insms_f = 0;
			gsm_busy = 0;
		}
////////////////////////////////////////////////////////////////////////////////////////////////////
		if(sendsms_f && !gsm_busy && ((gsm_reg == 1) || (gsm_reg == 5)))
		{
			if(insms_nbr[0] != 0)
			{
				gsm_busy = 1;
				sprintf(tmp_buf, "AT+CMGS=\"+%s\"\r", insms_nbr);
				SendCmd(tmp_buf, "> ", 10);
				SendCmd(outsms_txt, "+CMGS", 100);
				gsm_busy = 0;
			}
			sendsms_f = 0;
		}
///////////////////////////////////////////////////////////////////////////////////////////////////
		if(cpureset)
		{
			save_pt();
			asm("RESET");
		}
///////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(RFM73)
		if(rfm73snd_f == 1)
		{
			while(rfm73addr != 0)
				OS_Yield();
			rfm73addr = RFmod_nbr;
			rfm73cmd = 0;
			if(RF_command == 1)
				rfm73cmd = 0xAA;
			if(RF_command == 2)
				rfm73cmd = 0xBB;
			rfm73ret = 0;
			while(rfm73addr != 0)
				OS_Yield();
			pkt = 27;
			wr_pkt();
			rfm73snd_f = 0;
			send_imm = send_imm | 2;
		}
		if(rfm73snd_f == 2)
		{
			while(rfm73addr != 0)
				OS_Yield();
			rfm73cmd = 0xDD;
			rfm73ret = 0;
			while(rfm73addr != 0)
				OS_Yield();
			rfm73snd_f = 0;
		}
#endif
/////////////////////////////////////////////////////////////////////////////////////
#if defined(RFM73)
		if((!imm_cnt) && (Settings1 & 8192)&& (uv_f == 0))
		{
			imm_cnt = 10;
			while(rfm73addr != 0)
				OS_Yield();
			rfm73addr = 1000;
			rfm73once = 1;
			rfm73cmd = 0xCC;
		}
#endif
		OS_Yield();
	}
}

void GSM_4(void)
{
	while(1)
	{
		if((gprs_state == 9) && !gsm_busy && reindent)
		{
			gsm_busy = 1;
SendCmd("ATE0\r", "OK", 10);
			IMEI[16] = (unsigned char)(Sts_d >> 8);
			IMEI[17] = (unsigned char)Sts_d;
			IMEI[18] = 0;
			for(cnt1 = 0; cnt1 < 18; cnt1++)
				IMEI[18] = IMEI[18] + IMEI[cnt1];

			SendCmd("AT+CIPQSEND=1\r", "OK", 10);
			SendCmd("AT+CIPSEND=19\r", "> ", 10);

			for(cnt3 = 0; cnt3 < 19; cnt3++)
			{
				U1TXREG = IMEI[cnt3];
			 	OS_Wait_TO(U1STAbits.TRMT, 1);
			}
			WaitAns("C0", 150);
			if(!ret_err && !memcmp(ib1, "C0", 2))
			{
				read_prm();
				senderror_cnt = 0;
				Rsindp_cnt = 0;
			}
			else
			{
				ChangeIP();
			}
SendCmd("ATE1\r", "OK", 10);
			gsm_busy = 0;
		}

		if((gprs_state == 9) && !gsm_busy)
		{
			gsm_busy = 1;
			if((eeprom_p1 != eeprom_p2) && !senderrt_cnt)
			{
				eeprom_p2tmp = eeprom_p2;
				outb1_col = 0;
				while((outb1_col < 675) && (eeprom_p2tmp != eeprom_p1))
				{
					out_cnt = pktsz(read_byte());
					for(cnt3 = 0; cnt3 < out_cnt; cnt3++)
					{
						outb1[outb1_col] = read_byte();
						eeprom_p2tmp++;
						outb1_col++;
						if(eeprom_p2tmp >= eeprom_size)
							eeprom_p2tmp = 0;
					}
				}

				if(Rsindp_cnt > Rsindp)
				{
SendCmd("ATE0\r", "OK", 10);
					IMEI[16] = (unsigned char)(Sts_d >> 8);
					IMEI[17] = (unsigned char)Sts_d;
					IMEI[18] = 0;
					for(cnt1 = 0; cnt1 < 18; cnt1++)
						IMEI[18] = IMEI[18] + IMEI[cnt1];

					SendCmd("AT+CIPQSEND=1\r", "OK", 10);
					SendCmd("AT+CIPSEND=19\r", "> ", 10);

					for(cnt3 = 0; cnt3 < 19; cnt3++)
					{
						U1TXREG = IMEI[cnt3];
		 				OS_Wait_TO(U1STAbits.TRMT, 1);
					}
					WaitAns("C0", 150);
					if(!ret_err && !memcmp(ib1, "C0", 2))
					{
						read_prm();
						senderror_cnt = 0;
						Rsindp_cnt = 0;
					}
					else
					{
						ChangeIP();
					}
SendCmd("ATE1\r", "OK", 10);
				}
				
SendCmd("ATE0\r", "OK", 10);
				sprintf(tmp_buf, "AT+CIPSEND=%u\r", outb1_col);
				SendCmd(tmp_buf, "> ", 10);
				for(cnt3 = 0; cnt3 < outb1_col; cnt3++)
				{
					U1TXREG = outb1[cnt3];
		 			OS_Wait_TO(U1STAbits.TRMT, 1);
				}
				WaitAns("C0", 150);
				if(!ret_err && !memcmp(ib1, "C0", 2))
				{
					Rsindp_cnt = 0;
					if(memcmp(ib1, "C09", 3))
					{
						eeprom_p2 = eeprom_p2tmp;
						buf_col_get();
						read_prm();
						reindent = 0;
					}
					else
						reindent = 1;
					senderror_cnt = 0;
				}
				else
				{
					ChangeIP();
				}
SendCmd("ATE1\r", "OK", 10);
			}
			if((!(Settings & 2048) || (senderror_cnt > 3)) && !reindent)
			{
				SendCmd("AT+CIPCLOSE=1\r", "CLOSE OK", 10);
				gprs_state = 12;
			}
			gsm_busy = 0;
			send_imm = 0;
			if(gprs_state != 9)
				send_prgrs = 0;
		}

		OS_Yield();
	}
}

void GSM_5(void)
{
	while(1)
	{
#if !defined(ZTR1type) || defined(ZTR_BTN)
		if(IN1)
		{
			if(Sts_d & 16384)
			{
				Sts_d = Sts_d & (0xFFFF - 16384);
				if(Settings1 & 2048)
				{
					pkt = 22;
					wr_pkt();
					send_imm = send_imm | 3;
				}
			}
		}
		else
		{
			if(!(Sts_d & 16384))
			{
				Sts_d = Sts_d | 16384;
				if(Settings1 & 2048)
				{
					pkt = 22;
					wr_pkt();
					send_imm = send_imm | 3;
				}
			}
		}
		if(IN2)
		{
			if(Sts_d & 32768)
			{
				Sts_d = Sts_d & (0xFFFF - 32768);
				if(Settings1 & 4096)
				{
					pkt = 22;
					wr_pkt();
					send_imm = send_imm | 3;
				}
			}
		}
		else
		{
			if(!(Sts_d & 32768))
			{
				Sts_d = Sts_d | 32768;
				if(Settings1 & 4096)
				{
					pkt = 22;
					wr_pkt();
					send_imm = send_imm | 3;
				}
			}
		}
#endif
#ifndef ZTR1type
		AD1CHS = 0x0001;
		ADCtmp = 0;
		for(cnt5 = 0; cnt5 < 4; cnt5++)
		{
			AD1CON1bits.SAMP = 1;
			OS_Delay(1);
			AD1CON1bits.SAMP = 0;
			OS_Wait_TO(AD1CON1bits.DONE, 5);
			ADCtmp = ADCtmp + (ADC1BUF0 >> 2);
		}
		Vcc = (unsigned char)(ADCtmp / 4);
		Vcc = Vcc * 154;
		if(Vcc < Vcc_lim)
		{
			if(!Vcc_limf)
			{
				Vcc_limf = 1;
				Sts_d = Sts_d | 2048;
				Event_nbr = 16;
				pkt = 18;
				wr_pkt();
			}
		}
		else
		{
			if(Vcc > (Vcc_lim + 500))
			{
				Vcc_limf = 0;
				Sts_d = Sts_d & (0xFFFF - 2048);
			}
		}

		if(Vcc < 5000)
		{
			if((Vbt < Vbt_off) && (Vbt != 0))
			{
				uv_f = 1;
				fun1();
				SIMoff();
#if defined(L50type) || defined(ZTR1type)
				GPS_TX_TRIS = 1;
				GPS_R_TRIS = 1;
				GPS_OFFB_TRIS = 1;
				GPS_ON = 0;
				LED_ER = 0;
				LED_FX = 0;
#endif
			}

			if(!(Sts_d & 512))
			{
				Event_nbr = 5;
				pkt = 18;
				wr_pkt();
				Sts_d = Sts_d | 512;
				if(Settings1 & 4)
				{
					send_imm = send_imm | 1;
				}
				if(Settings1 & 8)
					send_imm = send_imm | 2;
			}
		}
		else
		{
			uv_f = 0;
			if(Sts_d & 512)
			{
				Sts_d = Sts_d & (0xFFFF - 512);
				if(Settings1 & 4)
				{
					send_imm = send_imm | 1;
				}
				if(Settings1 & 8)
					send_imm = send_imm | 2;
			}
		}
//////////////////////////////////////////////////////
		if(!INZ)
		{
			if(!Ignition)
			{
				Ignition = 1;
				Sts_d = Sts_d | 8;
				if(Settings1 & 1)
					send_imm = send_imm | 1;
				if(Settings1 & 2)
					send_imm = send_imm | 2;
			}
		}
		else
		{
			if(Ignition)
			{
				Ignition = 0;
				Sts_d = Sts_d & (0xFFFF - 8);
				if(Settings1 & 1)
					send_imm = send_imm | 1;
				if(Settings1 & 2)
					send_imm = send_imm | 2;
			}
		}
#endif
/////////////////////////////////////////////////////////////////////////////////
		if((trip_cnt >= trip_per) && (trip_per != 0) && Move)
		{
			trip_cnt = 0;
			pkt = 6;
			wr_pkt();
		}
		if((debug1_cnt >= debug1_per) && (debug1_per != 0))
		{
			debug1_cnt = 0;
			pkt = 7;
			wr_pkt();
		}
		if((debug2_cnt >= debug2_per) && (debug2_per != 0))
		{
			debug2_cnt = 0;
		}
		if((can1_cnt >= can1_per) && (can1_per != 0) && (speed_can != 0xFF))
		{
			can1_cnt = 0;
			pkt = 9;
			wr_pkt();
		}
		if((can2_cnt >= can2_per) && (can2_per != 0) && (speed_can != 0xFF))
		{
			can2_cnt = 0;
			pkt = 10;
			wr_pkt();
		}
		if((send_cnt >= send_per) && (send_per != 0) && IMEI[1])
		{
			send_cnt = 0;
			out_send = 1;
		}
//////////////////////////////////////////////////////////////////////////////////////
		if(Settings & 4)
			Rouming = RoumingMAP;
		if(Settings & 8)
			Rouming = RoumingSIM;
		if((Settings & 4) && (Settings & 8))
		{
			if(RoumingSIM || RoumingMAP)
				Rouming = 1;
			else
				Rouming = 0;
		}
		if(!(Settings & 4) && !(Settings & 8))
			Rouming = 0;
////////////////////////////////////////////////////////////////////////////
		switch(Settings & 3)
		{
			case 0:
				if(Move)
				{
					Period = Period1;
					send_per = Send_per1;
				}
				else
				{
					Period = Period2;
					send_per = Send_per2;
				}
			break;
			case 1:
				if(Ignition)
				{
					Period = Period1;
					send_per = Send_per1;
				}
				else
				{
					Period = Period2;
					send_per = Send_per2;
				}
			break;
			case 2:
					Period = Period1;
					send_per = Send_per1;
			break;
			case 3:
				if(Move)
				{
					send_per = Send_per1;
					if(Turn)
						Period = Period3;
					else
						Period = Period1;
				}
				else
				{
					Period = Period2;
					send_per = Send_per2;
				}
			break;
		}
#ifndef ZTR1type
////////////////////////////////////////////////////////////////////////////
		if(!tripsv_cnt && Move)
		{
			tripsv_cnt = 7200;
#ifndef OZUmem
			save_pt();
#endif
		}
#endif
////////////////////////////////////////////////////////////////////////////
		if((((ident_cnt >= ident_per) && (ident_per != 0)) || ident_f)  && ICC[0] && simrev[0])
		{
			ident_f = 0;
			ident_cnt = 0;
			pkt = 26;
			wr_pkt();
		}
////////////////////////////////////////////////////////////////////////////
		if(((offsim_cnt >= offsim_per) && (offsim_per != 0) && !Move) || offsim_f)
		{
			offsim_f = 0;
			offsim_cnt = 0;
			SIMCOM_OFF
		}
////////////////////////////////////////////////////////////////////////////////////
#if !defined(ZTR1type) || defined(ZTR_BTN)
		if(Settings1 & 16)
		{
			if(Settings1 & 64)
			{
				if(IN1)
				{
					if(!(alert & 1))
					{
						Event_nbr = 9;
						pkt = 18;
						wr_pkt();
						alert = alert | 1;
						send_imm = send_imm | 3;
					}
				}
				else
				{
					if(alert & 1)
					{
						Event_nbr = 20;
						pkt = 18;
						wr_pkt();
						alert = alert & (0xFFFF - 1);
						send_imm = send_imm | 3;
					}
				}
			}
			else
			{
				if(!IN1)
				{
					if(!(alert & 1))
					{
						Event_nbr = 9;
						pkt = 18;
						wr_pkt();
						alert = alert | 1;
						send_imm = send_imm | 3;
					}
				}
				else
				{
					if(alert & 1)
					{
						Event_nbr = 20;
						pkt = 18;
						wr_pkt();
						alert = alert & (0xFFFF - 1);
						send_imm = send_imm | 3;
					}
				}
			}
		}
////////////////////////////////////////////////////////////////////////
		if(Settings1 & 32)
		{
			if(Settings1 & 128)
			{
				if(IN2)
				{
					if(!(alert & 2))
					{
						Event_nbr = 10;
						pkt = 18;
						wr_pkt();
						alert = alert | 2;
						send_imm = send_imm | 3;
					}
				}
				else
				{
					if(alert & 2)
					{
						Event_nbr = 21;
						pkt = 18;
						wr_pkt();
						alert = alert & (0xFFFF - 2);
						send_imm = send_imm | 3;
					}
				}
			}
			else
			{
				if(!IN2)
				{
					if(!(alert & 2))
					{
						Event_nbr = 10;
						pkt = 18;
						wr_pkt();
						alert = alert | 2;
						send_imm = send_imm | 3;
					}
				}
				else
				{
					if(alert & 2)
					{
						Event_nbr = 21;
						pkt = 18;
						wr_pkt();
						alert = alert & (0xFFFF - 2);
						send_imm = send_imm | 3;
					}
				}
			}
		}
#endif
////////////////////////////////////////////////////////////////////////
#ifndef ZTR1type
		if((INx_cnt >= INx_per) && (INx_per != 0))
		{
			INx_cnt = 0;
			if(IN1)
			{
				if(!(INx & 1))
				{
					INx = INx | 1;
				}
			}
			else
			{
				if(INx & 1)
				{
					INx = INx & (0xFFFF - 1);
				}
			}
			if(IN2)
			{
				if(!(INx & 2))
				{
					INx = INx | 2;
				}
			}
			else
			{
				if(INx & 2)
				{
					INx = INx & (0xFFFF - 2);
				}
			}
			pkt = 14;
			wr_pkt();
		}
////////////////////////////////////////////////////////////////////////
		if(!(eng_block & 3))
		{
			if(OUT1)
			{
				OUT1 = 0;
				Sts_d = Sts_d & (0xFFFF - 1024);
				send_imm = send_imm | 3;
			}
		}

		if(eng_block & 1)
		{
			if(!OUT1)
			{
				OUT1 = 1;
				Sts_d = Sts_d | 1024;
				send_imm = send_imm | 3;
			}
		}

		if((eng_block & 2) && !Move && (((SatUs >> 6) == 2) && (Latitude != 0) && (Longitude != 0)))
		{
			if(!OUT1)
			{
				OUT1 = 1;
				Sts_d = Sts_d | 1024;
				send_imm = send_imm | 3;
			}
		}
#endif
////////////////////////////////////////////////////////////////////////
		if(jd == 1)
		{
			jd = 0;
			if(!(Sts_d & 256))
			{
				Event_nbr = 7;
				pkt = 18;
				wr_pkt();
				Sts_d = Sts_d | 256;
			}
		}
		if(jd == 2)
		{
			jd = 0;
			if(Sts_d & 256)
			{
				Sts_d = Sts_d & (0xFFFF - 256);
			}
		}
		OS_Yield();
	}
}

void GSM_6(void)
{
	while(1)
	{
///////////////////////////////////////////////////////////////////////////////
		if((ADC_cnt >= ADC_per) && (ADC_per != 0))
		{
			ADC_cnt = 0;
#ifndef ZTR1type
			AD1CHS = 0x0009;
			ADCtmp = 0;
			for(cnt5 = 0; cnt5 < 8; cnt5++)
			{
				AD1CON1bits.SAMP = 1;
				OS_Delay(1);
				AD1CON1bits.SAMP = 0;
				OS_Wait_TO(AD1CON1bits.DONE, 5);
				ADCtmp = ADCtmp + (ADC1BUF0 >> 2);
			}
			ADC1 = (unsigned char)(ADCtmp / 8);
#endif
#if defined(SIM908type)
			AD1CHS = 0x000A;
			ADCtmp = 0;
			for(cnt5 = 0; cnt5 < 8; cnt5++)
			{
				AD1CON1bits.SAMP = 1;
				OS_Delay(1);
				AD1CON1bits.SAMP = 0;
				OS_Wait_TO(AD1CON1bits.DONE, 5);
				ADCtmp = ADCtmp + (ADC1BUF0 >> 2);
			}
			ADC2 = (unsigned char)(ADCtmp / 8);
#endif

			pkt = 13;
			wr_pkt();
		}
/////////////////////////////////////////////////////////////////////
#ifndef ZTR1type
		if((freq_cnt >= freq_per) && (freq_per != 0))
		{
			freq_cnt = 0;
			TMR3 = 0;
			IFS0bits.T3IF = 0;
			t3_f = 1;
			in_tmp1 = IN1;
			in_tmp2 = IN2;
			freq1 = 0;
			freq2 = 0;
			while(t3_f)
			{
				if(in_tmp1 != IN1)
				{
					freq1++;
					in_tmp1 = IN1;
				}
				if(in_tmp2 != IN2)
				{
					freq2++;
					in_tmp2 = IN2;
				}
			}
			freq1 = freq1 / 2;
			freq2 = freq2 / 2;
			pkt = 12;
			wr_pkt();
		}
#endif
		OS_Yield();
	}
}

void GSM_7(void)
{
	while(1)
	{
		if((gprs_state != 5 && gprs_state != 12 && gprs_state != 9) && !gsm_busy && gprs_reg && (!Rouming || (Settings & 32)) && !gprsregt_cnt && !(Settings & 512))
		{
			gsm_busy = 1;
			SendCmd("AT+CIPSHUT\r", "SHUT OK", 10);
			SendCmd("AT+CIPMUX=0\r", "OK", 10);
			SendCmd("AT+CIPMODE=0\r", "OK", 10);
			if(APN_user[0])
				sprintf(tmp_buf, "AT+CSTT=\"%s\",\"%s\",\"%s\"\r", APN, APN_user, APN_pass);
			else
				sprintf(tmp_buf, "AT+CSTT=\"%s\"\r", APN);
			SendCmd(tmp_buf, "OK", 100);
			SendCmd("AT+CIICR\r", "OK", 100);
			if(!ret_err)
			{
				SendCmd("AT+CIFSR\r", "10", 100);
				gsm_busy = 0;
				gprs_state = 5;
				gprsregerr_cnt = 0;
			}
			else
			{
				gprsregerr_cnt++;
				if(gprsregerr_cnt > 30)
				{
					SIMCOM_OFF
				}
				else
					gprsregt_cnt = 60;
			}
			gsm_busy = 0;
		}
		OS_Yield();
	}
}

void GSM_8(void)
{
	while(1)
	{
		if(((out_send == 1) || (Settings & 2048)) && !gsm_busy && (gprs_state == 5 || gprs_state == 12) && !incall_prgrs && !senderrt_cnt && gprs_reg)
		{
			out_send = 0;
			gsm_busy = 1;
			send_prgrs = 1;
			switch(IPnbr)
			{
				case 0:
					if(PortPr & 1)
						sprintf(tmp_buf, "AT+CIPSTART=\"UDP\",\"%s\",\"%u\"\r", ExIP1, ExPort1);
					else
						sprintf(tmp_buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%u\"\r", ExIP1, ExPort1);
				break;
				case 1:
					if(PortPr & 2)
						sprintf(tmp_buf, "AT+CIPSTART=\"UDP\",\"%s\",\"%u\"\r", ExIP2, ExPort2);
					else
						sprintf(tmp_buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%u\"\r", ExIP2, ExPort2);
				break;
				case 2:
					if(PortPr & 4)
						sprintf(tmp_buf, "AT+CIPSTART=\"UDP\",\"%s\",\"%u\"\r", ExIP3, ExPort3);
					else
						sprintf(tmp_buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%u\"\r", ExIP3, ExPort3);
				break;
			}
			SendCmd(tmp_buf, "OK", 10);
			WaitAns("CONNECT", 150);
			if(!ret_err)
			{
				if(!strcmp(ib1, "CONNECT OK"))
				{
					gprs_state = 9;
SendCmd("ATE0\r", "OK", 10);
					if(Rsindp_cnt > Rsindp)
					{
						IMEI[16] = (unsigned char)(Sts_d >> 8);
						IMEI[17] = (unsigned char)Sts_d;
						IMEI[18] = 0;
						for(cnt1 = 0; cnt1 < 18; cnt1++)
							IMEI[18] = IMEI[18] + IMEI[cnt1];

						SendCmd("AT+CIPQSEND=1\r", "OK", 10);
						SendCmd("AT+CIPSEND=19\r", "> ", 10);

						for(cnt3 = 0; cnt3 < 19; cnt3++)
						{
							U1TXREG = IMEI[cnt3];
						 	OS_Wait_TO(U1STAbits.TRMT, 1);
						}
						WaitAns("C0", 150);
						if(!ret_err && !memcmp(ib1, "C0", 2))
						{
							read_prm();
							senderror_cnt = 0;
							gprs_state = 9;
							Rsindp_cnt = 0;
						}
						else
						{
							ChangeIP();
						}
					}
				}
				else
				{
					ChangeIP();
				}
			}
			else
			{
				ChangeIP();
			}
SendCmd("ATE1\r", "OK", 10);
			gsm_busy = 0;
			if(gprs_state != 9)
				send_prgrs = 0;
		}
		OS_Yield();
	}
}

void LED1(void)
{
	for (;;)
	{
		if(gps_ok)
		{
			LED_FX = 1;
			OS_Delay(1);
			LED_FX = 0;
			if((SatUs >> 6) == 1)
			{
				OS_Delay(2);
				LED_FX = 1;
				OS_Delay(1);
				LED_FX = 0;
			}
			if((SatUs >> 6) == 2)
			{
				OS_Delay(2);
				LED_FX = 1;
				OS_Delay(1);
				LED_FX = 0;
				OS_Delay(2);
				LED_FX = 1;
				OS_Delay(1);
				LED_FX = 0;
			}
			OS_Delay(20);
		}
		OS_Yield();
	}
}

void LED2(void)
{
	for (;;)
	{
		ClrWdt();
#ifndef ZTR1type
		if(senderror_cnt != 0)
			LED_ER = 1;
		else
		if(incall_prgrs)
		{
			LED_ER = 1;
			OS_Delay(1);
			LED_ER = 0;
			OS_Delay(1);
		}
		else
		if(send_prgrs)
		{
			LED_ER = 1;
			OS_Delay(2);
			LED_ER = 0;
			OS_Delay(2);
		}
		else
			LED_ER = 0;
#endif
		OS_Yield();
	}
}

#if defined(RFM73)
void RFM73T(void)
{
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
	rfm73_crc_length(2);
	rfm73_address_length(3);
	rfm73_mode_transmit();

	for (;;)
	{
		if(rfm73addr != 0)
		{
			for(rfm73snd_cnt = 0; rfm73snd_cnt < 5; rfm73snd_cnt++)
			{
				TX_Address[2] = (unsigned char)rfm73addr;
				TX_Address[1] = (unsigned char)(rfm73addr >> 8);
				rfm73_transmit_address(TX_Address);
		    	rfm73_receive_address_p0(TX_Address);
				rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
				rfm73tmp = rfm73_register_read(RFM73_REG_STATUS);
				rfm73_register_write(RFM73_REG_STATUS, rfm73tmp);
				rfm73buf[0] = 0;
				rfm73buf[1] = Imm_period;
				rfm73buf[2] = rfm73cmd;
				if(rfm73once == 1)
					rfm73_buffer_write( RFM73_CMD_W_TX_PAYLOAD_NOACK, rfm73buf, 3);
				else
					rfm73_buffer_write(RFM73_CMD_W_TX_PAYLOAD, rfm73buf, 3);
				do{
					rfm73tmp = rfm73_register_read(RFM73_REG_STATUS);
					OS_Yield();
				}while(!(rfm73tmp & (STATUS_MAX_RT | STATUS_TX_DS)));
				if((rfm73tmp & STATUS_MAX_RT))
				{
					rfm73_register_write(RFM73_CMD_FLUSH_TX, 0);
					rfm73_register_write(RFM73_REG_STATUS, rfm73tmp);
					rfm73ret = 1;
					break;
				}
				else
				{
					rfm73_register_write(RFM73_REG_STATUS, rfm73tmp);
					rfm73ret = 0;
				}
			}
			rfm73once = 0;
			rfm73addr = 0;
		}
		OS_Yield();
	}
}
#endif

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{
/* Interrupt Service Routine code goes here */
	IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag and Return from ISR

	if(Rsindp_cnt != 0xFFFF)
		Rsindp_cnt++;
	timec++;
	if(U1STAbits.OERR)
		U1STAbits.OERR = 0;
	if(U2STAbits.OERR)
		U2STAbits.OERR = 0;

	if(!Stealth_per)
	{
		if(send_cnt != 0xFFFF)
			send_cnt++;
		if(prm_cnt != 0xFFFF)
			prm_cnt++;
		if(gsmloc_cnt != 0xFFFF)
			gsmloc_cnt++;
		if(money_cnt != 0xFFFF)
			money_cnt++;
		if(trip_cnt != 0xFFFF)
			trip_cnt++;
		if(debug1_cnt != 0xFFFF)
			debug1_cnt++;
		if(debug2_cnt != 0xFFFF)
			debug2_cnt++;
		if(can1_cnt != 0xFFFF)
			can1_cnt++;
		if(can2_cnt != 0xFFFF)
			can2_cnt++;
		if(ADC_cnt != 0xFFFF)
			ADC_cnt++;
		if(INx_cnt != 0xFFFF)
			INx_cnt++;
		if(freq_cnt != 0xFFFF)
			freq_cnt++;
		if(ident_cnt != 0xFFFF)
			ident_cnt++;
		if(offsim_cnt != 0xFFFF)
			offsim_cnt++;
		if(tripsv_cnt)
			tripsv_cnt--;
	}

	if(smsread_cnt)
		smsread_cnt--;
	if(netchk_cnt)
		netchk_cnt--;
	if(gprsregt_cnt)
		gprsregt_cnt--;
	if(senderrt_cnt)
		senderrt_cnt--;
	if(stealth_ontmc)
		stealth_ontmc--;
#if defined(RFM73)
	if(imm_cnt)
		imm_cnt--;
#endif
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
	switch(SU_state)
	{
		case 0:
			if(SUART == 1)
			{
				SU_recbc++;
				if(SU_recbc > 8)
				{
					SU_state = 1;
					SU_recbc1 = 0;
				}
			}
			else
				SU_recbc = 0;
		break;
		case 1:
			if(SUART == 0)
			{
				SU_state = 2;
				SU_recb = 0;
				SU_recbc = 0;
			}
		break;
		case 2:
			SU_recb = SU_recb >> 1;
			if(SUART == 1)
				SU_recb = SU_recb | 0x80;
			SU_recbc++;
			if(SU_recbc == 8)
			{
				SU_state = 3;
			}
		break;
		case 3:
			if(SUART == 1)
			{
				SU_recf = 1;
				SU_state = 0;
				SU_recbc = 0;
				SU_recbc1++;
				if(SU_recbc1 == 1)
				{
					if(SU_recb == 0x55)
					{
						SU_state = 1;
					}
					else
						SU_state = 0;
				}
				if(SU_recbc1 == 2)
				{
					if(SU_recb == 0x55)
					{
						SU_state = 1;
					}
					else
						SU_state = 0;
				}
				if(SU_recbc1 == 3)
				{
					if(SU_recb == 0x55)
					{
						SU_state = 1;
						CRC_can = 0;
					}
					else
						SU_state = 0;
				}
				if(SU_recbc1 == 4)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					speed_cant = SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 5)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					accel_cant = SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 6)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					fuel_cant = SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 7)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					temp_cant = SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 8)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					rpm_cant = SU_recb;
					rpm_cant = rpm_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 9)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					rpm_cant = rpm_cant + SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 10)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					fuela_cant = SU_recb;
					fuela_cant = fuela_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 11)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					fuela_cant = fuela_cant + SU_recb;
					fuela_cant = fuela_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 12)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					fuela_cant = fuela_cant + SU_recb;
					fuela_cant = fuela_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 13)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					fuela_cant = fuela_cant + SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 14)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					mth_cant = SU_recb;
					mth_cant = mth_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 15)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					mth_cant = mth_cant + SU_recb;
					mth_cant = mth_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 16)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					mth_cant = mth_cant + SU_recb;
					mth_cant = mth_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 17)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					mth_cant = mth_cant + SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 18)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					dist_cant = SU_recb;
					dist_cant = dist_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 19)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					dist_cant = dist_cant + SU_recb;
					dist_cant = dist_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 20)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					dist_cant = dist_cant + SU_recb;
					dist_cant = dist_cant << 8;
					SU_state = 1;
				}
				if(SU_recbc1 == 21)
				{
					CRC_can = (unsigned char)(CRC_can + SU_recb);
					dist_cant = dist_cant + SU_recb;
					SU_state = 1;
				}
				if(SU_recbc1 == 22)
				{
					if((CRC_can == SU_recb) && (can_read == 0))
					{
						speed_can = speed_cant;
						accel_can = accel_cant;
						fuel_can = fuel_cant;
						temp_can = temp_cant;
						rpm_can = rpm_cant;
						fuela_can = fuela_cant;
						mth_can = mth_cant;
						dist_can = dist_cant;
					}
					SU_state = 0;
				}
			}
		break;
	}
}
#endif

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;
	while(U1STA & 1)
	{
		in_byte1 = U1RXREG;
		if(ibp1 > &ib1[295])
			ibp1 = ib1;
		if(in_byte1 > 0x1F)
			*ibp1++ = in_byte1;
		else
		{
			if(in_byte1 == 0x0A)
			{
				if(ibp1 != ib1)
				{
					*ibp1++ = 0x00;
					RTS = 1;
					ibp1 = ib1;
					if(!memcmp(ib1, "+CMTI", 5))
						insms_f = 1;
					else
					if(!strcmp(ib1, "+SJDR: JAMMING DETECTED"))
						jd = 1;
					else
					if(!strcmp(ib1, "+SJDR: NO JAMMING"))
						jd = 2;
				}
			}
		}
		if((ib1[0] == '>') && (ib1[1] == ' '))
		{
			*ibp1++ = 0x00;
			RTS = 1;
			ibp1 = ib1;
		}
	}
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void)
{
	IFS1bits.U2RXIF = 0;
	while(U2STA & 1)
	{
		in_byte2 = U2RXREG;
		switch(gps_state)
		{
			case 0:
				if(in_byte2 == '$')
				{
					gps_ok = 1;
					gps_state = 1;
					ibp2 = ib2;
				}
			break;
			case 1:
				*ibp2++ = in_byte2;
				if(in_byte2 == '*')
				{
					ibp2 = ib2;
					if(!memcmp(ibp2 + 2, "RMC", 3))
					{
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							Hour = ((*ibp2 - 0x30) * 10) + (*(ibp2 + 1) - 0x30);
							Minute = ((*(ibp2 + 2) - 0x30) * 10) + (*(ibp2 + 3) - 0x30);
							Second = ((*(ibp2 + 4) - 0x30) * 10) + (*(ibp2 + 5) - 0x30);
						}
						else
						{
							Hour = 0;
							Minute = 0;
							Second = 0;
						}
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
						{
							deg = ((*ibp2 - 0x30) * 10) + (*(ibp2 + 1) - 0x30);
							ibp2 = ibp2 + 2;
							Latitude = atoi(ibp2);
							Latitude = Latitude * 10000;
							ibp2 = ibp2 + 3;
#if defined(SIM908type)
							*(ibp2 + 4) = 0x20;
#endif
							Latitude = atoi(ibp2) + Latitude;
							Latitude = Latitude * 0.16666666;
							Latitude = (deg * 100000) + Latitude;
						}
						else
							Latitude = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
						{
							deg = ((*ibp2 - 0x30) * 100) + ((*(ibp2 + 1) - 0x30) * 10) + (*(ibp2 + 2) - 0x30);
							ibp2 = ibp2 + 3;
							Longitude = atoi(ibp2);
							Longitude = Longitude * 10000;
							ibp2 = ibp2 + 3;
#if defined(SIM908type)
							*(ibp2 + 4) = 0x20;
#endif
							Longitude = atoi(ibp2) + Longitude;
							Longitude = Longitude * 0.16666666;
							Longitude = (deg * 100000) + Longitude;
						}
						else
							Longitude = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
							Speed = atoi(ibp2);
						else
							Speed = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2) & ((SatUs >> 6) > 0))
							Course = atoi(ibp2);
						else
							Course = 0;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							Day = ((*ibp2 - 0x30) * 10) + (*(ibp2 + 1) - 0x30);
							Month = ((*(ibp2 + 2) - 0x30) * 10) + (*(ibp2 + 3) - 0x30);
							Year = ((*(ibp2 + 4) - 0x30) * 10) + (*(ibp2 + 5) - 0x30);
						}
						else
						{
							Day = 0;
							Month = 0;
							Year = 0;
						}
						pps1 = 1;
					}
					if(!memcmp(ibp2 + 2, "GGA", 3))
					{
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							SatUs = SatUs & 0b11000000;
							SatUs = SatUs + atoi(ibp2);
						}
					}
					if(!memcmp(ibp2 + 2, "GSA", 3))
					{
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						ibp2 = strchr(ibp2, ',');
						ibp2++;
						if(isdigit(*ibp2))
						{
							SatUs = SatUs & 0b00111111;
							SatUs = SatUs + ((atoi(ibp2) - 1) << 6);
						}
					}
					gps_state = 0;
				}
			break;
		}
	}
}
