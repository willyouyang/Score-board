/*#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h" */

/*--------hardward setting--------*/

#define MAX_7_SEGAMENT_LED_NUMBER 10
#define Buzzer_NUM 1
#define MAX_STRLEN 12
#define Delay_Time 100

/*--------hardward setting--------*/

#define LED_0 0
#define LED_1 1
#define LED_2 2
#define LED_3 3
#define LED_4 4
#define LED_5 5
#define LED_6 6
#define LED_7 7
#define LED_8 8
#define LED_9 9

#define time_always_carry(NUM) (NUM/1000+(NUM%1000!=0))

volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string
static volatile int TimeDelay=0,field_time=0,field_pause=0,time_24=0,time_14,long_pause=0,short_pause=0;
int count_24_sec,count_field,count_pause;
static int pause_flag=1,switch_flag=0,disable_24_flag=0;

struct control_power
{
	GPIO_TypeDef *GPIOx;
	uint32_t on_off;
	
};


void setup_led(void);
void USART1_IRQHandler(void);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void usart_rxtx(void);
void init_USART1(uint32_t baudrate);

void shownumber(unsigned int No_led,unsigned int number);
void set_count_time(unsigned int time0,unsigned int time1,unsigned int time2,unsigned int time3,unsigned int time4);
void refresh_led(void);

void modify_scores(int *target,int modify);
void modify_time(int min,int sec);

void stop(void),start(void);
void pause(unsigned int type);
void disable_24_sec(void);
void switch_field(void);
void reset_24_sec(void),reset_14_sec(void);
void restart(void);
void buzzer(void);
void close_buzzer(void);
void recover(void);
void send_time();

#ifdef info_in_hardware 
/*--------basketball rules--------*/

#define MAX_team_player 10
#define MAX_long_pause 1
#define MAX_short_pause 2

/*--------basketball rules--------*/
struct info_game
{
	
};
struct info_team
{
		unsigned int score;
		unsigned int info_player[MAX_team_player];
		unsigned int long_pause_times;
		unsigned int short_pause_times;
};

struct info_player
{
		unsigned int point_3;
		unsigned int point_2;
		unsigned int point_1;
		unsigned int foul;
		unsigned int rebound;
		unsigned int no;
};
#else
int score[2]={0,0};
#endif

