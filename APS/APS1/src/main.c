#include "asf.h"
#include "musicas.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/


// LED PLACA
#define LED_BOARD_PIO      PIOC
#define LED_BOARD_PIO_ID   ID_PIOC
#define LED_BOARD_IDX      8
#define LED_BOARD_IDX_MASK (1 << LED_BOARD_IDX)

// LED1 OLED
#define LED1_OLED_PIO      PIOA
#define LED1_OLED_PIO_ID   ID_PIOA
#define LED1_OLED_IDX      0
#define LED1_OLED_IDX_MASK (1 << LED1_OLED_IDX)

// LED2 OLED
#define LED2_OLED_PIO      PIOC
#define LED2_OLED_PIO_ID   ID_PIOC
#define LED2_OLED_IDX      30
#define LED2_OLED_IDX_MASK (1 << LED2_OLED_IDX)

// LED3 OLED
#define LED3_OLED_PIO      PIOB
#define LED3_OLED_PIO_ID   ID_PIOB
#define LED3_OLED_IDX      2
#define LED3_OLED_IDX_MASK (1 << LED3_OLED_IDX)

// BUT OLED 1
#define BUT1_PIO           PIOD
#define BUT1_PIO_ID        ID_PIOD
#define BUT1_PIO_IDX       28
#define BUT1_PIO_IDX_MASK  (1u << BUT1_PIO_IDX)

// BUT OLED 2
#define BUT2_PIO           PIOC
#define BUT2_PIO_ID        ID_PIOC
#define BUT2_PIO_IDX       31
#define BUT2_PIO_IDX_MASK  (1u << BUT2_PIO_IDX)

// BUT OLED 3
#define BUT3_PIO		   PIOA
#define BUT3_PIO_ID		   ID_PIOA
#define BUT3_PIO_IDX	   19
#define BUT3_PIO_IDX_MASK  (1u << BUT3_PIO_IDX)

// BUT 4
#define BUT4_PIO		   PIOA
#define BUT4_PIO_ID		   ID_PIOA
#define BUT4_PIO_IDX	   11
#define BUT4_PIO_IDX_MASK  (1u << BUT4_PIO_IDX)

// BUZZER
#define BUZZER_PIO      PIOA
#define BUZZER_PIO_ID   ID_PIOA
#define BUZZER_IDX      3
#define BUZZER_IDX_MASK (1 << BUZZER_IDX)

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */

typedef struct  {
	int frequency;
	double duration;
} note_s;

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile char b1 = 0;
volatile char b2 = 0;
volatile char b3 = 0;
volatile char b4 = 0;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void LED_init(int estado);
void play_music(note_s melody[], double songspeed, int n);
void tone(int frequency, double duration);
void build_melody(int note[], int duration[], note_s melody[], int n);
void apaga_leds();
void acende_leds();
/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
*  Handle Interrupcao botao
*/
void Button1_Handler(void)
{
	b1 = 1;
}

void Button2_Handler(void)
{
	b2 = 1;
}

void Button3_Handler(void)
{
	b3 = 1;
}

void Button4_Handler(void)
{
	b4 = 1;
}



/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void acende_leds(){
	pio_set(LED3_OLED_PIO, LED3_OLED_IDX_MASK);
	pio_set(LED2_OLED_PIO, LED2_OLED_IDX_MASK);
	pio_set(LED1_OLED_PIO, LED1_OLED_IDX_MASK);
}

void apaga_leds(){
	pio_clear(LED3_OLED_PIO, LED3_OLED_IDX_MASK);
	pio_clear(LED2_OLED_PIO, LED2_OLED_IDX_MASK);
	pio_clear(LED1_OLED_PIO, LED1_OLED_IDX_MASK);
}

void build_melody(int note[], int duration[], note_s melody[], int n){
	for (int i=0; i<n; i++){
		melody[i].frequency = note[i];
		melody[i].duration = duration[i];
	}
	return melody;
}

void tone(int frequency, double duration){
	double half_period;
	
	if(frequency != 0){
		half_period = ((1.0/frequency)/2)*1000000;
		
		for (double timer=0.0; timer<duration*1000; timer += half_period*2){
			//coloca 1 no pino do LED	
			pio_set(BUZZER_PIO, BUZZER_IDX_MASK);
			delay_us(half_period);
			//coloca 0 no pino do LED
			pio_clear(BUZZER_PIO, BUZZER_IDX_MASK);
			delay_us(half_period);
		}
	}else{
		delay_ms(duration);
	}
}

void play_music(note_s melody[], double songspeed, int n){
	for (int i=0; i < n; i++){
		double wait = (melody[i].duration) * songspeed;
		tone(melody[i].frequency, wait);
		if (b4 == 1){
			b4 = 0;
			delay_ms(500);
			while(1){
				if(b4 == 1){
					b4 = 0;
					break;
				}
			}
		}
		acende_leds();
		delay_ms(wait);
		apaga_leds();
		
	}
}


/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED1_OLED_PIO_ID);
	pio_set_output(LED1_OLED_PIO , LED1_OLED_IDX_MASK, estado, 0, 0);

	pmc_enable_periph_clk(LED2_OLED_PIO_ID);
	pio_set_output(LED2_OLED_PIO , LED2_OLED_IDX_MASK, estado, 0, 0);
	
	pmc_enable_periph_clk(LED3_OLED_PIO_ID);
	pio_set_output(LED3_OLED_PIO , LED3_OLED_IDX_MASK, estado, 0, 0);

	pmc_enable_periph_clk(LED_BOARD_PIO_ID);
	pio_set_output(LED_BOARD_PIO , LED_BOARD_IDX_MASK, estado, 0, 0);
};


/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	gfx_mono_ssd1306_init();

	/* Configura Leds */
	LED_init(0);
	/* Button*/
	
	//Inicializa o Buzzer
	pmc_enable_periph_clk(BUZZER_PIO_ID);

	//Inicializa PC8 como saída
	pio_set_output(BUZZER_PIO, BUZZER_IDX_MASK, 0, 0, 0);
	
	//Inicializa Buttons
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	//Interrupt
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, Button2_Handler);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, Button3_Handler);
	pio_handler_set(BUT4_PIO, BUT4_PIO_ID, BUT4_PIO_IDX_MASK, PIO_IT_FALL_EDGE, Button4_Handler);
	
	//Interrupts
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_enable_interrupt(BUT4_PIO, BUT4_PIO_IDX_MASK);
	
	//NVIC
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_EnableIRQ(BUT4_PIO_ID);
	
	b1 = 0;
	b2 = 0;
	b3 = 0;
	b4 = 0;
	
	note_s pirate_melody[N_PIRATE];
	build_melody(pirate_notes, pirate_duration, pirate_melody, N_PIRATE);
	
	note_s mario_melody[N_MARIO_MAIN];
	build_melody(mario_notes, mario_temp, mario_melody, N_MARIO_MAIN);
	
	note_s mario_underworld_melody[N_UNDERWORLD];
	build_melody(underworld_notes, underworld_duration, mario_underworld_melody, N_UNDERWORLD);
  
	while (1){
		if(b1 == 1){
			gfx_mono_draw_string("Caribeam", 3,8, &sysfont);
			play_music(pirate_melody, 0.5, N_PIRATE);
			gfx_mono_draw_string("                    ", 3,8, &sysfont);
			b1 = 0;
		}
		else if(b2 == 1){
			gfx_mono_draw_string("Mario Theme", 3,8, &sysfont);
			play_music(mario_melody, 7, N_MARIO_MAIN);
			gfx_mono_draw_string("                    ", 3,8, &sysfont);
			b2 = 0;
		}
		else if(b3 == 1){
			gfx_mono_draw_string("Under World", 3,8, &sysfont);
			play_music(mario_underworld_melody, 7, N_UNDERWORLD);
			gfx_mono_draw_string("                    ", 3,8, &sysfont);
			b3 = 0;
		}
	}
}

