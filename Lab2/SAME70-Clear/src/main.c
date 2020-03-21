/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        12                  // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define BUT_PIO        PIOA                 // periferico que controla o BUTTOM
#define BUT_PIO_ID     10                  // ID do periférico PIOC (controla BUTTOM)
#define BUT_PIO_IDX     8                    // ID do BUTTOM no PIO
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)   // Mascara para CONTROLARMOS o BUTTOM
#define PIOA   ((Pio    *)0x400E0E00U) 

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void)
{
	// Initialize the board clock
	sysclk_init();
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	
}

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
					const uint32_t ul_pull_up_enable)
{
	
}
// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	  _pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
	  delay_ms(200);                        // Delay por software de 200 ms
	  pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
	  delay_ms(200);                        // Delay por software de 200 ms
  }
  return 0;
}
