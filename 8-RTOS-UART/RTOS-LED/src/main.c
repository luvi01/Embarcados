/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE  (1024/sizeof(portSTACK_TYPE))

#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_LED1_STACK_SIZE  (1024/sizeof(portSTACK_TYPE))

#define TASK_LED2_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_LED2_STACK_SIZE  (1024/sizeof(portSTACK_TYPE))

#define TASK_LED3_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_LED3_STACK_SIZE  (1024/sizeof(portSTACK_TYPE))

#define TASK_COMMAND_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_COMMAND_STACK_SIZE  (1024/sizeof(portSTACK_TYPE))

#define TASK_EXECUTE_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_EXECUTE_STACK_SIZE  (1024/sizeof(portSTACK_TYPE))

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



extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);



/** Semaforo a ser usado pela task led 
    tem que ser var global! */
SemaphoreHandle_t xSemaphore1;

SemaphoreHandle_t xSemaphore2;

SemaphoreHandle_t xSemaphore3;




typedef struct {
	char a;
} uartData;

typedef struct {
	char command[10];
} commandData;


//Fila
QueueHandle_t xQueueChar;
QueueHandle_t xQueueCommand;


/**
* callback do botao
* libera semaforo: xSemaphore
*/
void but1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

/**
* callback do botao
* libera semaforo: xSemaphore
*/
void but2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore2, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

/**
* callback do botao
* libera semaforo: xSemaphore
*/
void but3_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore3, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

static void task_led1(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore1 = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init bot�o */
  pmc_enable_periph_clk(BUT1_PIO_ID);
  pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
  pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT1_PIO_ID);
  NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
  
  int i = 0;
  if (xSemaphore1 == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore1, ( TickType_t ) 500) == pdTRUE ){
		if(i){
			pio_set(LED1_OLED_PIO, LED1_OLED_IDX_MASK);
			i = 0;

		}else{
			pio_clear(LED1_OLED_PIO, LED1_OLED_IDX_MASK);
			i = 1;
		}
      		

    }
  }
}

static void task_led2(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore2 = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init bot�o */
  pmc_enable_periph_clk(BUT2_PIO_ID);
  pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
  pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT2_PIO_ID);
  NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4
  int i = 0;
  if (xSemaphore2 == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore2, ( TickType_t ) 500) == pdTRUE ){
      if(i){
	      pio_set(LED2_OLED_PIO, LED2_OLED_IDX_MASK);
	      i = 0;

	      }else{
	      pio_clear(LED2_OLED_PIO, LED2_OLED_IDX_MASK);
	      i = 1;
      }
    }
  }
}

static void task_led3(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore3 = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init bot�o */
  pmc_enable_periph_clk(BUT3_PIO_ID);
  pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback);
  pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT3_PIO_ID);
  NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
  int i = 0;
  if (xSemaphore3 == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore3, ( TickType_t ) 500) == pdTRUE ){
      if(i){
	      pio_set(LED3_OLED_PIO, LED3_OLED_IDX_MASK);
	      i = 0;

	      }else{
	      pio_clear(LED3_OLED_PIO, LED3_OLED_IDX_MASK);
	      i = 1;
      }
    }
  }
}



/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(3000);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
//static void task_led(void *pvParameters)
//{
	//UNUSED(pvParameters);
	//for (;;) {
		//LED_Toggle(LED0);
		//vTaskDelay(1000);
	//}
//}

static void task_led(void *pvParameters)
{
	UNUSED(pvParameters);
	
	const TickType_t tempo = 3000 / portTICK_PERIOD_MS;
	const TickType_t pisca = 200 / portTICK_PERIOD_MS;
	for (;;) {
		for(int i = 0; i<5; i++){
			LED_Toggle(LED0);
			vTaskDelay(1000);
		}
		vTaskDelay(tempo);
	}
}


static void USART1_init(void){
	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4); // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

	/* Configura opcoes USART */
	const sam_usart_opt_t usart_settings = {
		.baudrate       = 115200,
		.char_length    = US_MR_CHRL_8_BIT,
		.parity_type    = US_MR_PAR_NO,
		.stop_bits    = US_MR_NBSTOP_1_BIT    ,
		.channel_mode   = US_MR_CHMODE_NORMAL
	};

	/* Ativa Clock periferico USART0 */
	sysclk_enable_peripheral_clock(ID_USART1);

	stdio_serial_init(CONF_UART, &usart_settings);

	/* Enable the receiver and transmitter. */
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);

	/* map printf to usart */
	ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

	/* ativando interrupcao */
	usart_enable_interrupt(USART1, US_IER_RXRDY);
	NVIC_SetPriority(ID_USART1, 4);
	NVIC_EnableIRQ(ID_USART1);
}


uint32_t usart1_puts(uint8_t *pstring){
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART1))
	usart_serial_putchar(USART1, *(pstring+i++));
}





void USART1_Handler(void){
	uint32_t ret = usart_get_status(USART1);

	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	uartData character;          /* touch queue data type*/
	char c;

	// Verifica por qual motivo entrou na interrup�cao?
	// RXRDY ou TXRDY

	//  Dados dispon�vel para leitura
	if(ret & US_IER_RXRDY){
		usart_serial_getchar(USART1, &c);
		printf("%c", c);
		character.a = c;
		xQueueSendFromISR( xQueueChar, &character, 0);

		// -  Transmissoa finalizada
		} else if(ret & US_IER_TXRDY){

	}
}






static void task_uartRX(void *pvParameters){
	uartData character;
	commandData teste;
	xQueueChar = xQueueCreate( 10, sizeof( uartData ) );
	int n = 0;
	while (true) {
		if (xQueueReceive( xQueueChar, &(character), ( TickType_t )  100 / portTICK_PERIOD_MS)) {
			if (character.a == '\n'){
				teste.command[n] = NULL;
				n = 0;
				//printf("ALLARMMMMM");
				xQueueSendFromISR( xQueueCommand, &teste, 0);
			}
			else{
				printf("task_uartRX");
				printf("%c", character.a);				
				teste.command[n] = character.a;
				n += 1;
			}
		}		
	}
}

static void task_execute(void *pvParameters){
	xQueueCommand = xQueueCreate( 10, sizeof( commandData ) );
	commandData teste;
	while(true){
		if (xQueueReceive( xQueueCommand, &(teste), ( TickType_t )  100 / portTICK_PERIOD_MS)) {
			printf("%d", strcmp(teste.command, "toggle3"));
			printf("%s","toggle1");
			printf("%s","------");
			printf("%s",teste.command);
			
			
			if (!strcmp(teste.command, "toggle1")){
				xSemaphoreGiveFromISR(xSemaphore1, 4);
	
			}else if (!strcmp(teste.command, "toggle3")){
				xSemaphoreGiveFromISR(xSemaphore3, 4);
			}
		}
	}
}



/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	
	pmc_enable_periph_clk(LED1_OLED_PIO_ID);
	pio_set_output(LED1_OLED_PIO , LED1_OLED_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(LED2_OLED_PIO_ID);
	pio_set_output(LED2_OLED_PIO , LED2_OLED_IDX_MASK, 0, 0, 0);

	pmc_enable_periph_clk(LED3_OLED_PIO_ID);
	pio_set_output(LED3_OLED_PIO , LED3_OLED_IDX_MASK, 0, 0, 0);

	/* Initialize the console uart */
	USART1_init();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL,
	TASK_LED1_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(task_led2, "Led2", TASK_LED2_STACK_SIZE, NULL,
	TASK_LED2_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(task_led3, "Led3", TASK_LED3_STACK_SIZE, NULL,
	TASK_LED3_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(task_execute, "task_execute", TASK_EXECUTE_STACK_SIZE, NULL,
	TASK_EXECUTE_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(task_uartRX, "task_uartRX", TASK_EXECUTE_STACK_SIZE, NULL,
	TASK_EXECUTE_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
