/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMERO_SECCIONES 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


uint8_t letra;
uint8_t num_letra = 0;
char orden[10];//LONGITUD DEL ARRAY 10


unsigned char dist_min = 5;
unsigned char dist_max = 30;
uint32_t tiempo_limite = 10000;


enum msg_index{
	ahead,right,left,back,stop,min_update,max_update,invalid,auto_mode,manual_mode,
	manual_not_set,manual_already_set,auto_already_set
};
char modo_manual;
enum msg_index mode;
char* msgs[] = {
		"hacia delante\n""hacia la derecha\n","hacia la izquierda\n","hacia atras\n","parar\n",
		"distancia min actualizada\n","distancia max actualizada\n","comando no válido","modo auto :ON\n",
		"modo manual: ON\n","modo manual no establecido :(\n","modo manual establecido\n","modo automático establecido\n"
};

uint32_t valor_adc;
unsigned int veces_timer;
unsigned char leer;

enum modos_coche {AVANZAR, BUSCAR_SALIDA};
enum modos_coche modo;
char paso_busqueda_salida;
unsigned int tiempo_giro = 1100;
unsigned int veces_giro;


enum sentidos_marcha {ADELANTE, ATRAS};
enum sentidos_marcha sentido = ADELANTE;


enum estados{EMITIENDO, LEYENDO};
char estado, estado_prev;
unsigned int delay_trasero, delay_trasero_prev;
unsigned int delay_delantero, delay_delantero_prev;
unsigned int valor_delay_min, valor_delay_max;
char recibiendo;
enum estados_sonido{NO_SONAR, SONIDO_CONT, SONAR_INTERMITENTE};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void send_msg(enum msg_index i);
enum msg_index ejecutar_orden(char* orden);
void parar(void);

void girar_der(unsigned char velocidad);
void girar_izq(unsigned char velocidad);

void retroceder(unsigned char velocidad);
void avanzar(unsigned char velocidad);
void retroceder_der(unsigned char velocidad);
void avanzar_der(unsigned char velocidad);
void retroceder_izq(unsigned char velocidad);
void avanzar_izq(unsigned char velocidad);



void salir_modo_busqueda();
void TIM3_IRQHandler(void);

void trigger_delantero_OFF();
void trigger_delantero_ON();
void trigger_trasero_OFF();
void trigger_trasero_ON();

void sonido_continuo();
void no_sonar();
void sonido_on();
void sonar_intermitente();

void TIM4_IRQHandler(void);
void EXTI0_IRQHandler(void);
void ADC1_IRQHandler(void);
void invertir_sentido_marcha();


void set_rangos(uint16_t valores[NUMERO_SECCIONES +1]);
uint16_t get_posicion(uint16_t valores[NUMERO_SECCIONES +1], uint16_t valor);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//array para guardar los valores frontera entre secciones del rango del potenciómetro
//son 12 bits(valor max= 0x0FFF)
void set_rangos(uint16_t valores[NUMERO_SECCIONES +1]){

  uint16_t valor_max = 0x0fff; //los 12 bits a 1
  uint16_t step = valor_max / NUMERO_SECCIONES ;

  uint16_t valor = 0;
  for(uint16_t  i = 0; i<NUMERO_SECCIONES ; i++){
    valores[i] = valor;
    valor += step;
  }
  valores[NUMERO_SECCIONES ] = valor_max+1;
}


/*dado el array de secciones y un valor del ADC devuelve
en que sección se encuentra el potenciometro*/
uint16_t get_posicion(uint16_t valores[NUMERO_SECCIONES +1], uint16_t valor){

  for(uint16_t i = 0; i<NUMERO_SECCIONES ; i++){
    if(valores[i]<= valor && valor <=valores[i+1])
      return i;
  }
  return -1;
}

//interrupcion ADC
void ADC1_IRQHandler(void){
  if((ADC1->SR & (1 << 1))  != 0){ //EOC
    valor_adc = ADC1->DR;
  }

}



//interrupcion PA0
void EXTI0_IRQHandler(void){
	if((EXTI -> PR & (1<<0)) != 0){

		invertir_sentido_marcha();

		EXTI -> PR |= (1<<0);
	}
}


//Interrupcion TIM4
void TIM4_IRQHandler(void){

	if((TIM4->SR & (1<<1)) != 0){// ¿Se activó la interrupción en CCR1?
		if(estado == EMITIENDO){
			estado = LEYENDO;
			TIM4->CCR1 += 50000;// Configura el siguiente evento en 50 ms
		}
		else{
			estado = EMITIENDO;
			TIM4->CCR1 += 10;// Configura el siguiente evento en 10 us
		}
		TIM4->SR &= ~(1<<1);// Limpia el flag de interrupción
	}
	if((TIM4->SR & (1<<2)) != 0){// ¿Se activó la interrupción en CCR2?
		TIM4->CCR2 += 1000; // Configura el siguiente evento en 1 ms
		veces_timer++;// Incrementa el contador de tiempo
		if(modo == BUSCAR_SALIDA)
			veces_giro++;//Si está buscando salida, cuenta giros

		if(veces_timer == 2000){
			leer = 1;
			veces_timer = 0;

		}
		if(veces_giro == tiempo_giro && modo == BUSCAR_SALIDA){
			veces_giro = 0;
			paso_busqueda_salida++;
		}

		TIM4->SR &= ~(1<<2);// Limpia el flag de interrupción
	}
	if(((TIM4 -> SR) & (1<<3)) != 0){ // ¿Se activó la interrupción en CCR3?
		TIM4 -> CCR3 += 50000;// Configura el siguiente evento en 50 ms
		TIM4->SR &= ~(1<<3);//limpiar flag de la interrupcion
	}
}

void sonar_intermitente(){
	TIM4->CCMR2 &= ~(1<<6);
	TIM4->CCMR2 |=  (1<<5);
	TIM4->CCMR2 |=  (1<<4);
	sonido_on();
}
void sonido_on(){
	TIM4 -> CCER |= (1<<8);//TIM4_CH3 HW ON
}

void no_sonar(){
	TIM4 -> CCER &= ~(1<<8);//TIM4_CH3 HW OFF
}

void sonido_continuo(){
	TIM4->CCMR2 |=  (1<<6); //low 100
	TIM4->CCMR2 &= ~(1<<5);
	TIM4->CCMR2 &= ~(1<<4);
	sonido_on();
}

//Trigger delantero PC8
void trigger_trasero_ON(){
	GPIOC->BSRR |= (1<<8);
}

void trigger_trasero_OFF(){
	GPIOC->BSRR |= (1<<(8+16));

}


//Trigger delantero PC6
void trigger_delantero_ON(){
	GPIOC->BSRR |= (1<<6);

}

void trigger_delantero_OFF(){
	GPIOC->BSRR |= (1<<(6+16));

}




//Para medir el echo: TIM3_CH4 (PC9)
void TIM3_IRQHandler(void){
  if( (TIM3 -> SR & (1<<4)) != 0){//trasero
	if(recibiendo == 0){
		recibiendo = 1;
		TIM3->CCER = 0x3000;//falling CH4 + CC4E a 1
		delay_trasero_prev = TIM3->CCR4;
	}
	else{
		recibiendo = 0;
		TIM3->CCER = 0x1000;//rising CH2 + CC2E a 1
		delay_trasero = TIM3->CCR4 - delay_trasero_prev;
	}
	TIM3->SR &= ~(1<<4);
  }
	  if( (TIM3 -> SR & (1<<2)) != 0){//delantero
		if(recibiendo == 0){
			recibiendo = 1;
			TIM3->CCER = 0x0030;//falling CH2 + CC2E a 1
			delay_delantero_prev = TIM3->CCR2;
		}
		else{
			recibiendo = 0;
			TIM3->CCER = 0x0010;//rising CH4 + CC4E a 1
			delay_delantero = TIM3->CCR2 - delay_delantero_prev;
		}
		TIM3->SR &= ~(1<<2);
    }

}




//RUEDAS
void avanzar_izq(unsigned char velocidad){
	GPIOB->BSRR |=  (1<<(12+16));
	TIM2->CCR3 = velocidad;
}


void retroceder_izq(unsigned char velocidad){
	GPIOB->BSRR |=  (1<<12);
	TIM2->CCR3 = 100 - velocidad;
}

void avanzar_der(unsigned char velocidad){
	GPIOB->BSRR |=  (1<<(13+16));
	TIM2->CCR4 = velocidad;
}


void retroceder_der(unsigned char velocidad){
	GPIOB->BSRR |=  (1<<13);
	TIM2->CCR4 = 100 - velocidad;
}


void avanzar(unsigned char velocidad){
	avanzar_izq(velocidad);
	pp_der(velocidad);
}


void retroceder(unsigned char velocidad){
	retroceder_izq(velocidad);
	retroceder_der(velocidad);
}

void girar_izq(unsigned char velocidad){
	avanzar_der(velocidad);
	retroceder_izq(velocidad);
}

void girar_der(unsigned char velocidad){
	avanzar_izq(velocidad);
	retroceder_der(velocidad);
}

void parar(void){
	avanzar_izq(1);
	avanzar_der(1);
}


void salir_modo_busqueda(){
	modo = AVANZAR;
	paso_busqueda_salida = 0;
	veces_giro = 0;
	delay_trasero = -1;
	delay_delantero = -1;
}

void invertir_sentido_marcha(){
	if(sentido == ADELANTE){
		  sentido = ATRAS;
		  TIM3->CCER = 0x1000;//rising CH4 + CC4E a 1
	  }
	  else{
		  sentido = ADELANTE;
		  TIM3->CCER = 0x0010;//rising CH2 + CC2E a 1
	  }
}



void reset_auto_mode(){
	parar();// Detiene cualquier movimiento en curso
	modo_manual = 0; // Desactiva el modo manual

	// Reinicia el array de órdenes
	for(int i = 0; i<10; i++)//Longitud del array char orden[10]
			orden[i] = '\0';

	// Configuración inicial del TIM4
	TIM4->CR1 = 0;//apaga el temp
	TIM4->CNT = 0;//reinicia el contador
	TIM4->CCR2 = 1000;
	TIM4->CCR1 = 10;
	TIM4 -> CCR3 = 50000;
	TIM4->CR1 |= (1<<0);//enciende el temp
	TIM4->SR = 0;//limpia flags

	// Configuración inicial del TIM3
	TIM3->CR1 = 0;//apaga el temp
	TIM3->CNT = 0;//reinicia el contador
	TIM3->CCER = 0x0010;// rising del CH2, FRONT
	TIM3->CR1 |= 0x0001; //ON
	TIM3->SR  = 0; //flags bajados

	recibiendo = 0;

	delay_trasero = -1;
	delay_trasero_prev = 0;

	delay_delantero = -1;
	delay_delantero_prev = 0;

	estado_prev = -1;
	estado = EMITIENDO;

	veces_timer = 0;
	leer = 0;


	modo = AVANZAR;
	paso_busqueda_salida = 0;
	veces_giro = 0;

}


// Función que ejecuta las órdenes recibidas por el sistema
enum msg_index ejecutar_orden(char* orden){

	// Cambio a modo manual
	if(strcmp(orden, "MODE_MANUAL") == 0){
		if(modo_manual == 1)
			return manual_already_set;
		parar();
		no_sonar();
		modo_manual = 1;
		return manual_mode;
	}

	  // Cambio a modo automático
	if(strcmp(orden, "MODE_AUTO") == 0 ){
		if(modo_manual == 0)
			return auto_already_set;


		reset_auto_mode();
		return auto_mode;
	}


	if(modo_manual == 0)
		return manual_not_set;

	if(strncmp(orden, "MIN", 3) == 0){//compara los primeros 3 caracteres de orden con "MIN"
		char num[4];//almacenar los valores numéricos después de "MIN",inicializa con '\0'
		for(int i = 0; i<4;i++)
			num[i] = '\0';//asegurarse que no hay nada dentro
		strcpy(num, orden+3);//Copia los caracteres desde orden+3 (después de "MIN") hasta num
		dist_min = strtol(num, NULL, 10);//convierte el MIN a entero
		if(dist_min<5 || dist_min > dist_max)//Verifica que la distancia mínima esté en un rango válido
			return invalid;
		valor_delay_min = 2*dist_min /0.034;
		return min_update;
	}
	if(strncmp(orden, "MAX", 3) == 0){
		char num[4];
		for(int i = 0; i<4;i++)
			num[i] = '\0';
		strcpy(num, orden+3);
		dist_max = strtol(num, NULL, 10);//convierte el MAX a entero
		if(dist_max>30 || dist_min > dist_max)//Verifica que la distancia máxima esté en un rango válido
					return invalid;
		valor_delay_max = 2*dist_max /0.034;

		return max_update;
	}
	//comandos desde el móvil
	if(strcmp(orden, "GO")==0 ){
		avanzar(99);
		return ahead;
	}
	if(strcmp(orden, "BACK")==0 ){
		retroceder(99);
		return back;
	}
	if(strcmp(orden, "IZQ")==0 ){
		girar_izq(99);
		return left;
	}
	if(strcmp(orden, "DER")==0 ){
		girar_der(99);
		return right;
	}
	if(strcmp(orden, "STP")==0 ){
		parar();
		return stop;
	}
	return invalid;
}



// Función que envía mensajes por UART
void send_msg(enum msg_index i){
//casting para que no salte el aviso
    //envia un mensaje a traves de la UART

	//PARAMETROS DE HAL_UART_Transmit()
	//&huart1 -> Identificador de la interfaz UART
	//(unsigned char*)msgs[i] -> Mensaje a enviar
	//strlen(msgs[i]) -> Longitud del mensaje en bytes
	//tiempo_limite -> Tiempo de espera
	HAL_UART_Transmit(&huart1, (unsigned char*)msgs[i], strlen(msgs[i]) , tiempo_limite);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){//Se ejecuta al recibir un dato por UART
	if(letra == '\n'){//si el byte recibido es un salto de linea la orden esta completa
	  orden[num_letra-1] = '\0'; //fin de la cadena
	  mode = ejecutar_orden(orden);//ejecuta la orden almacenada en el buffer
	  send_msg(mode);//rnvia el mensaje de respuesta

	 for(int i = 0; i<10; i++)//10 ->longitud del array char orden[10]
		orden[i] = '\0'; //resetea el buffer a \0s
	 num_letra = 0;

	}
	else{
	  orden[num_letra] = letra; // Almacena la letra recibida en el buffer
	  num_letra++;
	}
	HAL_UART_Receive_IT(&huart1, &letra, 1);  // Vuelve a habilitar la recepción por interrupción

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */



  	//PA0 digital input
  	GPIOA -> MODER &= ~(1 << 0);//0
  	GPIOA -> MODER &= ~(1 << (0+1));// 0

  	//configuración interrupción PA0
  	SYSCFG -> EXTICR[0] &= ~(1 << 0);
  	SYSCFG -> EXTICR[0] &= ~(1 << 1);
  	SYSCFG -> EXTICR[0] &= ~(1 << 2);
  	SYSCFG -> EXTICR[0] &= ~(1 << 3);

  	EXTI -> IMR |= 1;
  	EXTI -> RTSR |= 1; // rising
  	NVIC -> ISER[0] |= 1 << 6;


  //PB8 función alternativa(buzzer)
  	GPIOB -> MODER |= (1 << (8*2 + 1));//1
  	GPIOB -> MODER &= ~(1 << (8*2)); //0

  	GPIOB -> AFR[1] = (2 << 0); //f. alternativa 1 TIM4 en PB8



  	//PC8 digital output (echo)
  	GPIOC -> MODER &= ~(1 << (8*2 + 1));//0
  	GPIOC -> MODER |=  (1 << 8*2); //1

  	//PC6 digital output (echo)
  	GPIOC -> MODER &= ~(1 << (6*2 + 1)); //0
  	GPIOC -> MODER |=  (1 << (6*2)); // 1



  	//PC9 f. alternativa TIM3 (echo)
  	GPIOC->MODER |=  (1<<(9*2+1)); // 1
  	GPIOC->MODER &= ~(1<<(9*2));//0

  	GPIOC->AFR[1] &=  ~(1<<(1*4+3)); //TIM3 0010
  	GPIOC->AFR[1] &=  ~(1<<(1*4+2));
  	GPIOC->AFR[1] |=   (1<<(1*4+1));
  	GPIOC->AFR[1] &=  ~(1<<(1*4));

  	//PC7 f. alternativa TIM3 (echo)
  	GPIOC->MODER |=  (1<<(7*2+1)); //1
  	GPIOC->MODER &= ~(1<<(7*2));//0

  	GPIOC->AFR[0] &=  ~(1<<(7*4+3)); // TIM3 0010
  	GPIOC->AFR[0] &=  ~(1<<(7*4+2));
  	GPIOC->AFR[0] |=   (1<<(7*4+1));
  	GPIOC->AFR[0] &=  ~(1<<(7*4));


  	//PA2 en analógico
  	GPIOA->MODER |= (1 << (2*2+1));
  	GPIOA->MODER |= (1 << (2*2));

  	//configuración ADC
  	ADC1->CR2 = 0;//ADC off
  	ADC1->CR1 = 0x00000020; //12 bits ,sin SCAN,con interrupciones

  	ADC1->CR2 =0x0000412;

  	ADC1->SQR1 = 0x00000000; //1 canal
  	ADC1->SQR5 = 0x00000002; //IN2


  	ADC1->CR2 |= (1<<0); //ADC on
  	while( (ADC1->SR & 0x0040)==0); //esperar a ADC preparado
  	ADC1->CR2 |= 0x40000000;


  	NVIC->ISER[0] |= (1<<18);//para la interrupción


  	//TIM4 ch2 para contar 1ms
  	TIM4->CR1 = 0;
  	TIM4->CR2 = 0;
  	TIM4->SMCR = 0;

  	TIM4->CNT = 0;
  	//con 32 son 1000000 pasos/s (1us)
  	TIM4->PSC = 31;
  	TIM4->ARR = 0xFFFF;
  	TIM4->CCR2 = 1000;
  	TIM4->CCR1 = 10;
  	TIM4 -> CCR3 = 50000;
  	TIM4 -> DIER |= 1 << 3; // enable interrupción CH4

  	TIM4->DIER |= (1<<1);
  	TIM4->DIER |= (1<<2);
  	NVIC->ISER[0] |= (1<<30);

  	TIM4->CCMR1 = 0;
  	TIM4->CCER = 0;

  	TIM4->CR1 |= (1<<0);
  	TIM4->EGR = 0;
  	TIM4->SR = 0;

  	//TIM3CH2 echo
  	//TIM3CH4 echo
  	TIM3->CR1 = 0;
  	TIM3->CR2 = 0;
  	TIM3->SMCR = 0;

  	//con 32 son 1000000 pasos/s(1 us)
  	TIM3->PSC = 32-1;
  	TIM3->CNT = 0; //contador=0
  	TIM3->ARR = 0xffff; //tope de cuenta


  	//interrupción
  	TIM3->DIER |= (1<<2); //  CH2
  	TIM3->DIER |= (1<<4); // CH4

  	TIM3->CCMR1 = 0x0100;
  	TIM3->CCMR2 = 0x0100;

  	TIM3->CCER = 0x0010;

  	//inicializar el timer
  	TIM3->CR1 |= 0x0001;
  	TIM3->EGR |= 0x0001;
  	TIM3->SR  = 0;

  	NVIC->ISER[0] |= (1 << 29); //NVIC para TIM3




	//PB12 en DIGITAL OUTPUT 01
  	GPIOB->MODER &= ~(1<<(12*2+1));
  	GPIOB->MODER |=  (1<<(12*2));

  	//PB10 en función alternativa 10
  	GPIOB->MODER |=  (1<<(10*2+1));
  	GPIOB->MODER &= ~(1<<(10*2));

 	//PB13 en DIGITAL OUTPUT 01
  	GPIOB->MODER &= ~(1<<(13*2+1));
  	GPIOB->MODER |=  (1<<(13*2));

  	//PB11 en función alternativa 10
  	GPIOB->MODER |=  (1<<(11*2+1));
  	GPIOB->MODER &= ~(1<<(11*2));

  	GPIOB->AFR[1] |= 0x00001100;



  	//TIM2_CH3  PWM
  	TIM2->CR1 = 0;
  	TIM2->CR2 = 0;
  	TIM2->SMCR = 0;

  	TIM2->CNT = 0;
  	TIM2->PSC = 32-1;
  	TIM2->ARR = 100-1;
  	TIM2->CCR3 = 1;
  	TIM2->CCR4 = 1;//empezar parado

  	TIM2->DCR = 0;
  	TIM2->DIER = 0x0000;

  	TIM2->CCMR2 = 0;


  	TIM2->CCMR2 |= (1<<3);	//CH3

  	TIM2->CCMR2 |= (1<<6);
  	TIM2->CCMR2 |= (1<<5);
  	TIM2->CCMR2 &= ~(1<<4);


  	TIM2->CCMR2 |= (1<<11);//CH4

  	TIM2->CCMR2 |= (1<<14);
  	TIM2->CCMR2 |= (1<<13);
  	TIM2->CCMR2 &= ~(1<<12);

  	TIM2->CCER |= (1<<8);//CC3E
  	TIM2->CCER |= (1<<12);//CC4E

  	TIM2->CR1 |= (1<<7);
  	TIM2->EGR |= (1<<0);
  	TIM2->CR1 |= (1<<0);
  	TIM2->SR = 0;



  	//variables
	recibiendo = 0;

	delay_trasero = -1;
	delay_trasero_prev = 0;

	delay_delantero = -1;
	delay_delantero_prev = 0;

	estado_prev = -1;
	estado = EMITIENDO;

	uint16_t posicion = 0;
	uint16_t valores[NUMERO_SECCIONES+1];
	set_rangos(valores);

	veces_timer = 0;
	leer = 0;

	enum estados_sonido sonido = NO_SONAR;
	enum estados_sonido sonido_previo = -1;

	char velocidad_max = 1;
	char velocidad = velocidad_max;

	modo = AVANZAR;
	paso_busqueda_salida = 0;
	veces_giro = 0;

	char leer_puntual = 0;


	valor_delay_min = 294;//5cm
	valor_delay_max = 1765;//30cm

  	modo_manual = 1;
	uint8_t* msg = (uint8_t*)"ready!\n";// casting para que no salte un warning
	//buffer para la orden del móvil
	for(int i = 0; i<10; i++)//longitud del array char orden[10]
		orden[i] = '\0';
	//transmisión del mensaje de ready!
	HAL_UART_Transmit(&huart1, msg, strlen((char*)msg) , tiempo_limite);
	HAL_UART_Receive_IT(&huart1, &letra, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


     //sino entra en modo auto,no se hace nada con los temps ni sensores
	  if(modo_manual == 0){


		  //El ADC se lee cada 2s
		  if(leer == 1){
			  leer = 0;
			  posicion = get_posicion(valores, valor_adc);
			  velocidad_max = 50 + posicion*6;
			  if(velocidad_max>99)
				  velocidad_max = 99;
		  }



		//estados sensores
		  if(estado_prev != estado){
			estado_prev = estado;
			switch(estado){
				case EMITIENDO:
					if(sentido == ADELANTE){
						trigger_delantero_ON();
					}
					else {
						trigger_trasero_ON();
					}
					break;
				case LEYENDO:
					if(sentido == ADELANTE){
						trigger_delantero_OFF();
					}
					else{
						trigger_trasero_OFF();
					}
					break;
			}
		  }


		 //estados buzzer
		  if(sonido_previo != sonido){
			  sonido_previo = sonido;
			  switch(sonido){
				  case NO_SONAR:
					  no_sonar();
					  break;
				  case SONIDO_CONT:
					  sonido_continuo();
					  break;
				  case SONAR_INTERMITENTE:
					  sonar_intermitente();
					  break;
				  }
		  }



          //estados para buscar la salida
		  if(modo == BUSCAR_SALIDA){
			  switch(paso_busqueda_salida){
			  case 0:
				  if(sentido == ADELANTE){
					  retroceder_izq(velocidad);
					  avanzar_der(1);
				  }
				  else{
					  avanzar_izq(velocidad);
					  retroceder_der(1);
				  }
				  leer_puntual = 1;
				  break;
			  case 1: //deshace giro
				  if(sentido == ADELANTE){
					  if(delay_delantero > 1176 && leer_puntual == 1){//20cm
						  salir_modo_busqueda();
					  }
					  avanzar_izq(velocidad);
					  avanzar_der(1);
				  }
				  else{
					  if(delay_trasero>1176 && leer_puntual == 1){//20
						  salir_modo_busqueda();
					  }
					  retroceder_izq(velocidad);
					  avanzar_der(1);
				  }
				  leer_puntual = 0;

				  break;
			  case 2:
				  if(sentido == ADELANTE){
					  retroceder_der(velocidad);
					  avanzar_izq(1);
				  }
				  else{
					  avanzar_der(velocidad);
					  avanzar_izq(1);
				  }
				  leer_puntual = 1;
				  break;
			  case 3:
				  if(sentido == ADELANTE){
					  if(delay_delantero >1176 && leer_puntual == 1){//20cm
						  salir_modo_busqueda();
					  }
					  avanzar_der(velocidad);
					  avanzar_izq(1);
				  }
				  else{
					  if(delay_trasero >1176 && leer_puntual == 1){//20cm
						  salir_modo_busqueda();
					  }
					  retroceder_der(velocidad);
					  avanzar_izq(1);
				  }
				  leer_puntual = 0;
				  break;
			  case 4: //invertir sentido
				  invertir_sentido_marcha();
				  salir_modo_busqueda();

				  break;
			  }
			  if(modo == BUSCAR_SALIDA)
				  continue;
		  }




		  //velocidad en funcion de la distancia
		  	  // d = t*0.034/2 ----> t = 2*d/0.034
		  	  //#si d = 30 cm -> t = 1765
		  	  //#si d = 20 cm -> t = 1176
		  	  //#si d = 10 cm -> t = 588
		  	  //#si d = 5 cm -> t = 294

		  if(delay_trasero < valor_delay_min || delay_delantero< valor_delay_min){ //< 5cm.
			  velocidad = 1;
			  sonido = SONIDO_CONT;
		  }
		  else{
			  sonido = SONAR_INTERMITENTE;
			  if(delay_trasero < 588 || delay_delantero< 588){ //10 cm
				  velocidad = 50 + (velocidad_max-50)/8;
				  if(modo == AVANZAR){
					  velocidad = 70; //velocidad de busqueda
					  modo = BUSCAR_SALIDA;
				  }

			  }
			  else{
				  if(delay_trasero < 1176 || delay_delantero< 1176){ //< 20cm
					  velocidad = 50 + (velocidad_max-50)/4;
				  }
				  else{
					  if(delay_trasero < valor_delay_max || delay_delantero< valor_delay_max){ //< 30cm
						  velocidad = 50 + (velocidad_max-50)/2;
					  }
					  else{//no hay objeto delante
						  velocidad = velocidad_max;
						  sonido = NO_SONAR;
					  }
				  }
			  }
		  }
		  if(sentido == ADELANTE)
			  avanzar(velocidad);
		  else
			  retroceder(velocidad);


	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : SEG14_Pin SEG15_Pin SEG16_Pin SEG17_Pin
                           SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin
                           SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG14_Pin|SEG15_Pin|SEG16_Pin|SEG17_Pin
                          |SEG18_Pin|SEG19_Pin|SEG20_Pin|SEG21_Pin
                          |SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin COM0_Pin
                           COM1_Pin COM2_Pin SEG12_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|COM0_Pin
                          |COM1_Pin|COM2_Pin|SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG6_Pin SEG7_Pin SEG8_Pin SEG9_Pin
                           SEG10_Pin SEG11_Pin SEG3_Pin SEG4_Pin
                           SEG5_Pin SEG13_Pin COM3_Pin */
  GPIO_InitStruct.Pin = SEG6_Pin|SEG7_Pin|SEG8_Pin|SEG9_Pin
                          |SEG10_Pin|SEG11_Pin|SEG3_Pin|SEG4_Pin
                          |SEG5_Pin|SEG13_Pin|COM3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
