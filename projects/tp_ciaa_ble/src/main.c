/* Copyright 2016, Ian Olivieri.
 * Copyright 2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Date: 2016-04-26
 */

/* Diagrama de conexion ESP8266
  ┌--------------------------------┐	
  |  |       █████                 |
  |  └-┐     █████      RX OO VCC  | 
  |  ┌-┘ |           GPIO0 OO RST  |  
  |  └-┐ |   █████   GPIO2 OO CH_PD|  
  |  ┌-┘ |   █████     GND OO TX   |  
  |  └---┴                         |
  └--------------------------------┘

   VCC ESP8266 <--> +3.3V EDU-CIAA-NXP
   RST ESP8266 <--> (SIN CONEXION)
 CH_PD ESP8266 <--> +3.3V EDU-CIAA-NXP
    TX ESP8266 <--> RX EDU-CIAA-NXP

    RX ESP8266 <--> TX EDU-CIAA-NXP
 GPIO0 ESP8266 <--> (SIN CONEXION)
 GPIO0 ESP8266 <--> (SIN CONEXION)
   GND ESP8266 <--> GND EDU-CIAA-NXP

  AT commands: http://www.pridopia.co.uk/pi-doc/ESP8266ATCommandsSet.pdf
*/

/*==================[inclusions]=============================================*/

#include "../inc/main.h"   /* <= own header */
#include "sapi.h"                 /* <= sAPI header */

/*==================[macros and definitions]=================================*/

#define BAUD_RATE 115200 // Baudrate por defecto del ESP8266

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

//Constante para definir si el anti rebote pasó
const uint8_t BTN_SHOULD_CLICK = 10;
const uint8_t MSG_FTO[] = "Introduzca el valor de entrada en el formato :XY y presione Enter\r\n";

/** @brief used for delay counter */
static uint32_t pausems_count;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void imprimirMensajeDeBienvenida( uartMap_t uart ){

   /* Imprimo el mensaje de bienvenida */
   uartWriteString( uart,
      "Bievenido al asistente de configuracion del modulo ESP8266\r\n" );
   uartWriteString( uart,
      "Antes de continuar, por favor asegurese que su terminal\r\n" );
   uartWriteString( uart,
      "serie dispone del terminador CR+LF (enter)\r\n\r\n" );
   uartWriteString( uart,
      "A continuacion se realiza un listado de algunos de los\r\n" );
   uartWriteString( uart, "comandos AT disponibles:\r\n\r\n" );
   uartWriteString( uart,
      "> Saber si el modulo responde correctamente: AT\r\n" );
   uartWriteString( uart,
      "> Version del Firmware: AT+GMR\r\n" );
   uartWriteString( uart, "> Resetear el modulo: AT+RST\r\n" );
   uartWriteString( uart,
      "> Listar todas las redes disponibles: AT+CWLAP\r\n" );
   uartWriteString( uart,
      "> Checkear la red actual: AT+CWJAP?\r\n" );
   uartWriteString( uart,
      "> Unirse a una red: AT+CWJAP=\"nombreRedInalambrica\",\"password\"\r\n" );
   uartWriteString( uart,
      "  - NOTA: Las comillas dobles son parte del mensaje\r\n" );
   uartWriteString( uart,
      "> Salir de la red: AT+CWQAP\r\n" );

   delay(100);
}

void init_hw_PWM(void){
	Chip_SCTPWM_Init(LPC_SCT);

	Chip_SCTPWM_SetRate(LPC_SCT, 10000);

	Chip_SCU_PinMux(2,10,0,FUNC1);
	Chip_SCU_PinMux(2,11,0,FUNC1);
	Chip_SCU_PinMux(2,12,0,FUNC1);

	Chip_SCTPWM_SetOutPin(LPC_SCT, 1, 2);
	Chip_SCTPWM_SetOutPin(LPC_SCT, 2, 5);
	Chip_SCTPWM_SetOutPin(LPC_SCT, 3, 4);

	Chip_SCTPWM_Start(LPC_SCT);
}

uint8_t boton_salida(uint8_t current_duty1, uint8_t num_boton){
	uint8_t nuevo_duty = (uint8_t) botones[num_boton].funcion_btn(current_duty1);

	//convierto el duty hexadecimal en un porcentaje
	uint8_t percentage = (nuevo_duty * (uint8_t)0x64)/((uint8_t)0xff);
	//funcion de interpolacion lineal
	uint16_t interpolado = interpolar_num(nuevo_duty);
	//convierto el duty interpolado en un porcentaje
	uint8_t percent_interpolado = ((uint8_t)interpolado * (uint8_t)0x64)/((uint8_t)0xff);

	//Entrada por LED 2
	Chip_SCTPWM_SetDutyCycle(LPC_SCT, 2, Chip_SCTPWM_PercentageToTicks(LPC_SCT, percentage));
	//Salida por LED 3
	Chip_SCTPWM_SetDutyCycle(LPC_SCT, 3, Chip_SCTPWM_PercentageToTicks(LPC_SCT, percent_interpolado));

	/* Devuelvo el valor de duty para poder guardarlo */
	return nuevo_duty;
}

void pausems(uint32_t t)
{
   pausems_count = t;
   while(pausems_count != 0) {
      __WFI();
   }
}

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){

   /* ------------- INICIALIZACIONES ------------- */

   /* Inicializar la placa */
   boardConfig();

   /* Inicializar el conteo de Ticks con resolución de 1ms, sin tickHook */
   tickConfig( 1, 0 );

   /* Inicializar GPIOs */
   gpioConfig( 0, GPIO_ENABLE );

   /* Configuración de pines de entrada para Teclas de la CIAA-NXP */
   gpioConfig( TEC1, GPIO_INPUT_PULLDOWN );
   gpioConfig( TEC2, GPIO_INPUT_PULLDOWN );
   gpioConfig( TEC3, GPIO_INPUT_PULLDOWN );
   gpioConfig( TEC4, GPIO_INPUT_PULLDOWN );

   /* Configuración de pines de salida para Leds de la CIAA-NXP */
   gpioConfig( LEDR, GPIO_OUTPUT );
   gpioConfig( LEDG, GPIO_OUTPUT );
   gpioConfig( LEDB, GPIO_OUTPUT );
   gpioConfig( LED1, GPIO_OUTPUT );
   gpioConfig( LED2, GPIO_OUTPUT );
   gpioConfig( LED3, GPIO_OUTPUT );

   /* Inicializar las UART a 115200 baudios */
   uartConfig( UART_USB, BAUD_RATE );
   uartConfig( UART_232, 9600 );
   uartWriteString(UART_232, "AT\r\n");
   uartWriteString(UART_232, "AT+BAUD0\r\n");
   uartWriteString(UART_232, "AT+ROLE0\r\n");
   
   /* Inicializar PWM */
   init_hw_PWM();

   volatile int antiRebAcum = 0;

   uint8_t datoUSB  = 0;
   uint8_t datoRS232 = 0;
   uint8_t salidaUART;
   uint8_t percentageUART;
   uint8_t hexValue;
   uint32_t BUTTON_STATUS_POLLED = 0x00;
   uint8_t current_duty = 0x00;

   InputDTO maquina = {.state=IDLE_MSG, .data={[0]='0', [1]='0', [2]='\0'}};

   /* ------------- REPETIR POR SIEMPRE ------------- */
   while(1) {

      if( uartReadByte( UART_USB, &datoUSB )){
    	  uartWriteByte(UART_USB, datoUSB);
        transicionar_dto(&maquina, datoUSB);
      }else if (uartReadByte( UART_232, &datoRS232 ) ){
        transicionar_dto(&maquina, datoRS232);
      }

      //Si la maquina procesó una palabra, se hace la interpolacion lineal y se calculan los porcentajes
      if(maquina.state == END_MSG){
        hexValue = string_to_8bit_hex(maquina.data);
        percentageUART = ( hexValue * (uint8_t)0x64)/((uint8_t)0xff);
        salidaUART = (interpolar_string(maquina.data) * (uint8_t)0x64)/((uint8_t)0xff);
        //cambiar intensidad del LED 0 con la entrada
        //LED_Duty_change(maquina.data, LED0);
        Chip_SCTPWM_SetDutyCycle(LPC_SCT, 2, Chip_SCTPWM_PercentageToTicks(LPC_SCT, percentageUART));
        //cambiar intensidad del LED 1 con la salida
        //LED_Duty_change(salidaUART, LED1);
        Chip_SCTPWM_SetDutyCycle(LPC_SCT, 3, Chip_SCTPWM_PercentageToTicks(LPC_SCT, salidaUART));
        maquina.state = IDLE_MSG;
      }

      /* Si presionan TEC1 muestro el mensaje de bienvenida */
      if( !gpioRead( TEC1 ) ){
    	  gpioWrite( LEDB, ON );
    	  imprimirMensajeDeBienvenida(UART_USB);
    	  imprimirMensajeDeBienvenida(UART_232);
    	  current_duty = boton_salida(current_duty, TEC1 - TEC1);
    	  gpioWrite( LEDB, OFF );
      } else if (!gpioRead(TEC2)) {
    	  current_duty = boton_salida(current_duty, TEC2 - TEC1);
//    	  while (!gpioRead(TEC2)){
//    		  current_duty = boton_salida(current_duty, TEC2 - TEC1);
//    	  }
      } else if (!gpioRead(TEC3)) {
    	  current_duty = boton_salida(current_duty, TEC3 - TEC1);
//    	  while (!gpioRead(TEC3)){
//    		  current_duty = boton_salida(current_duty, TEC3 - TEC1);
//    	  }
      } else if (!gpioRead(TEC4)) {
    	  current_duty = boton_salida(current_duty, TEC4 - TEC1);
//    	  while (!gpioRead(TEC4)){
//    		  current_duty = boton_salida(current_duty, TEC4 - TEC1);
//    	  }
      }


   }

   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
   return 0 ;
}

/*==================[end of file]============================================*/
