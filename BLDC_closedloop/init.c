/*
 * init.c
 *
 *  Created on: 1 oct. 2021
 *      Author: Arnau Quintana
 */
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "init.h"

void Clock_vInit(void)  //Configuración en pag 320 del 'family's guide'
{
   PJSEL0 |= BIT0 + BIT1; //activando pines de entradas LFXT de 32kHz
   //Ajustes para utilizar el DCO a 12MHz:
    CS->KEY = 0x695A; //valor según datasheet para poder desbloquear los registros del CS
    CS->CTL0 = CS_CTL0_DCORSEL_3; // Seleccionamos rango de DCO 8MHz-16MHz
   // sin ningun otro ajuste, se queda en la frec. nominal central de 12MHz.

    CS->CTL1 = CS_CTL1_SELS__DCOCLK |CS_CTL1_DIVS__1 | CS_CTL1_SELM__DCOCLK;
    //SELS -> Fuente de reloj para SMCLK y HSMCLK
    //DIVS -> Divide la frecuencia de la fuente del SMCLK
    //SELM -> Fuente de reloj para el MCLK

    //ACLK queda por defecto a LFXT, que es el cuarzo de 32kHz
    while (!(CS->STAT & (CS_STAT_MCLK_READY | CS_STAT_SMCLK_READY | CS_STAT_ACLK_READY )));
    /*me espero hasta que el registro CSSTAT me indique que todos mis relojes se
    han estabilizado*/

   /*En este punto, tenemos:
     * MCLK = 12 MHz
    * SMCLK = 12 MHz
    * ACLK = LFXTCLK = 32 kHz (el cuarzo)
     */

    // Activar los pines siguientes para comprobar y debugar los relojes -->
    //P4SEL0 |= BIT2|BIT3|BIT4; //activo funciones alternativas primarias MCLK (P4.3),HSMCLK (P4.4) y ACLK (P4.2),
    //para poder observar los relojes con el osciloscopio.
    // <-- Volver a comentar una vez comprobado que los relojes estan bien configurados.
    //Los correspondientes bits en P4SEL1 han de estar a 0, que es su valor por defecto al iniciar el micro
    //P4DIR |=BIT2|BIT3|BIT4; //activo salida MCLK (P4.3), HSMCLK (P4.4) y ACLK (P4.2)
    // <-- Volver a comentar una vez comprobado que los relojes estan bien configurados.
}

void ini_GPIO_control_duty(void) //switches que controlan duty (los integrados en la launchpad)
{
    P1->SEL0 &= ~0x10; //I/O function
    P1->SEL1 &= ~0x10;
    P1->DIR &= ~0x10;
    P1->REN |= 0x10; // habilitamos resistencia pull up/down
    P1->OUT |= 0x10; //resistencia pull up

    P1->IFG = 0; //limpiamos interrupciones pendientes
    P1->IE |= 0x10; //habilitamos interrupcion para el pin P1.4
    P1->IES |= 0x10; //control de interrupcion: high to low.

                    //el botón nos ayuda a hacer un reinicio de la rampa en caso de fallo

    P1DIR |= 0x01; //led que indica que se recibe algo por SPI
    P1OUT &= 0xFE;

    P3->SEL0 &= 0xFE; //pin que indica que se puede empezar la rampa
    P3->SEL1 &= 0xFE;
    P3->DIR &= 0XFE;
}

void init_timerA0(void) //Timer que genera las salidas PWM
{
    TIMER_A0->CCR[0] = 1000-1; //La onda a la salida del timer será de 12kHz

    TIMER_A0->CCR[1] = 250;   //Lo configuramos para comenzar con un duty del 24%
    TIMER_A0->CCTL[1] = 0x0000; //Inicializamos en MODO = 0 (OUT_TIMER = bit 2 del registro CCTL)

    TIMER_A0->CCR[3] = 250;   //el PWM trabajará desde el 18% hasta el 35% de duty cycle.
    TIMER_A0->CCTL[3] = 0x000;

    TIMER_A0->CCR[4] = 250;
    TIMER_A0->CCTL[4] = 0x0000;

    TIMER_A0->CTL = 0x0214; // SMCLK, count up, ID = /1, clear TA0R register.
}

void init_timerA1(void) //timer que controla los cambios de fase
{
    TIMER_A1->CTL = 0x02D4; //SMCLK (12MHz), ID = /8 (ClkTimer = 1.5MHz), UP MODE, TACLR.
    TIMER_A1->CCR[0]= 20000-1; //interrupción cada 8ms (empezamos por un periodo corto para la rampa de activación)
    TIMER_A1->CCTL[0] = CCIE;//activemos la interrupción del bloque

    P2DIR |= 0x01; //LED que se encenderá cuando empiece el control en lazo cerrado
    P2OUT &= 0x00;
}

void ini_GPIO_map(void)
{
    //Salida del bloque comparador 1, 3 y 4 del timer 0 (Puerto 2 pin 4, 6 y 7)
    P2->SEL0 |= 0xD0;  //Función de timer SEL1SEL0 = 01.
    P2->SEL1 &= ~0xD0;
    P2->DIR |= 0xD0;   //Son salidas (por tanto = 1)

    //Configuramos los pines del puerto 7 (pines a los que mapearemos las salidas de timers y los GPIOs)
    P7->SEL0 |= 0xA4;  //Las salidas de timers -> SEL1SEL0 = 11; salidas GPIO -> SEL1SEL0 = 00
    P7->SEL1 &= ~0xA4; //Del puerto 7, los pines 2,5 y 7 salidas de timers. Los pines 1,4 y 6 son GPIOs.
    P7->DIR |= 0xF6;   //Tanto salidas de timers como GPIOs son salidas
    P7 -> OUT &= ~0x52; //nos aseguramos que los GPIOs inicializan con salida = 0

    //Mapping de las salidas de los timers a los pines del conector J5
    PMAP->KEYID=0x2D52;        //contraseña: desbloquea el controlador del port mapping
    P7MAP->PMAP_REGISTER2= 20; //P7.2 mapeado a la salida del bloque 1 del timer
    P7MAP->PMAP_REGISTER5= 22; //P7.5 mapeado a la salida del bloque 3 del timer
    P7MAP->PMAP_REGISTER7= 23; //P7.7 mapeado a la salida del bloque 4 del timer
                               //datasheet pag 124 para saber el num asociado a la salida del timer
    PMAP->CTL=1; //bloquea el controlador del port mapping para evitar modificaciones accidentales
    PMAP->KEYID=0;
}

void init_interrupciones()
{
    NVIC_EnableIRQ(TA1_0_IRQn); //habilitación de interrupciones TIMER_A1 en NVIC..
    //NVIC_EnableIRQ(EUSCIB0_IRQn); //habilitación de interrupciones EUSCIB0 en NVIC.
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31); //habilitación de interrupciones ADC en NVIC.
    NVIC_EnableIRQ(PORT1_IRQn);
}

void init_SPI()
{
    EUSCI_B0->CTLW0 = UCSWRST;
    /*
     * Disable module while configuration
     */
    EUSCI_B0->CTLW0 |= UCMSB | UCSYNC;
    /*
     * Data is changed on the first UCLK edge and captured on the following edge (UCCKPH = 0)
     * Polarity: Inactive state is low (UCCKPL = 0)
     * MSB first (UCMSB = 1)
     * 8bit data (UC7BIT = 0)
     * Slave mode (UCMST = 0)
     * 3-pin SPI (UCMODEx = 0)
     * Synchronous mode (UCSYN = 1)
     * UCxCLK used in slave mode (UCSSEL = 0)
     * STE pin is used to prevent conflicts with others masters (we don't use this pin) (UCSTEM = 0)
     * Software reset disabled (UCSWRST = 0)
     */
    //EUSCI_B0->BRW = 12; //Bit clock prescaler
    /*
     * Clock prescaler -> 12MHz / 12 = 1MHz.
     */

    EUSCI_B0->CTLW0 &= ~UCSWRST;
    /*
     * Enable module after configuration
     */
    EUSCI_B0-> IE |= 1; //Receive interrupt enabled

    P1->SEL0 |= 0x60;         /* P1.5, P1.6 for UCB0CLK, UCB0SIMO */
    P1->SEL1 &= ~0x60;
}

void init_ADC() //configuración del ADC
{
    volatile unsigned int i;

    // GPIO Setup
    P8->SEL1 |= BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7; // Configure P8.2 to P8.7 for ADC A1
    P8->SEL0 |= BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;

    ADC14 -> CTL0 |= ADC14_CTL0_ON;
    /*
     * Activamos el ADC
     */
    ADC14->CTL0 |= ADC14_CTL0_MSC | ADC14_CTL0_SHT0_0 | ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_3 | ADC14_CTL0_SSEL_0;
    /* El primer flanco de subida de la señal SHI triggeriza el timer y luego las conversiones son automáticas
     * Número de ciclos de reloj para cada periodo de muestreo (4 en nuestro caso)
     * Fuente de la señal de muestreo, en nuestro caso usamos el timer interno
     * Modo conversión repetida de una secuencia de canales
     * Fuente de la senyal de reloj (MODCLK en nuestro caso)
     */
    ADC14->CTL1 = ADC14_CTL1_RES_3;
    /*14 bits de resolución
     */
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_0 | ADC14_MCTLN_WINC | ADC14_MCTLN_DIF | ADC14_MCTLN_INCH_18;
    /* Seleccionamos la referencia interna de AVCC y AVSS
     * Activamos ventana de comparación
     * Modo diferencial
     * Indicamos el primer canal de la secuencia -> Valor del canal 18 se restará con el del 19.
     */
    ADC14->MCTL[1] = ADC14_MCTLN_VRSEL_0 | ADC14_MCTLN_WINC | ADC14_MCTLN_DIF | ADC14_MCTLN_INCH_20;
    ADC14->MCTL[2] = ADC14_MCTLN_VRSEL_0 | ADC14_MCTLN_WINC | ADC14_MCTLN_DIF | ADC14_MCTLN_INCH_22 | ADC14_MCTLN_EOS;
    /*
     * Misma configuracion que las anteriores pero esta vez indicamos que es el último canal a convertir con '_EOS'.
     */
    ADC14->HI0 = High_Threshold;
    ADC14->LO0 = Low_Threshold;
    ADC14->HI1 = High_Threshold_offset; //Una vez controle el ADC, la senyal de B y C sufre un pequeño offset
    ADC14->LO1 = Low_Threshold_offset;
    /*
     * Valores alto y bajo de threshold
     */
    ADC14->IER1 = ADC14_IER1_HIIE | ADC14_IER1_LOIE;
    /*
     * Habilitamos las interrupciones
     */
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
    /*
     * Habilitamos la conversión del ADC
     * Empezamos las conversiones.
     */
}
