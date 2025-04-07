/*
 * main.c
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

#define FASE_0  0
#define FASE_1  1
#define FASE_2  2
#define FASE_3  3
#define FASE_4  4
#define FASE_5  5
#define FASE_6  6

uint8_t FASE = 1; //indicará la fase actual
uint8_t control_ADC = 0; //indica si estamos realizando la rampa de arranque o estamos en control con ADC (=1)
uint8_t rampa_completa = 0; //nos indica si se ha completado la rampa (si está completa = 1)

char duty_anterior = 'H';
char duty_actual; //variable donde guardamos el valor recibido por SPI

/*
 * Función que controla las activaciones de los MOS según la fase
 */
void control_de_fases()
{
    switch(FASE)
    {
    case 1: //A+ B-
        TIMER_A0->CCTL[1] = OUTMOD_7; //modo SET/RESET para la salida del timer
        TIMER_A0->CCTL[3] = OUTMOD_0; //salida del bloque = valor del bit OUT (en nuestro caso 0)
        TIMER_A0->CCTL[4] = OUTMOD_0;
        P7 -> OUT |= 0x10;
        break;

    case 2: //A+ C-
        TIMER_A0->CCTL[1] = OUTMOD_7;
        TIMER_A0->CCTL[3] = OUTMOD_0;
        TIMER_A0->CCTL[4] = OUTMOD_0;
        P7 -> OUT ^= 0x50;
        break;

    case 3: //B+ C-
        TIMER_A0->CCTL[1] = OUTMOD_0;
        TIMER_A0->CCTL[3] = OUTMOD_7;
        TIMER_A0->CCTL[4] = OUTMOD_0;
        P7 -> OUT |= 0x40;
        break;

    case 4: //B+ A-
        TIMER_A0->CCTL[1] = OUTMOD_0;
        TIMER_A0->CCTL[3] = OUTMOD_7;
        TIMER_A0->CCTL[4] = OUTMOD_0;
        P7 -> OUT ^= 0x42;
        break;

    case 5: //C+ A-
        TIMER_A0->CCTL[1] = OUTMOD_0;
        TIMER_A0->CCTL[3] = OUTMOD_0;
        TIMER_A0->CCTL[4] = OUTMOD_7;
        P7 -> OUT |= 0x02;
        break;

    case 6: //C+ B-
        TIMER_A0->CCTL[1] = OUTMOD_0;
        TIMER_A0->CCTL[3] = OUTMOD_0;
        TIMER_A0->CCTL[4] = OUTMOD_7;
        P7 -> OUT ^= 0x12;
        break;
    }
    if(FASE == 6) //si hemos realizado la fase 6 y debemos cambiar, volvemos a la 1
    {
        FASE = 1;
    }
    else //en caso de encontrarnos en cualquier otra fase, incrementamos una fase
    {
        FASE++;
    }
}

/*
 * Función que comprueba si se ha recibido una orden de variar el duty cycle
 * La launchpad de control siempre mandará un 0 una vez de haya terminado de cambiar el duty,
 * por tanto, siempre que el valor recibido sea != de 0, significa que debemos cambiar el duty
 */
void check_duty()
{
    int i;
            if(duty_actual > duty_anterior)
            {
                if(duty_actual == 'A')
                {
                    while(TIMER_A0->CCR[1] < 180)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'B')
                {
                    while(TIMER_A0->CCR[1] < 190)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'C')
                {
                    while(TIMER_A0->CCR[1] < 200)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'D')
                {
                    while(TIMER_A0->CCR[1] < 210)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'E')
                {
                    while(TIMER_A0->CCR[1] < 220)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'F')
                {
                    while(TIMER_A0->CCR[1] < 230)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'G')
                {
                    while(TIMER_A0->CCR[1] < 240)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'H')
                {
                    while(TIMER_A0->CCR[1] < 250)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'I')
                {
                    while(TIMER_A0->CCR[1] < 260)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'J')
                {
                    while(TIMER_A0->CCR[1] < 270)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'K')
                {
                    while(TIMER_A0->CCR[1] < 270)
                    {
                        TIMER_A0->CCR[1] += 2;
                        TIMER_A0->CCR[3] += 2;
                        TIMER_A0->CCR[4] += 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
            }
            else
            {
                if(duty_actual == 'A')
                {
                    while(TIMER_A0->CCR[1] > 140)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'B')
                {
                    while(TIMER_A0->CCR[1] > 160)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'C')
                {
                    while(TIMER_A0->CCR[1] > 170)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'D')
                {
                    while(TIMER_A0->CCR[1] > 180)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'E')
                {
                    while(TIMER_A0->CCR[1] > 190)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'F')
                {
                    while(TIMER_A0->CCR[1] > 200)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'G')
                {
                    while(TIMER_A0->CCR[1] > 210)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'H')
                {
                    while(TIMER_A0->CCR[1] > 220)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'I')
                {
                    while(TIMER_A0->CCR[1] > 240)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
                if(duty_actual == 'J')
                {
                    while(TIMER_A0->CCR[1] > 250)
                    {
                        TIMER_A0->CCR[1] -= 2;
                        TIMER_A0->CCR[3] -= 2;
                        TIMER_A0->CCR[4] -= 2;
                        for(i = 0; i < 5000; i++);
                    }
                }
            }

        duty_anterior = duty_actual;
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
    __disable_irq();//deshabilitamos interrupciones a nivel global
    Clock_vInit();// inicializa reloj
    ini_GPIO_control_duty();
    ini_GPIO_map(); //inicializamos y mapeamos las salidas del timer
    init_interrupciones(); //inicializamos las interrupciones en NVIC
    init_SPI(); //inicializamos pines y módulo del SPI
    init_timerA1();//inicializacion TIMER_1 (PWM de los MOS superiores)
    init_timerA0();//inicializacion TIMER_0 (para realizar la rampa de arranque)
    init_ADC(); //inicializamos el ADC
    __enable_irq();//habilitación de interrupciones a nivel global

    int i;

    while(!(P3->IN & 0x01));

    while (1)
    {
        if(control_ADC == 0)
        {
            if(TIMER_A1->CCR[0] == 1600 || TIMER_A1->CCR[0] < 1600)
            {
                rampa_completa = 1; //damos por concluida la rampa y aseguramos que ya hay suficiente BEMF
            }
            else if(TIMER_A1->CCR[0] > 1500)
            {
                TIMER_A1->CCR[0] -= 100;
                /*if(TIMER_A0->CCR[1] < 230)
                {
                    TIMER_A0->CCR[1] += 2;
                    TIMER_A0->CCR[3] += 2;
                    TIMER_A0->CCR[4] += 2;
                }*/
            }
            for(i = 0; i < 25000; i++){}
        }
        else
        {
            if(EUSCI_B0 -> IFG & EUSCI_B_IFG_RXIFG) //si se ha recibido algun mensaje por SPI
            {
                duty_actual = EUSCI_B0->RXBUF; //guardamos el mensaje en una variable bajando el flag de lectura
                EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG;
                P1OUT ^= 0x01;
                if((duty_actual == 'A' || duty_actual == 'B' || duty_actual == 'C' || duty_actual == 'D' ||
                   duty_actual == 'E' || duty_actual == 'F' || duty_actual == 'G' || duty_actual == 'H' ||
                   duty_actual == 'I' || duty_actual == 'J' || duty_actual == 'K') && duty_actual != duty_anterior)
                {
                    check_duty(); //comprobamos que hemos recibido por si debemos variar el duty cycle
                    P2OUT ^= 0x01;
                }
            }
        }
    }
}

void PORT1_IRQHandler(void)
{
    P1->IE &= 0xEF;

    control_ADC = 0;
    rampa_completa = 0;
    TIMER_A1->CCR[0] = 20000;
    TIMER_A1->CTL = 0x02D4;
    P2OUT &= 0xFE;

    P1->IE |= 0x10;
    P1->IFG = 0x00;
}

/*
 * Interrupción del timer que usamos para generar la rampa de arranque
 */
void TA1_0_IRQHandler(void)
{
    TIMER_A1 -> CCTL[0] &= ~CCIFG; //Bajamos el flag de interrupción

    control_de_fases(); //cambiamos la fase mientras hacemos la rampa de arranque
}

/*
 * Interrupción del ADC que controla los cambios de fase en bucle cerrado
 * La base de esta interrupción es la selección del canal que debe mirarse para el siguiente cambio de fase.
 * ADC14->MCTL[0] pertenece al canal 18 y 19 en modo diferencial. Tenemos conectada la bobina A del motor.
 * ADC14->MCTL[1] pertenece al canal 20 y 21 en modo diferencial. Tenemos conectada la bobina B del motor.
 * ADC14->MCTL[2] pertenece al canal 22 y 23 en modo diferencial. Tenemos conectada la bobina C del motor.
 */
void ADC14_IRQHandler(void)
{
    if (ADC14->IFGR1 & ADC14_IFGR1_HIIFG) //Si estamos por encima del valor de threshold
    {
        ADC14->CLRIFGR0 = 0xFFFFFFFF; //Bajamos todas las banderas posibles
        ADC14->CLRIFGR1 = 0xFFFFFFFF;
        ADC14->IER1 &= ~ADC14_IER1_HIIE; //Desactivamos las interrupciones para que no siga saltando
        ADC14->IER1 &= ~ADC14_IER1_LOIE;
        if(control_ADC == 0) //Si estamos realizando la rampa de arranque
        {
            ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC); //deshabilitamos el ADC para cambiar la bobina a mirar
            ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC; //Seleccionamos el canal 22 para mirar la bobina C
            ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
            ADC14->MCTL[0] |= ADC14_MCTLN_WINC;
            ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);

            if(rampa_completa == 1) //Una vez se haya completado la rampa de arranque
            {
                TIMER_A1->CTL &= MC_0; //Desactivamos el timer
                control_ADC = 1; //Activamos la condición de lazo cerrado
                control_de_fases(); //Cambiamos de fase pues ha habido un paso por cero
                switch(FASE) //No sabemos de que fase venimos así que las definimos todas por evitar fallos
                {
                case 1:
                    ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[1] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH;;
                    ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    break;
                case 2:
                    ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[0] |= ADC14_MCTLN_WINC;
                    ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    break;
                case 3:
                    ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[2] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH;;
                    ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    break;
                case 4:
                    ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[1] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH;;
                    ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    break;
                case 5:
                    ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[0] |= ADC14_MCTLN_WINC;
                    ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    break;
                case 6:
                    ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                    ADC14->MCTL[2] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH;;
                    ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                    break;
                 }
                 P2OUT |= 0x01; //Encendemos el LED para indicar que el ADC tiene el control de los cambios de fase
             }
        }
        else //control_ADC = 1 (control en lazo cerrado)
        {
        switch(FASE) //definiremos únicamente las fases que pueden ocurrir si salta el HIGH threshold
        {
            case 1: //definimos la siguiente bobina a mirar, en la fase 2 será la B
                ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[1] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH; //Cambiamos los valores de threshold debido al offset que se suma
                ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                control_de_fases(); //cambiamos la fase
                break;
            case 3: //la siguiente bobina a mirar será la C
                ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[2] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH;
                ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                control_de_fases();
                break;
            case 5: //la siguiente bobina a mirar será la A
                ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[0] |= ADC14_MCTLN_WINC;
                ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                control_de_fases();
                break;
        }
        }
        ADC14->IER1 |= ADC14_IER1_LOIE; //Activamos la interrupción LOW porque es la siguiente que debe darse
    }
    if (ADC14->IFGR1 & ADC14_IFGR1_LOIFG) //Si ha saltado la interrupción LOW
    {
        ADC14->CLRIFGR0 = 0xFFFFFFFF;
        ADC14->CLRIFGR1 = 0xFFFFFFFF;
        ADC14->IER1 &= ~ADC14_IER1_LOIE;
        ADC14->IER1 &= ~ADC14_IER1_HIIE;
        if(control_ADC == 0) //Si estamos realizando la rampa de arranque
        {}
        else //Si se ha completado la rampa y estamos en control con el ADC, debemos cambiar de fase y el canal a mirar
        {
        switch(FASE) //definiremos únicamente las fases que pueden ocurrir si salta el LOW threshold
        {
            case 2: //la siguiente bobina a mirar será la A
                ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[0] |= ADC14_MCTLN_WINC;
                ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                control_de_fases();
                break;
            case 4: //la siguiente bobina a mirar será la B
                ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[2] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[1] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH;
                ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                control_de_fases();
                break;
            case 6: //la siguiente bobina a mirar será la C
                ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);
                ADC14->MCTL[0] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[1] &= ~ADC14_MCTLN_WINC;
                ADC14->MCTL[2] |= ADC14_MCTLN_WINC | ADC14_MCTLN_WINCTH;
                ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC);
                control_de_fases();
                break;
        }
        }
        ADC14->IER1 |= ADC14_IER1_HIIE; //Activamos la interrupción HIGH pues es la siguiente que debe saltar
    }
}


