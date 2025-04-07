/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Most of the information used in this code has been extracted from:
 * https://longnight975551865.wordpress.com/2018/02/11/how-to-read-data-from-mpu9250/
 * https://github.com/Earsuit/I2C/blob/master/examples/MPU9250/MPU9250.ino
 * 'MPU-9250 Register Map And Descriptions' document from InvenSense
 */

/*
 * main.c
 *
 *  Created on: 28 nov. 2021
 *      Author: Arnau Quintana
 */

/*
 *  ======== empty_min.c ========
 */
/* Board Header file */
#include "Board.h"
/* XDCtools Header files */
#include <xdc/std.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Mailbox.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <math.h>

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define IMU_I2C_ADDR 0x68 //(1101000 according to datasheet)
#define IMU_I2C MSP_EXP432P401R_I2CB1

void HwiButtons(UArg arg); //subirá o bajrá el ángulo deseado según el botón pulsado
void init_I2C(); //inicializa los pines del I2C, es redundante pero nos aseguramos que definimos bien los pines
void DataClock(UArg arg); //swi que desbloquea la task de recogida de datos de la IMU
void I2CFxn(UArg arg0, UArg arg1); //task que recoge y procesa los datos de la IMU
void PIDFxn(UArg arg0, UArg arg1); //acaba el procesamiento de datos desde la IMU y calcula los parámetros PID para ajustar la velocidad de los motores
void init_SPI(); //inicializa los pines del SPI, es redundante pero aseguramos que la inicialización es correcta
void SPITaskFxn(UArg arg0, UArg arg1); //task que envía el valor de duty necesario al motor derecho
void spiBusTransmitFinished(SPI_Handle handle, SPI_Transaction *transaction); //función que toggle el LED una vez la transacción se ha realizado
void SPI1TaskFxn(UArg arg0, UArg arg1); //task que envía el valor de duty necesario al motor izquierdo

extern Task_Handle task0; //to obtain data from IMU
extern Task_Handle task1; //processes data from task0
extern Task_Handle task2;
extern Task_Handle task3;
extern Clock_Handle clock0; //For get data every 1ms
extern Semaphore_Handle semData; //will unlock the GetDataFxn task
extern Semaphore_Handle semIMUDATA;
extern Semaphore_Handle semSPI0;
extern Semaphore_Handle semSPI2;

uint16_t angle_anterior = 0; //valor del angulo anterior
float desired_angle = 0.0; //angulo de inclinación deseado
int tstart = 0; //guarda el valor del tiempo inicial

float elapsedTime; //tiempo transcurrido entre medidas
int  timeAct, timePrev; //tiempo de la medida actual y tiempo de la anterior
float rad_to_deg = 180/3.1415926; //para pasar de radianes a grados

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
uint8_t L, R;

uint16_t cal_int = 0;
int16_t gyro_x_cal, gyro_y_cal, gyro_z_cal; //valor que restaremos a cada medida, para compensar el offset
int16_t acc_x_cal, acc_y_cal, acc_z_cal;
int16_t x_acc, y_acc, z_acc,x_gyro, y_gyro, z_gyro; //medida que recogemos de la IM

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_init();
    GPIO_init();
    init_I2C();
    init_SPI();

    GPIO_enableInt(Board_GPIO_BUTTON0);
    GPIO_enableInt(Board_GPIO_BUTTON1);

    Clock_start(clock0); //clock initialization
    tstart = Timestamp_get32(); //guardamos el tiempo de inicio
    P10 -> DIR |= 0x30; //configuramos GPIOs que avisaran a los motores de que empiecen la rampa
    P10 -> OUT &= 0xCF;
    //P2 -> OUT &= 0xFC; //Leds verde y azul que indican cuando se ha producido un cambio en el angulo deseado

    /* Start BIOS */
    BIOS_start();

    return (0);
}

/*
 * -----------------------------------------Hwi----------------------------------------------------
 */

/*
 * Function that changes the desired angle depending on the buttton that have been pressed
 * If S1 is pressed, the rocker will go left, and when S2 is pressed, the rocker will go to the right.
 */
void HwiButtons(UArg arg)
{
    if((P1 -> IFG & 0x02))
    {
        GPIO_clearInt(Board_GPIO_BUTTON0);
        if(desired_angle < 20.0)
        {
            desired_angle += 1;
        }
        else
        {
            desired_angle = 20.0;
        }
        P2 -> OUT ^= 0x02; //cada vez que cambie en angulo a más + togglea el led verde
    }
    if(P1 -> IFG & (1 << 4))
    {
        GPIO_clearInt(Board_GPIO_BUTTON1);
        if(desired_angle > -20.0)
        {
            desired_angle -= 1;
        }
        else
        {
            desired_angle = -20.0;
        }
        P2 -> OUT ^= 0x04; //cada vez que cambie el ángulo a más + toggle el led azul
    }
}

/*
 * ----------------------------------------Swi--------------------------------------------------
 */

/*
 * Function that unlocks the 'I2CFxn' task every 1ms
 */
void DataClock(UArg arg)
{
    Semaphore_post(semData); //to unlock the task that request data from IMU
}

/*------------------------------------Tasks---------------------------------------------------*/

/*
 * Function that gets data from the IMU
 */
void I2CFxn(UArg arg0, UArg arg1)
{
    uint8_t         rxBuffer[14]; //data bytes that will be received
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction = {0};

    bool status; //if = true, correct transaction. If = false, transaction failed

    uint8_t clock_source[2];
    clock_source[0] = 0x6B; //PWR_MGMT_1 register to configure clock source
    clock_source[1] = 0x00; //bit 0 = CLKSEL[2:0]. If = 0, internal 20MHz oscillator

    uint8_t gyro_setup[2];
    gyro_setup[0] = 0x1B; //register 27 controls the gyroscope (GYRO_CONFIG)
    gyro_setup[1] = 0x08; //GYRO_FS_SEL = +500dps (bit 4 and 3 = 01)

    uint8_t acc_setup[2];
    acc_setup[0] = 0x1C; //register 28 controls the accelerometer
    acc_setup[1] = 0x08; //ACCEL_FS_SEL = +- 4g (bit 4 and 3 = 01)

    uint8_t read_registers[1];
    read_registers[0] = 0x3B;

    /*
    uint8_t acc_setup_2[2];
    acc_setup_2[0] = 0x1D; //register 29 controls part of the accelerometer
    acc_setup_2[1] = 0x05; //turn on internal low-pass filter with 10.2Hz bandwidth

    uint8_t gyro_setup_2[2];
    gyro_setup_2[0] = 0x1A; //register 26 controls part of the gyroscope
    gyro_setup_2[1] = 0x05; //turn on internal low-pass filter with 10Hz bandwidth
    */

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz; //I2C frequency
    i2c = I2C_open(IMU_I2C, &i2cParams); //open the module
    if (i2c == NULL)
    {
        System_abort("Error Initializing I2C\n"); //Initialization failed
    }
    else //I2C correctly initializated
    {
        //System_printf("I2C Initialized!\n");
        i2cTransaction.slaveAddress = IMU_I2C_ADDR; //Slave adress
        i2cTransaction.writeBuf = clock_source; //data that will be sent to slave
        i2cTransaction.writeCount = sizeof(clock_source); //size of the data that will be sent
        i2cTransaction.readBuf = NULL; //data that will be received from slave (none this time)
        i2cTransaction.readCount = 0;

        status = I2C_transfer(i2c, &i2cTransaction); //start transaction

        if (status == true)
        {
            System_printf("IMU clock source correctly established\n");
            System_flush();
        }
        else
        {
            System_printf("IMU clock configuration failed\n");
            System_flush();
            while(1){} //Configuration failed, system waits forever
        }

        i2cTransaction.slaveAddress = IMU_I2C_ADDR;
        i2cTransaction.writeBuf = acc_setup;
        i2cTransaction.writeCount = sizeof(acc_setup);
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0;

        status = I2C_transfer(i2c, &i2cTransaction);

        if (status == true)
        {
            System_printf("IMU accelerometer correctly configured\n");
            System_flush();
        }
        else
        {
        System_printf("IMU accelerometer configuration failed\n");
        System_flush();
        while(1){} //Configuration setup failed, system waits for a reset
        }

        i2cTransaction.slaveAddress = IMU_I2C_ADDR;
        i2cTransaction.writeBuf = gyro_setup;
        i2cTransaction.writeCount = sizeof(gyro_setup);
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0;

        status = I2C_transfer(i2c, &i2cTransaction);

        if (status == true)
        {
            System_printf("IMU gyroscope correctly configured\n");
            System_flush();
        }
        else
        {
            System_printf("IMU gyroscope configuration failed\n");
            System_flush();
            while(1){} //Configuration failed, system waits forever
        }
        while (1)
        {
            Semaphore_pend(semData, BIOS_WAIT_FOREVER); //wait for the semaphore to unlock the task

            i2cTransaction.slaveAddress = IMU_I2C_ADDR;
            i2cTransaction.writeBuf = read_registers;
            i2cTransaction.writeCount = 1;
            i2cTransaction.readBuf = rxBuffer;
            i2cTransaction.readCount = sizeof(rxBuffer); //number of bytes that we will receive

            status = I2C_transfer(i2c, &i2cTransaction);

            if (status == true){}
            else
            {
                System_printf("Lecture failed\n");
                System_flush();
                while(1){} //Lecture failed, system waits forever
            }

            x_acc = rxBuffer[0] << 8 | rxBuffer[1]; //(highByte<<8 | lowByte)
            y_acc = rxBuffer[2] << 8 | rxBuffer[3];
            z_acc = rxBuffer[4] << 8 | rxBuffer[5];
            x_gyro = rxBuffer[8] << 8 | rxBuffer[9];
            y_gyro = rxBuffer[10] << 8 | rxBuffer[11];
            z_gyro = rxBuffer[12] << 8 | rxBuffer[13];

            Semaphore_post(semIMUDATA); //to unlock the PID task
        }
        /* Deinitialized I2C */
        I2C_close(i2c); //for some reason the while(1) has failed so we must release the module
        System_printf("I2C closed!\n");
        System_flush();
    }
}

/*
 * Function to process the data obtained in the previous task
 */
void PIDFxn(UArg arg0, UArg arg1)
{
    float dutyLeft, dutyRight;
    float PID, error, previous_error;
    float duty_initial = 25.0;
    float pid_p = 0.0;
    float pid_i = 0.0;
    float pid_d = 0.0;
    float kp = 3.0; //constantes PID //will 8.0, works fine
    float ki = 0.0022; //with 0.005 works fine
    float kd = 0.038; //with 0.06 works fine

    Types_FreqHz freq;
    Timestamp_getFreq(&freq); //to obtain clock frequency

    while(1)
    {
        Semaphore_pend(semIMUDATA, BIOS_WAIT_FOREVER);

        if (cal_int < 2000) //calibration of the gyro and acelerometer
        {
            if(cal_int == 0)
            {
                System_printf("Calibrating Gyroscope...\n");
                System_flush();
            }
            if(cal_int == 1999)
            {
                System_printf("Gyroscope calibrated\n");
                System_flush();
            }
            gyro_x_cal += x_gyro;
            gyro_y_cal += y_gyro;
            gyro_z_cal += z_gyro;
            acc_z_cal += z_acc;
            acc_z_cal += z_acc;
            acc_z_cal += z_acc;
            cal_int++;
            continue;
        }
        if(cal_int == 2000)
        {
            gyro_x_cal /= 2000;
            gyro_y_cal /= 2000;
            gyro_z_cal /= 2000;
            acc_x_cal /= 2000;
            acc_y_cal /= 2000;
            acc_z_cal /= 2000;
            cal_int++;
        }
        x_gyro -= gyro_x_cal; //substract the constant error from the measurement
        y_gyro -= gyro_y_cal;
        z_gyro -= gyro_z_cal;
        x_acc -= acc_x_cal;
        y_acc -= acc_y_cal;
        z_acc -= acc_z_cal;

        timePrev = timeAct; //el tiempo previo ahora es el actual
        timeAct = Timestamp_get32(); //capturamos el momento en que cogemos una medida
        elapsedTime = ((timeAct - timePrev)/(freq.lo/1000000)); //ticks a valor de tiempo
        elapsedTime /= 1000000; //pasamos el valor a segundos

        /*---X---*/
        Acceleration_angle[0] = atanf((y_acc/8192.0)/sqrt(pow((x_acc/8192.0),2) + pow((z_acc/8192.0),2)))*rad_to_deg; //filtro complementario
        /*---Y---*/
        Acceleration_angle[1] = atanf(-1*(x_acc/8192.0)/sqrt(pow((y_acc/8192.0),2) + pow((z_acc/8192.0),2)))*rad_to_deg;

        /*---X axis angle---*/
        Total_angle[0] = 0.98 *(Total_angle[0] + (x_gyro/65.5)*elapsedTime) + 0.02*Acceleration_angle[0]; //filtro que tiene en cuanta las aportaciones del giroscopio y el acelerómetro
        /*---Y axis angle---*/
        Total_angle[1] = 0.98 *(Total_angle[1] + (y_gyro/65.5)*elapsedTime) + 0.02*Acceleration_angle[1];

        float angle = (Total_angle[1] - 37.7) * 2.0; //ajustamos el valor del angulo teniendo en cuenta el offset

        //int angle_int = (int) angle; //por si necesitamos el angulo sin los decimales

        if(cal_int < 2300) //esperemos unos milisegundos para que se estabilicen las medidas
        {
            cal_int++;
            continue;
        }
        else
        {
        P10  -> OUT |= 0x30; //indicamos a los ESC que empiecen la rampa

        error = angle - desired_angle; //calculamos la diferencia entre el angulo actual y el deseado

        pid_p = kp * error; //parte proporcional
        pid_i = pid_i + (ki*error); //parte integral
        pid_d = kd * ((error - previous_error) / elapsedTime); //parte derivativa

        PID = pid_p + pid_i + pid_d; //sumamos todas las aportaciones

        if(PID < -5.0) //la diferencia entre mi duty minimo y máximo es de 10
        {
          PID = -5.0;
        }
        if(PID > 5.0)
        {
          PID = 5.0;
        }

        dutyLeft = duty_initial + PID ; //recalculamos los dutys necesarios
        dutyRight =  duty_initial - PID;

        if(dutyLeft < 18) //seguro para no imponer un duty demasiado alto o bajo a los motores, aunque
        {                 //en las task del SPI se escribe en el buffer únicamente cuando el valor dado por
            dutyLeft= 18; //estas variables esta entre 18 y 28
        }
        if(dutyLeft > 28)
        {
            dutyLeft = 28;
        }
        if(dutyRight < 18)
        {
            dutyRight = 18;
        }
        if(dutyRight > 28)
        {
            dutyRight = 28;
        }

        L = (int)dutyLeft; //convertimos el float en un int para las comparaciones en la siguiente task
        R = (int)dutyRight;

        previous_error = error; //actualizamos la variable para poder tener en cuenta el error anterior en el siguiente cálculo
       }

        Semaphore_post(semSPI0); //desbloqueamos la primera task del SPI
    }
}

void SPITaskFxn(UArg arg0, UArg arg1)
{
    SPI_Handle      spi1;
    SPI_Params      spiParams;
    SPI_Transaction spiTransaction;
    uint8_t MSGSIZE = 1; //bytes a enviar
    uint8_t         transmitBuffer[MSGSIZE];
    uint8_t         receiveBuffer[MSGSIZE];
    bool            transferOK;

    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.dataSize = 8;       // 8-bit data size
    spiParams.mode = SPI_MASTER;  //modo MASTER
    spiParams.bitRate = 3000000;   // 1MHz SPI clock
    spiParams.frameFormat= SPI_POL0_PHA1; //Data sampled on falling edge and changed in rising edge
    /*
     * https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
     */
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = spiBusTransmitFinished; //función llamada al finalizar una transacción


    spi1 = SPI_open(Board_SPI1, &spiParams); //nos apoderamos el módulo
    while (spi1 == NULL)
    {
    System_printf("SPI initialitzation gone wrong\n");
    System_flush();
    }

    while(1)
    {
        Semaphore_pend(semSPI0, BIOS_WAIT_FOREVER);

        if(R == 18) {strcpy((char*)transmitBuffer, "A");} //dependiendo del duty necesario, enviamos un char o otro
        else if(R == 19) {strcpy((char*)transmitBuffer, "B");}
        else if(R == 20) {strcpy((char*)transmitBuffer, "C");}
        else if(R == 21) {strcpy((char*)transmitBuffer, "D");}
        else if(R == 22) {strcpy((char*)transmitBuffer, "E");}
        else if(R == 23) {strcpy((char*)transmitBuffer, "F");}
        else if(R == 24) {strcpy((char*)transmitBuffer, "G");}
        else if(R == 25) {strcpy((char*)transmitBuffer, "H");}
        else if(R == 26) {strcpy((char*)transmitBuffer, "I");}
        else if(R == 27) {strcpy((char*)transmitBuffer, "J");}
        else if(R == 28) {strcpy((char*)transmitBuffer, "K");}

        // Fill in transmitBuffer
        spiTransaction.count = MSGSIZE;
        spiTransaction.txBuf = (void *)transmitBuffer; //buffer que se enviará
        spiTransaction.rxBuf = (void *)receiveBuffer; //buffer que se recibirá

        transferOK = SPI_transfer(spi1, &spiTransaction); //ejectuamos la transacción
        if (!transferOK)
        {
        // Error in SPI or transfer already in progress.
        System_printf("Transfer gone wrong\n");
        System_flush();
        while (1);
        }
        else
        {
            System_printf("Transfer OK\n");
            System_flush();
        }

        Semaphore_post(semSPI2); //desbloqueamos la otra task del SPI
    }
}

/*
 * Función a la que se entra cuando se finaliza una transacción
 */
void spiBusTransmitFinished(SPI_Handle handle, SPI_Transaction *transaction)
{
    GPIO_toggle(Board_GPIO_LED0);
}


void SPI1TaskFxn(UArg arg0, UArg arg1)
{
    SPI_Handle      spi;
    SPI_Params      spiParams;
    SPI_Transaction spiTransaction;
    uint8_t MSGSIZE = 1; //bytes a enviar
    uint8_t         transmitBuffer[MSGSIZE];
    uint8_t         receiveBuffer[MSGSIZE];
    bool            transferOK;

    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.dataSize = 8;       // 8-bit data size
    spiParams.mode = SPI_MASTER;  //modo MASTER
    spiParams.bitRate = 3000000;   // 1MHz SPI clock
    spiParams.frameFormat= SPI_POL0_PHA1; //Data sampled on falling edge and changed in rising edge
    /*
     * https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
     */
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = spiBusTransmitFinished; //función llamada al finalizar una transacción


    spi = SPI_open(Board_SPI0, &spiParams);
    if (spi == NULL)
    {
    System_printf("SPI initialitzation gone wrong\n");
    System_flush();
    }

    while(1)
    {
        Semaphore_pend(semSPI2, BIOS_WAIT_FOREVER);

        if(L == 18) {strcpy((char*)transmitBuffer, "A");}
        else if(L == 19) {strcpy((char*)transmitBuffer, "B");}
        else if(L == 20) {strcpy((char*)transmitBuffer, "C");}
        else if(L == 21) {strcpy((char*)transmitBuffer, "D");}
        else if(L == 22) {strcpy((char*)transmitBuffer, "E");}
        else if(L == 23) {strcpy((char*)transmitBuffer, "F");}
        else if(L == 24) {strcpy((char*)transmitBuffer, "G");}
        else if(L == 25) {strcpy((char*)transmitBuffer, "H");}
        else if(L == 26) {strcpy((char*)transmitBuffer, "I");}
        else if(L == 27) {strcpy((char*)transmitBuffer, "J");}
        else if(L == 28) {strcpy((char*)transmitBuffer, "K");}

        // Fill in transmitBuffer
        spiTransaction.count = MSGSIZE;
        spiTransaction.txBuf = (void *)transmitBuffer;
        spiTransaction.rxBuf = (void *)receiveBuffer;

        transferOK = SPI_transfer(spi, &spiTransaction);
        if (!transferOK)
        {
        // Error in SPI or transfer already in progress.
        System_printf("Transfer gone wrong\n");
        System_flush();
        while (1);
        }
    }
}
/*
 * ---------------------------------------Configuration NO RTOS--------------------------------------------
 */

/*
 * I2C initialization
 */
void init_I2C()
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P6,
            GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION); //PIN SCL

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P6,
            GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION); //PIN SDA
    /* Initialize the I2C driver */
    I2C_init();
}

void init_SPI()
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P1,
            GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION); //PIN CLK MODULE 0

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P1,
            GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION); //PIN MOSI MODULE 0

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION); //PIN CLK MODULE 2

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION); //PIN MOSI MODULE 2

    SPI_init();
}
