/**********************************************************

        Student:    Israel Fabela Perez
        P:          SE_T2_FabelaPerez_busCan_Master.ccs
        Device:     Tiva C - TM4C1294NCPDT
        Subject:    Embedded Systems        *Semester: 2021-1
        Topic:      BUS CAN
        HM:         "Development of a protocol communication
        		BUS CAN to two boards TM4C129. This program contains
        			the protocol and the code for a sensor IR."

***********************************************************/
/* ======================================================== *
               MASTER - TM4C1294NCPDT [SENSOR IR]
 * ======================================================== */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/tm4c1294ncpdt.h"
#include "IEEE_CAN.h"
#include "driverlib/sysctl.h"

uint32_t adc_result_master;
uint64_t DatoRx_master;
float IR;

//--------------------------------------------------------------------
//%%%%%%    INICIALIZACION DE PUERTOS ASOCIADOS AL CAN0    %%%%%%%%%%%
//                 CAN0Rx: PA0    CAN0Tx: PA1
//--------------------------------------------------------------------
void Config_Puertos(void){                      //(TM4C1294NCPDT)
    SYSCTL_RCGCGPIO_R|=0x1;                     //Reloj Puerto A
    while((SYSCTL_PRGPIO_R&0x1)==0){}
    GPIO_PORTA_AHB_CR_R=0x3;
    GPIO_PORTA_AHB_AFSEL_R=0x3;                //PA0 y PA1 funciï¿½n alterna
    GPIO_PORTA_AHB_PCTL_R=0x77;                 //Funciï¿½n CAN a los pines PA0-PA1
    GPIO_PORTA_AHB_DIR_R=0x2;                  //PA1 Salida Tx y PA0 Entrada Rx
    GPIO_PORTA_AHB_DEN_R=0x3;                  //Hab funciï¿½n digital PA0 y PA1
}

//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%    INICIALIZACION CAN0     %%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
void Config_CAN(void){
    SYSCTL_RCGCCAN_R=0x1;                       //Reloj modulo 0 CAN
    while((SYSCTL_PRCAN_R&0x1)==0){}
                                                //Bit Rate= 1 Mbps      CAN clock=16 [Mhz]
    CAN0_CTL_R=0x41;                            //Deshab. modo prueba, Hab. cambios en la config. y hab. inicializacion
    CAN0_BIT_R=0x2BC0;                          //TSEG2=4   TSEG1=9    SJW=0    BRP=0
                                                //Lenght Bit time=[TSEG2+TSEG1+3]*tq
                                                //               =[(Phase2-1)+(Prop+Phase1-1)+3]*tq
    CAN0_CTL_R&=~0x41;                          //Hab. cambios en la config. y deshab. inicializacion
    CAN0_CTL_R|=0x2;                            //Hab de interrupcion en el modulo CAN
    NVIC_EN1_R|=((1<<(38-32)) & 0xFFFFFFFF);    //(TM4C1294NCPDT)
} //El 38 sale de p.116

//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%    RESOLUCION DE ERRORES    %%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
void CAN_Error(void){
    static int ent=0;
    if(CAN0_STS_R&0x80){
        if(ent){
            NVIC_APINT_R|=0x4;                      //Reinicio de todo el sistema
        }else{
            CAN0_CTL_R=0x41;                        //Hab. cambios en la config. y hab. inicializacion
            CAN0_CTL_R|=0x80;                       //Hab. modo prueba
            CAN0_TST_R|=0x4;                        //Hab. Modo silencio
            CAN0_CTL_R&=~0x41;                      //Hab. cambios en la config. y deshab. inicializacion
            SysCtlDelay(333333);
            CAN0_CTL_R=0x41;                        //Hab. cambios en la config. y hab. inicializacion
            CAN0_TST_R&=~0x4;                       //Deshab. Modo silencio
            CAN0_CTL_R&=~0x41;                      //Hab. cambios en la config. y deshab. inicializacion
            ent++;
        }
    }
}
//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%    INTERRUPCION DEL CAN0    %%%%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
void Inter_CAN0(void){
    uint8_t NoInt;
    NoInt=CAN0_INT_R;                       //Lectura del apuntador de interrupciones ¿Qué localidad recibió el dato?
    CAN0_STS_R&=~0x10;                     //Limpieza del bit de recepcion
    DatoRx_master=CAN_Rx(NoInt);                   //Recepcion de datos
    //CAN_Error();
}

//------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%  CONFIGURACION - SENSOR IR & LEDS   %%%%%%%%%%%%%%%%%%%%
//------------------------------------------------------------------
void Config_Sensor_IR(void){
	//Configuracion para el sensor IR
	//Puerto E3 para la entrada del sensor IR (Analogo)
	SYSCTL_RCGCGPIO_R |= 0x10; /* Enable Clock to GPIOE*/
	while((SYSCTL_PRGPIO_R & 0x10) == 0); /*Se espera a que el reloj se estabilice*/

	GPIO_PORTE_AHB_DIR_R = 0x00;    /*PE0 para Entradas (Analógica)*/
	GPIO_PORTE_AHB_AFSEL_R |= 0x08; /*Habilita la funcion Alterna; pg. 770*/
	GPIO_PORTE_AHB_DEN_R &= ~0x08;  /*Deshabilita la funcion Digital; pg. 781*/
	GPIO_PORTE_AHB_AMSEL_R |= 0x08; /*Habilita la funciona Analogica; pg. 786*/

	SYSCTL_RCGCADC_R |= 0x01;		/*Habilita el reloj para ADC0; pg. 396*/
	while((SYSCTL_PRADC_R & 0x01) == 0); /*Se espera a que el reloj se estabilice*/

	ADC0_PC_R = 0x01; 	   // Configura para 125Ksamp/s (p.1159)
	ADC0_SSPRI_R = 0x0123; //SS3 con la más alta prioridad
	ADC0_ACTSS_R &= ~0x00; /*Deshabilita SS3; pg. 1079*/
	ADC0_EMUX_R = ~0x0000; /*Software Trigger Conversion; pg. 1092*/
	ADC0_SSEMUX3_R = 0x00;  /*Analog Input E3 -> AIN0; pg. 1055*/
	ADC0_SSMUX3_R = (ADC0_SSMUX3_R & 0xFFFFFFF0) + 0;
	ADC0_SSCTL3_R = 0x06;  /*Sample Sequence Control; pg. 1142*/
	ADC0_IM_R = 0x0000;    /*Deshabilita interrupciones*/
	ADC0_ACTSS_R |= 0x08;  /*Habilita SS3*/

	SYSCTL_PLLFREQ0_R |= SYSCTL_PLLFREQ0_PLLPWR;  // encender PLL
	while((SYSCTL_PLLSTAT_R&0x01)==0); 			  // espera a que el PLL fije su frecuencia
	SYSCTL_PLLFREQ0_R &= ~SYSCTL_PLLFREQ0_PLLPWR; // apagar PLL

	ADC0_ISC_R = 0x08;	   /*Clear Flags*/
}


void Config_Leds_RX(void){
	SYSCTL_RCGCGPIO_R |= 0x1020; //Port N & F for LEDS

	while ((SYSCTL_PRGPIO_R & 0x1020) == 0){};  // reloj listo?

	GPIO_PORTN_DIR_R |= 0x0F;    // puerto N de salida N0-N3
	GPIO_PORTN_DEN_R |= 0x0F;    // habilita el puerto N

	GPIO_PORTF_AHB_DIR_R |= 0x1F;    // puerto F de salida F0-F4
	GPIO_PORTF_AHB_DEN_R |= 0x1F;    // habilita el puerto F
}

int Sensor_IR(void){
	ADC0_PSSI_R = 0x08;
	while ((ADC0_RIS_R & 0x08)==0);
	adc_result_master = (ADC0_SSFIFO3_R & 0xFFF);
	ADC0_ISC_R = 0x08;
	return adc_result_master;
}
void Leds_RX(){
	if(DatoRx_master>0 && DatoRx_master<64){
		GPIO_PORTN_DATA_R ^= 0x02;	// LED D1
		SysCtlDelay(16666);//DelayMilliSecs(1);
		GPIO_PORTN_DATA_R ^= 0x02;
	}else if(DatoRx_master>=64 && DatoRx_master<128){
		// LED D1 & D2
		GPIO_PORTN_DATA_R ^= 0x03;
		SysCtlDelay(16666);//DelayMilliSecs(1);
		GPIO_PORTN_DATA_R ^= 0x03;
	}else if(DatoRx_master>=128 && DatoRx_master<192){
		// LED D1, D2 & D3
		GPIO_PORTF_AHB_DATA_R ^= 0x10;
		GPIO_PORTN_DATA_R ^= 0x03;
		SysCtlDelay(16666);//DelayMilliSecs(1);
		GPIO_PORTN_DATA_R ^= 0x03;
		GPIO_PORTF_AHB_DATA_R ^= 0x10;
	}else if(DatoRx_master>=192){
		// LED D1, D2, D3 & D4
		GPIO_PORTF_AHB_DATA_R ^= 0x11;
		GPIO_PORTN_DATA_R ^= 0x03;
		SysCtlDelay(16666);//DelayMilliSecs(1);
		GPIO_PORTN_DATA_R ^= 0x03;
		GPIO_PORTF_AHB_DATA_R ^= 0x11;
	}

}

//------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%    PROGRAMA PRINCIPAL    %%%%%%%%%%%%%%%%%%%%
//------------------------------------------------------------------
void main(void){

    Config_Puertos();
    Config_CAN();
    Config_Sensor_IR();
    Config_Leds_RX();

    IR = Sensor_IR();
    //Localidad 1 Rx con Msk
    CAN_Memoria_Arb(0x222,false,0x1);//False recibir
    CAN_Memoria_CtrlMsk(0x111,8,false,true,false,0x1);
    //Localidad 2 Tx
    CAN_Memoria_Arb(0x111,true,0x2);//ID:0X111 True, mandar-- a localidad 0x2
    CAN_Memoria_CtrlMsk(0,8,false,false,false,0x2);
    CAN_Memoria_Dato(IR,0x2);
    //Localidad 3 Trama remota Rx
    CAN_Memoria_Arb(0x333,false,0x3);
    CAN_Memoria_CtrlMsk(0,8,false,true,false,0x3);

    while(1){

    	IR = Sensor_IR();
    	Leds_RX();

        CAN_Tx(0x2);//Localidad de memoria a la que se quiere transmitir
        CAN_Memoria_Dato(IR,0x2);
    }
}
