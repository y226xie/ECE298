#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
int is_monitored1 = 1;
int is_monitored2 = 1;
int is_monitored3 = 1;
int is_monitored4 = 1;
int buzzer_on = 0;
int toggle = 1;
int timer = 0;
int alarm_time1 = 0;
int alarm_time2 = 0;
int alarm_time3= 0;
int alarm_time4 = 0;
int disalarm_time1 = 0;
int disalarm_time2 = 0;
int disalarm_time3 = 0;
int disalarm_time4 = 0;
int defeat_flag = 0;
char user_input[9];
int user_counter = 0;

void main(void)
{
    char buttonState = 0;
    __disable_interrupt();
    WDT_A_hold(WDT_A_BASE);
    PM5CTL0 &= ~LOCKLPM5;

    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
    __enable_interrupt();
    RTC_init(RTC_BASE,32768,RTC_CLOCKPREDIVIDER_1);
    RTC_clearInterrupt(RTC_BASE,RTC_OVERFLOW_INTERRUPT_FLAG);
    RTC_enableInterrupt(RTC_BASE,RTC_OVERFLOW_INTERRUPT);
    RTC_start(RTC_BASE, RTC_CLOCKSOURCE_XT1CLK);
    while(1)
    {
        monitor();
        noise_sensor();
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState == 0)) //Look for rising edge
        {
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
            buttonState = 1;                //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState == 1)) //Look for falling edge
        {
            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
            buttonState = 0;                            //Capture new button state
        }
        if (ADCState == 0)
        {
            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        }
    }

}
////
#pragma vector=RTC_VECTOR
__interrupt
void RTC_ISR(void)
{
    timer++;
    displayValue(timer);

    RTC_clearInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT_FLAG);
}

void Init_GPIO(void)
{
    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN1);
    //4 pins for leds
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
}

void monitor(void) {
      int door1 = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6));
      int door2 = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3));
      int door3 = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4));
      int door4 = !(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5));

      if (timer >= alarm_time1  && timer <= disalarm_time1){
          is_monitored1 = 1;
      } else if (timer >= alarm_time1  && timer >= disalarm_time1){
          if (alarm_time1 > disalarm_time1){
              is_monitored1 = 1;
          } else {
              is_monitored1 = 0;
          }
      } else if (timer <= alarm_time1  && timer >= disalarm_time1){
          is_monitored1 = 0;
      }

      if (timer >= alarm_time2  && timer <= disalarm_time2){
                is_monitored2 = 1;
            } else if (timer >= alarm_time2  && timer >= disalarm_time2){
                if (alarm_time2 > disalarm_time2){
                    is_monitored2 = 1;
                } else {
                    is_monitored2 = 0;
                }
            } else if (timer <= alarm_time2  && timer >= disalarm_time2){
                is_monitored2 = 0;
            }
      if (timer >= alarm_time3  && timer <= disalarm_time3){
                is_monitored3 = 1;
            } else if (timer >= alarm_time3  && timer >= disalarm_time3){
                if (alarm_time3 > disalarm_time3){
                    is_monitored3 = 1;
                } else {
                    is_monitored3 = 0;
                }
            } else if (timer <= alarm_time3  && timer >= disalarm_time3){
                is_monitored3 = 0;
            }
      if (timer >= alarm_time4  && timer <= disalarm_time4){
                is_monitored4 = 1;
            } else if (timer >= alarm_time4  && timer >= disalarm_time4){
                if (alarm_time4 > disalarm_time4){
                    is_monitored4 = 1;
                } else {
                    is_monitored4 = 0;
                }
            } else if (timer <= alarm_time4  && timer >= disalarm_time4){
                is_monitored4 = 0;
            }

    /*LED 1 starts here*/
    if (is_monitored1 == 1) {
        if (door1 == 1 ) { //Door is open
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
            Init_Buzzer();
            buzzer_on = 1;
            defeat_flag = 1;
        }
        else if (door1 == 0) { //Door is closed
             GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
             buzzer_on = 0;
        }

    }

     /*LED2 starts here */
    if(is_monitored2 == 1) {
        if(door2 == 1) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
            Init_Buzzer();
            buzzer_on = 1;
            defeat_flag = 1;

        }
        else if (door2 == 0) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
            buzzer_on = 0;
        }
    }

     /*LED 3 starts here */
     if (is_monitored3 == 1) {
            if (door3 == 1 ) { //Door is open
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
                Init_Buzzer();
                buzzer_on = 1;
                defeat_flag = 1;

            }
            else if (door3 == 0) { //Door is closed
                 GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
                 buzzer_on = 0;
            }
     }
      /* LED 4 starts here */
      if (is_monitored4 == 1) {
           if (door4 == 1 ) {      //Door is open
               GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                Init_Buzzer();
                buzzer_on = 1;
                defeat_flag = 1;
           }
           else if (door4 == 0) { //Door is closed
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                buzzer_on = 0;
           }
     }

    //defeat control
    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 0 ){
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
        defeat_flag = 0;
    } else{
        if ( defeat_flag){
            Init_Buzzer();

        }
    }




}
void noise_sensor(void) {
    if (is_monitored1 == 1 || is_monitored2 == 1 || is_monitored3 == 1 || is_monitored4 == 1 ) {
        char temp_array[3] = {0};
        unsigned int temp = 0;
        temp = (int)ADCResult;
        temp_array[0] = temp%10;
        temp = temp/10;
        temp_array[1] = temp%10;
        temp = temp/10;
        temp_array[2] = temp%10;
        if (temp_array[2] >= 9 && temp_array[1] >= 6){
            Init_Buzzer();
            defeat_flag = 1;
        }
    }
}


void Init_Buzzer(void) {
    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1 ){
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1) {
               GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
           }
           else {
               GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
           }
    }
}

/* Clock System Initialization */
void Init_Clock(void)
{
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar    = 6;
    param.firstModReg       = 8;
    param.secondModReg      = 17;
    param.parity            = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode          = EUSCI_A_UART_MODE;
    param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);
    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        user_input[user_counter] = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);

        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
        if (user_input[user_counter] == '\r'){
            char temp = '\n';
            __delay_cycles(10000);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, temp);

            temp = '\r';
            __delay_cycles(10000);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, temp);

        }

        user_counter++;
        if (user_counter == 3 && user_input[2] == '\r'){

            user_input_helper1();
            user_counter = 0;

        } else if (user_counter == 5 && user_input[4] == '\r'){
            user_input_helper2();
            user_counter = 0;
        } else if (user_counter == 6 && user_input[5] == '\r'){
            user_input_helper2();
            user_counter = 0;
        } else if (user_counter == 7 && user_input[6] == '\r'){
            user_input_helper2();
            user_counter = 0;
        } else if (user_counter == 8 && user_input[7] == '\r'){
            user_input_helper2();
            user_counter = 0;
        } else if (user_counter > 8){
            user_counter = 0;
        }
    }
}
void nextline(void){
    char temp = '\n';
    __delay_cycles(10000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, temp);

    temp = '\r';
    __delay_cycles(10000);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, temp);
}
void display_string(char* str,int size){
    int i =0;
    for (i = 0; i< size; i++){
        __delay_cycles(10000);
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, str[i]);

    }
}

void user_input_helper1(void){
    if (user_input[0] == 'D' || user_input[0] == 'd'){
        if (user_input[1] == '1'){
            is_monitored1 = 0;
        } else if (user_input[1] == '2'){
            is_monitored2 = 0;
        } else if (user_input[1] == '3'){
            is_monitored3 = 0;
        } else if (user_input[1] == '4'){
            is_monitored4 = 0;
        }
    } else {
        if (user_input[1] == '1'){
            is_monitored1 = 1;
        } else if (user_input[1] == '2'){
            is_monitored2 = 1;
        } else if (user_input[1] == '3'){
            is_monitored3 = 1;
        } else if (user_input[1] == '4'){
            is_monitored4 = 1;
        }
    }
}

void user_input_helper2(void){
    if (user_input[0] == 'D'|| user_input[0] == 'd'){
//        showChar('2', pos5);
        if (user_counter == 5){
            if (user_input[1] == '1'){
                disalarm_time1 =  user_input[3] - '0';
                nextline();
                char temp[] = "Zone 1 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                nextline();
            } else if (user_input[1] == '2'){
                disalarm_time2 =  user_input[3] - '0';
                nextline();
               char temp[] = "Zone 2 is disarmed after ";
               display_string(temp, strlen(temp));
               __delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
               nextline();

            }  else if (user_input[1] == '3'){
                disalarm_time3 =  user_input[3] - '0';
                nextline();
               char temp[] = "Zone 3 is disarmed after ";
               display_string(temp, strlen(temp));
               __delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
               nextline();
            }  else if (user_input[1] == '4'){
                disalarm_time4 =  user_input[3] - '0';
                nextline();
               char temp[] = "Zone 4 is disarmed after ";
               display_string(temp, strlen(temp));
               __delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
               nextline();
            }
        } else if (user_counter == 6){
            if (user_input[1] == '1'){
                disalarm_time1 = (user_input[3] - '0')*10+(user_input[4] - '0');
                nextline();
               char temp[] = "Zone 1 is disarmed after ";
               display_string(temp, strlen(temp));
               __delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
               __delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
               nextline();
            } else if (user_input[1] == '2'){
                disalarm_time2 = (user_input[3] - '0')*10+(user_input[4] - '0');
                nextline();
                char temp[] = "Zone 2 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                nextline();
            }  else if (user_input[1] == '3'){
                disalarm_time3 = (user_input[3] - '0')*10+(user_input[4] - '0');
                nextline();
                char temp[] = "Zone 3 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                nextline();
            }  else if (user_input[1] == '4'){
                disalarm_time4 = (user_input[3] - '0')*10+(user_input[4] - '0');
                nextline();
               char temp[] = "Zone 4 is disarmed after ";
               display_string(temp, strlen(temp));
               __delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
               _delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
               nextline();
            }
        } else if (user_counter == 7){
            if (user_input[1] == '1'){
                disalarm_time1 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                nextline();
               char temp[] = "Zone 1 is disarmed after ";
               display_string(temp, strlen(temp));
               __delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
               _delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
               _delay_cycles(10000);
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
               nextline();
            } else if (user_input[1] == '2'){
                disalarm_time2 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                nextline();
                char temp[] = "Zone 2 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                nextline();
            }  else if (user_input[1] == '3'){
                disalarm_time3 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                nextline();
                char temp[] = "Zone 3 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                nextline();
            }  else if (user_input[1] == '4'){
                disalarm_time4 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                nextline();
                char temp[] = "Zone 4 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                nextline();
            }

        } else if (user_counter == 8){
            if (user_input[1] == '1'){
                disalarm_time1 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0');
                nextline();
                char temp[] = "Zone 1 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            } else if (user_input[1] == '2'){
                disalarm_time2 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0');
                nextline();
                char temp[] = "Zone 2 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            }  else if (user_input[1] == '3'){
                disalarm_time3 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0');
                nextline();
                char temp[] = "Zone 3 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            }  else if (user_input[1] == '4'){
                disalarm_time4 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0');
                nextline();
                char temp[] = "Zone 4 is disarmed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            }
        }
    } else {
        if (user_counter == 5){
            if (user_input[1] == '1'){
               alarm_time1 =  user_input[3] - '0';
               nextline();
                char temp[] = "Zone 1 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                nextline();
           } else if (user_input[1] == '2'){
               alarm_time2 =  user_input[3] - '0';
               nextline();
                char temp[] = "Zone 2 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                nextline();
           }  else if (user_input[1] == '3'){
               alarm_time3 =  user_input[3] - '0';
               nextline();
                char temp[] = "Zone 3 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                nextline();
           }  else if (user_input[1] == '4'){
               alarm_time4 =  user_input[3] - '0';
               nextline();
                char temp[] = "Zone 4 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                nextline();
           }
        } else if (user_counter ==6){
            if (user_input[1] == '1'){
                alarm_time1 = (user_input[3] - '0')*10+(user_input[4] - '0');
                nextline();
                char temp[] = "Zone 1 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                nextline();
            } else if (user_input[1] == '2'){
                alarm_time2 = (user_input[3] - '0')*10+(user_input[4] - '0');
                nextline();
                char temp[] = "Zone 2 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                nextline();
            }  else if (user_input[1] == '3'){
                alarm_time3 = (user_input[3] - '0')*10+(user_input[4] - '0');
                 nextline();
                char temp[] = "Zone 3 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                nextline();
            }  else if (user_input[1] == '4'){
                alarm_time4 = (user_input[3] - '0')*10+(user_input[4] - '0');
                 nextline();
                char temp[] = "Zone 4 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                nextline();
            }
        } else if (user_counter == 7){
            if (user_input[1] == '1'){
                alarm_time1 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                nextline();
                char temp[] = "Zone 1 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                nextline();
            } else if (user_input[1] == '2'){
                alarm_time2 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                nextline();
                char temp[] = "Zone 2 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                nextline();
            }  else if (user_input[1] == '3'){
                alarm_time3 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                 nextline();
                char temp[] = "Zone 3 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                nextline();
            }  else if (user_input[1] == '4'){
                alarm_time4 = (user_input[3] - '0')*100+(user_input[4] - '0')*10+(user_input[5] - '0') ;
                 nextline();
                char temp[] = "Zone 4 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                nextline();
            }
        } else if (user_counter == 8){
            if (user_input[1] == '1'){
                alarm_time1 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0');
                 nextline();
                char temp[] = "Zone 1 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            } else if (user_input[1] == '2'){
                alarm_time2 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0'); nextline();
                 nextline();
                char temp[] = "Zone 2 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            }  else if (user_input[1] == '3'){
                alarm_time3 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0');
                nextline();
                char temp[] = "Zone 3 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            }  else if (user_input[1] == '4'){
                alarm_time4 =  (user_input[3] - '0')*1000+ (user_input[4] - '0')*100+ (user_input[5] - '0')*10+ (user_input[6] - '0');
                 nextline();
                char temp[] = "Zone 4 is armed after ";
                display_string(temp, strlen(temp));
                __delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[3]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[4]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[5]);
                _delay_cycles(10000);
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, user_input[6]);
                nextline();
            }
        }
    }
}
/* PWM Initialization */
void Init_PWM(void)
{
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);
    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

