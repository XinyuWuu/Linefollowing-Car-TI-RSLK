// 编码器一转360个脉冲

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#define TIMER0_PERIOD 600  //PWM
#define TIMER1_PERIOD 3000 //红外PID
#define DUTY_CYCLER 300
#define DUTY_CYCLEL 300
#define NUMBER_TIMER_CAPTURES 5 //速度测量次数，一次转过一度
#define kp 3.5
#define kd 300.000
#define ki 0.0015
#define kpp 80.00
#define kdp 0.00
#define kip 0.00

static volatile uint8_t BEGIN = 0;
static volatile uint_fast16_t timerAcaptureValueR;      //右Timer时间捕获数
static volatile uint_fast16_t timerAcaptureValueL;      //左Timer时间捕获数
static volatile uint_fast16_t timerAcaptureValueRt;     //右Timer临时时间捕获数
static volatile uint_fast16_t timerAcaptureValueLt;     //左Timer临时时间捕获数
static volatile int32_t counterL, counterR;             //左右测速计数
static volatile uint_fast8_t dirR;                      //右方向
static volatile uint_fast8_t dirL;                      //左方向
static volatile double speedR = 0, speedL = 0;          //实际速度
static volatile int32_t periodL, periodR;               //速度捕获间隔
static volatile uint8_t R4, R3, R2, R1, L1, L2, L3, L4; //红外标志，TRUE为黑色
static volatile uint16_t velocity = 500;                //基准速度，最高速度约1450
static volatile double velocityR = 000;                 //右目标速度
static volatile double velocityL = 700;                 //左目标速度
static volatile double eR0 = 0, eR1 = 0, eR2 = 0;       //目标速度减当前速度差
static volatile double eL0 = 0, eL1 = 0, eL2 = 0;       //目标速度减当前速度差
static volatile int32_t outR = 0, outL = 0;             //左右PID输出
static volatile double err0 = 0, err1 = 0, err2 = 0;    //位置偏差
static volatile double out = 0;                         //位置输出

uint8_t Uint_ASCII(uint32_t u32, uint8_t *str);
void UART_transmitUInt(uint32_t moduleInstance, uint32_t data);
void UART_transmitFloat(uint32_t moduleInstance, double data);
void UART_transmitStr(uint32_t moduleInstance, uint8_t *data);

/* Timer_A UpDown PWM输出范围1-600 每周期0.05ms Timer_A0 */
const Timer_A_UpDownModeConfig upDownConfig =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock SOurce
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 12MHz
        TIMER0_PERIOD,                       // 600 tick period 0.05ms
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                     // Clear value
};

/* Timer_A Compare 右PWM输出 CCR3 */
Timer_A_CompareModeConfig compareConfig_PWM_R = {
    TIMER_A_CAPTURECOMPARE_REGISTER_3,        // Use CCR3
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,          // Toggle output resett
    DUTY_CYCLER                               // 150/600 Duty Cycle 25%
};

/* Timer_A Compare 左PWM输出 CCR4 */
Timer_A_CompareModeConfig compareConfig_PWM_L = {
    TIMER_A_CAPTURECOMPARE_REGISTER_4,        // Use CCR4
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,          // Toggle output reset
    DUTY_CYCLEL                               // 400/600 Duty Cycle 75%
};

/* Timer_A up PID周期中断 Timer_A1 */
const Timer_A_UpModeConfig upConfig_PID =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_4,      // SMCLK/4 = 3MHz
        TIMER1_PERIOD * 1,                  // 3000/1ms tick period  PID周期最长80ms
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                    // Clear value
};

/* Timer_A up 红外周期中断 Timer_A2 */
const Timer_A_UpModeConfig upConfig_Sensor =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // SMCLK/1 = 12MHz
        TIMER1_PERIOD * 12,                 // 3000/0.25ms tick period  红外周期最长5ms 测速最长11ms
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                    // Clear value
};

/* Timer_A Continuous Mode 速度测量 Timer_A3 */
const Timer_A_ContinuousModeConfig continuousModeConfig =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,      // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_2,  // SMCLK/2 = 6MHz    最长计11ms
        TIMER_A_TAIE_INTERRUPT_DISABLE, // Disable Timer ISR
        TIMER_A_SKIP_CLEAR              // Skip Clear Counter
};

/* Timer_A Capture Mode 右测速计数上升沿触发 CCR0 */
const Timer_A_CaptureModeConfig captureConfig_Counter_R =
    {
        TIMER_A_CAPTURECOMPARE_REGISTER_0,       // CCR1
        TIMER_A_CAPTUREMODE_RISING_EDGE,         // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,       // CCIxA Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,             // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE, // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE           // Output bit value
};

/* Timer_A Capture Mode 左测速计数上升沿触发 CCR2 */
const Timer_A_CaptureModeConfig captureConfig_Counter_L =
    {
        TIMER_A_CAPTURECOMPARE_REGISTER_2,       // CCR2
        TIMER_A_CAPTUREMODE_RISING_EDGE,         // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,       // CCIxA Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,             // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE, // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE           // Output bit value
};

// http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
const eUSCI_UART_ConfigV1 uartConfig115200 =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // SMCLK Clock Source
        6,                                             // BRDIV
        8,                                             // UCxBRF
        32,                                            // UCxBRS
        EUSCI_A_UART_NO_PARITY,                        // No Parity
        EUSCI_A_UART_LSB_FIRST,                        // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
        EUSCI_A_UART_MODE,                             // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
        EUSCI_A_UART_8_BIT_LEN                         // 8 bit data length
};

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    /* DCO 48MHz upping Vcore */
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    /* SMCLK 12MHz */
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4);

    volatile uint32_t i, j; //计数用

    //P1.1中断，左开关
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    MAP_Interrupt_enableInterrupt(INT_PORT1);

    /* pins UART mode UART0、2、3 */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                   GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                                   GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9,
                                                   GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig115200);
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig115200);
    MAP_UART_initModule(EUSCI_A3_BASE, &uartConfig115200);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableModule(EUSCI_A2_BASE);
    MAP_UART_enableModule(EUSCI_A3_BASE);

    /* Enabling UART interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
    MAP_UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA3);

    /* Timer_A0 UpDown Mode and compare mode for PWM */
    MAP_Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, //P2.7 and P2.6  PWM输出引脚
                                                    GPIO_PIN6 + GPIO_PIN7,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_R);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_L);
    compareConfig_PWM_R.compareValue = 0;
    compareConfig_PWM_L.compareValue = 0; //电机初始停止
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);

    /* Timer_A1 Up Mode CCR0 PID */
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig_PID);
    MAP_Interrupt_enableInterrupt(INT_TA1_0);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

    /* Timer_A2 Up Mode CCR0 红外周期中断 */
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig_Sensor);
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Timer_A3 continuous Mode capture mode 测速 */
    MAP_Timer_A_configureContinuousMode(TIMER_A3_BASE, &continuousModeConfig);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN4,
                                                   GPIO_PRIMARY_MODULE_FUNCTION); //P10.4 and P8.2
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN2,       //peripheral Inputs
                                                   GPIO_PRIMARY_MODULE_FUNCTION); //测速中断
    MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P10, GPIO_PIN5);
    MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P9, GPIO_PIN2); // P9.2 P10.5 方向判断
    MAP_Timer_A_initCapture(TIMER_A3_BASE, &captureConfig_Counter_L);
    MAP_Timer_A_initCapture(TIMER_A3_BASE, &captureConfig_Counter_R);
    MAP_Interrupt_enableInterrupt(INT_TA3_N);
    MAP_Interrupt_enableInterrupt(INT_TA3_0);
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_CONTINUOUS_MODE);

    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();

    //板载LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); //单红灯
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); //蓝
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1); //绿
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); //红三色灯
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    //电机方向
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6); //右方向
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7); //左方向

    //红外灯
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);

    //电机初始停止
    MAP_Interrupt_disableInterrupt(INT_TA1_0);
    compareConfig_PWM_R.compareValue = 0;
    compareConfig_PWM_L.compareValue = 0;
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_R);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_L);

    MAP_GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN7);

    uint8_t err = 50;
    while (1)
    {
        // if (speedL > velocityL + err)
        // {
        //     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); //正转红灯亮过高
        // }
        // else
        // {
        //     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        // }
        // if (speedL < velocityL - err)
        // {
        //     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); //正转蓝灯亮过低
        // }
        // else
        // {
        //     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        // }

        // UART_transmitFloat(EUSCI_A0_BASE, speedL);
        // UART_transmitStr(EUSCI_A0_BASE, "\r\n");

        // if (R4) //高压亮红灯 黑色
        // {
        //     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        //     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        // }
        // else //低压亮蓝灯 白色
        // {
        //     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
        //     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        // }
        uint8_t BEGINp = 0;
        if (BEGIN != BEGINp)
        {
            BEGINp = BEGIN;
            if (BEGIN)
            {
                MAP_Interrupt_enableInterrupt(INT_TA1_0);
            }
            else
            {
                //电机停止
                MAP_Interrupt_disableInterrupt(INT_TA1_0);
                compareConfig_PWM_R.compareValue = 0;
                compareConfig_PWM_L.compareValue = 0;
                MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_R);
                MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_L);
            }
        }
    }
}

/* GPIO_port1 ISR 电机启动开关 */
void PORT1_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
    if (status & GPIO_PIN1)
    {
        BEGIN = !BEGIN;
    }
}

/* Timer_A1_CCR0 PID */
void TA1_0_IRQHandler(void)
{
    uint32_t status = MAP_Timer_A_getCaptureCompareInterruptStatus(TIMER_A1_BASE,
                                                                   TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                                                   TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);
    if (!status)
        return;
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                                             TIMER_A_CAPTURECOMPARE_REGISTER_0);
    err2 = err1;
    err1 = err0;
    err0 = -8 * L4 + -4 * L3 + -2 * L2 + -1 * L1 + 1 * R1 + 2 * R2 + 4 * R3 + 8 * R4;

    out = out + kpp * (err0 - err1) + kip * err0 + kdp * (err0 - 2 * err1 + err2);
    velocityL = (velocity + out);
    velocityR = (velocity - out);

    eL2 = eL1;
    eL1 = eL0;
    eR2 = eR1;
    eR1 = eR0;
    eL0 = velocityL - speedL;
    eR0 = velocityR - speedR;
    outL = outL + kp * (eL0 - eL1) + ki * eL0 + kd * (eL0 - 2 * eL1 + eL2);
    outR = outR + kp * (eR0 - eR1) + ki * eR0 + kd * (eR0 - 2 * eR2 + eR2);

    if (outL <= 599 && outL >= 0)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7); //左低电平正转
        compareConfig_PWM_L.compareValue = outL;
    }
    else if (outL > 599)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7); //左低电平正转
        compareConfig_PWM_L.compareValue = 599;
    }
    else if (outL < 0 && outL >= -599)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7); //左低电平正转
        compareConfig_PWM_L.compareValue = -outL;
    }
    else
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7); //左低电平正转
        compareConfig_PWM_L.compareValue = 599;
    }

    if (outR <= 599 && outR > 0)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); //右低电平正转
        compareConfig_PWM_R.compareValue = outR;
    }
    else if (outR > 599)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); //右低电平正转
        compareConfig_PWM_R.compareValue = 599;
    }
    else if (outR < 0 && outR >= -599)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); //右低电平正转
        compareConfig_PWM_R.compareValue = -outR;
    }
    else
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); //右低电平正转
        compareConfig_PWM_R.compareValue = 599;
    }
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_R);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_L);
}

/* Timer_A2_CCR0 红外 */
void TA2_0_IRQHandler(void)
{
    uint32_t status = MAP_Timer_A_getCaptureCompareInterruptStatus(TIMER_A2_BASE,
                                                                   TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                                                   TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG);
    if (!status)
        return;
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
                                             TIMER_A_CAPTURECOMPARE_REGISTER_0);
    uint32_t i = 0;
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7);

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN7); //充能升压

    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN7); //停止升压并准备测量
    for (i = 0; i < 2500; i++)                       //降压，跑道越脏事件应越长，降低对黑色的敏感度，2500跑道内可以识别，4500跑道外地板识别为白色
    {
    }

    //测量
    L4 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN7); //最左边
    L3 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN6);
    L2 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN5);
    L1 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN4);
    R1 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN3);
    R2 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN2);
    R3 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN1);
    R4 = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN0);
    periodR = timerAcaptureValueRt - timerAcaptureValueR + 1;
    periodL = timerAcaptureValueLt - timerAcaptureValueL + 1;
    timerAcaptureValueL = timerAcaptureValueLt;
    timerAcaptureValueR = timerAcaptureValueRt;
    if (periodR < 0)
    {
        periodR = 65536 - periodR;
    }
    if (periodL < 0)
    {
        periodL = 65536 - periodL;
    }
    speedR = (double)counterR / periodR * 12000000.0 / 2;
    speedL = (double)counterL / periodL * 12000000.0 / 2;
    counterL = counterR = 0;
}

/* Timer_A3_CCR0 右测速 */
void TA3_0_IRQHandler(void)
{
    dirR = !MAP_GPIO_getInputPinValue(GPIO_PORT_P10, GPIO_PIN5);
    timerAcaptureValueRt = MAP_Timer_A_getCaptureCompareCount(TIMER_A3_BASE,
                                                              TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    counterR += (dirR ? 1 : -1);
}

/* Timer_A3_CCR2 左测速 */
void TA3_N_IRQHandler(void)
{
    dirL = MAP_GPIO_getInputPinValue(GPIO_PORT_P9, GPIO_PIN2);
    timerAcaptureValueLt = MAP_Timer_A_getCaptureCompareCount(TIMER_A3_BASE,
                                                              TIMER_A_CAPTURECOMPARE_REGISTER_2);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
    counterL += (dirL ? 1 : -1);
}

/* EUSCI A0 UART ISR */
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t data = MAP_UART_receiveData(EUSCI_A0_BASE);
        MAP_UART_transmitData(EUSCI_A0_BASE, 'A');
        MAP_UART_transmitData(EUSCI_A0_BASE, '0');
        MAP_UART_transmitData(EUSCI_A0_BASE, ':');
        MAP_UART_transmitData(EUSCI_A0_BASE, data);
        MAP_UART_transmitData(EUSCI_A0_BASE, '\r');
        MAP_UART_transmitData(EUSCI_A0_BASE, '\n');
        MAP_UART_transmitData(EUSCI_A2_BASE, data);
    }
}

/* EUSCI A2 UART ISR */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t data = MAP_UART_receiveData(EUSCI_A2_BASE);
        MAP_UART_transmitData(EUSCI_A0_BASE, 'A');
        MAP_UART_transmitData(EUSCI_A0_BASE, '2');
        MAP_UART_transmitData(EUSCI_A0_BASE, ':');
        MAP_UART_transmitData(EUSCI_A0_BASE, data);
        MAP_UART_transmitData(EUSCI_A0_BASE, '\r');
        MAP_UART_transmitData(EUSCI_A0_BASE, '\n');
        MAP_UART_transmitData(EUSCI_A3_BASE, data);
    }
}

/* EUSCI A3 UART ISR */
void EUSCIA3_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A3_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t data = MAP_UART_receiveData(EUSCI_A3_BASE);
        MAP_UART_transmitData(EUSCI_A0_BASE, 'A');
        MAP_UART_transmitData(EUSCI_A0_BASE, '3');
        MAP_UART_transmitData(EUSCI_A0_BASE, ':');
        MAP_UART_transmitData(EUSCI_A0_BASE, data);
        MAP_UART_transmitData(EUSCI_A0_BASE, '\r');
        MAP_UART_transmitData(EUSCI_A0_BASE, '\n');
    }
}

/* 将 uint32_t uint16_t 转化为 ASCII 编码的 Uint8_t 数组进行输出，第一个参数为待转化数，第二个参数为结果存贮位置 */
uint8_t Uint_ASCII(uint32_t u32, uint8_t *str)
{
    uint8_t temp[10];
    uint8_t i = 0, j = 0;
    while (u32)
    {
        temp[i] = u32 % 10 + 0x30;
        i++;
        u32 /= 10;
    }
    j = i;
    for (i = 0; i < j; i++)
    {
        str[i] = temp[j - i - 1];
    }
    return i;
}

/* 将 Uint 按 ASCII 进行串口输出 */
void UART_transmitUInt(uint32_t moduleInstance, uint32_t data)
{
    uint8_t datas[10];
    uint8_t i, j;
    j = Uint_ASCII(data, datas);
    for (i = 0; i < j; i++)
    {
        MAP_UART_transmitData(moduleInstance, datas[i]);
    }
}

/* 将浮点数进行 ASCII 串口输出 */
void UART_transmitFloat(uint32_t moduleInstance, double data)
{
    uint8_t pos = (data >= 0);
    if (data < 0)
        data = -data;
    int32_t intPart = (uint32_t)data;
    double floatPart = data - intPart;
    uint32_t intFloatPart = (uint32_t)(floatPart * 1000000);
    if (!pos)
    {
        MAP_UART_transmitData(moduleInstance, '-');
    }
    UART_transmitUInt(moduleInstance, intPart);
    MAP_UART_transmitData(moduleInstance, '.');
    UART_transmitUInt(moduleInstance, intFloatPart);
}

/* 对C风格字符串 进行串口输出 */
void UART_transmitStr(uint32_t moduleInstance, uint8_t *data)
{
    uint8_t i = 0;
    while (*(data + i) != '\0')
    {
        MAP_UART_transmitData(moduleInstance, *(data + i));
        i++;
    }
}
