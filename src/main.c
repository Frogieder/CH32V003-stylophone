/*
 * WIP version of EEE
 *
 * Based on examples by Ned Konz and E. Brombaugh, using cnlohr's ch32fun
 *
 * Low:
 * A  933
 * A# 857
 * B  788
 * C  620
 * C# 548
 * D  477
 * D# 286
 * E  210
 * F  116
 * F# 0
 * HIGH:
 * G  0
 * G# 117
 * A  211
 * A# 287
 * B  444
 * C  548
 * C# 619
 * D  788
 * D# 857
 * E  932
 */

#include <ch32fun.h>
#include <stdbool.h>

#define AR_POW 13

const int8_t sintab[] = {
    0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 59, 62, 65, 67, 70, 73, 75, 78, 80, 82,
    85, 87, 89, 91, 94, 96, 98, 100, 102, 103, 105, 107, 108, 110, 112, 113, 114, 116, 117, 118, 119, 120, 121, 122,
    123, 123, 124, 125, 125, 126, 126, 126, 126, 126, 127, 126, 126, 126, 126, 126, 125, 125, 124, 123, 123, 122, 121,
    120, 119, 118, 117, 116, 114, 113, 112, 110, 108, 107, 105, 103, 102, 100, 98, 96, 94, 91, 89, 87, 85, 82, 80, 78,
    75, 73, 70, 67, 65, 62, 59, 57, 54, 51, 48, 45, 42, 39, 36, 33, 30, 27, 24, 21, 18, 15, 12, 9, 6, 3, 0, -3, -6, -9,
    -12, -15, -18, -21, -24, -27, -30, -33, -36, -39, -42, -45, -48, -51, -54, -57, -59, -62, -65, -67, -70, -73, -75,
    -78, -80, -82, -85, -87, -89, -91, -94, -96, -98, -100, -102, -103, -105, -107, -108, -110, -112, -113, -114, -116,
    -117, -118, -119, -120, -121, -122, -123, -123, -124, -125, -125, -126, -126, -126, -126, -126, -127, -126, -126,
    -126, -126, -126, -125, -125, -124, -123, -123, -122, -121, -120, -119, -118, -117, -116, -114, -113, -112, -110,
    -108, -107, -105, -103, -102, -100, -98, -96, -94, -91, -89, -87, -85, -82, -80, -78, -75, -73, -70, -67, -65, -62,
    -59, -57, -54, -51, -48, -45, -42, -39, -36, -33, -30, -27, -24, -21, -18, -15, -12, -9, -6, -3
};

const int8_t dubreq_stylophone_sound[157] = {
    75, 75, 74, 71, 69, 68, 71, 74, 74, 70, 63, 59, 63, 72, 79, 78, 73, 69, 68, 69, 69, 69, 68, 68, 66, 60, 50, 41, 31,
    22, 16, 12, 5, -2, -11, -22, -35, -46, -50, -52, -54, -58, -66, -76, -87, -97, -105, -109, -112, -117, -123, -127,
    -127, -125, -125, -126, -125, -120, -114, -109, -104, -100, -99, -99, -101, -104, -109, -111, -106, -98, -93, -92,
    -95, -100, -102, -96, -85, -75, -70, -63, -57, -56, -60, -63, -59, -49, -40, -39, -48, -58, -61, -57, -50, -46, -44,
    -43, -42, -42, -43, -41, -33, -22, -12, -6, -6, -9, -8, -2, 6, 14, 20, 25, 28, 28, 28, 30, 33, 38, 45, 52, 56, 57,
    59, 64, 67, 71, 77, 82, 85, 89, 95, 101, 104, 103, 99, 95, 96, 101, 107, 110, 105, 98, 92, 89, 85, 80, 77, 76, 75,
    73, 69, 64, 64, 68, 73, 75, 75
};

#define MAGIC_NUMBER 69420

#define OCTAVE_BUTTON PD5

volatile uint16_t adc_buffer[2] = {420, 69};

volatile uint32_t omega = 0;
volatile bool buzzer_state = false;

// Frequency * 10
const int frequencies[] = {
    4400, 4662, 4939, 5233, 5544, 5873, 6223, 6593, 6985, 7400, 7840, 8306,
    8800, 9323, 9878, 10465, 11087, 11747, 12445, 13185
    // 8800, 9324, 9868, 10466, 11088, 11746, 12446, 13186, 13970, 14800,
    // 15680, 16612, 17600, 18646, 19756, 20930, 22174, 23494, 24890, 26370
};

int find_bin(const int x)
{
    const int boundaries[] = {0, 59, 165, 249, 368, 500, 588, 707, 823, 895, 978};

    // find the first boundary that is strictly greater than x
    for (int i = 0; i < 11; ++i)
    {
        if (x < boundaries[i])
        {
            return i - 1; // one bin below this boundary
        }
    }

    // if x is >= last boundary, return the last index
    return -1;
}

/*
 * initialize TIM1 for PWM
 * We use ATRLR to keep track of phase and PSC determines the angular velocity (or rather its inverse)
 */
void t1pwm_init(void)
{
    // Enable GPIOA, TIM1, and AFIO clocks
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO;

    // Re-map AFIO to allow PC3=TIM1_CH1N and PC6=TIM1_CH1 to be used
    AFIO->PCFR1 &= ~AFIO_PCFR1_TIM1_REMAP_FULLREMAP;
    // AFIO->PCFR1 |= AFIO_PCFR1_TIM1_REMAP_PARTIALREMAP2;


    funGpioInitA();
    // PC3 is T1CH1_N, 10MHz Output alt func, push-pull
    funPinMode(PA1, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF);

    // Reset TIM1 to init all regs
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    // CTLR1: default is up, events generated, edge align
    TIM1->CTLR1 = 0;

    // CTLR2: set output idle states (MOE off) via OIS1 and OIS1N bits
    TIM1->CTLR2 = 0;

    // SMCFGR: default clk input is 48MHz CK_INT

    // We won't use a prescaler (okay, we will...)
    TIM1->PSC = 0;

    // This results in 187.6 kHz PWM or 4.69 cycles per update
    TIM1->ATRLR = 256;

    // Reload immediately
    TIM1->SWEVGR |= TIM_UG;

    // Enable CH1 output, positive polarity
    TIM1->CCER |= TIM_CC2E | TIM_CC2P;

    // Enable CH1N output, positive polarity
    TIM1->CCER |= TIM_CC2NE | TIM_CC2NP;

    // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;

    // Set the Capture Compare Register to be half of the ATRLR
    TIM1->CH2CVR = 128;

    // Enable TIM1 outputs (also see OSSI and OSSR bits)
    TIM1->BDTR |= TIM_MOE;

    // Enable TIM1
    TIM1->CTLR1 |= TIM_CEN;
}


void tim2_init(void)
{
    // Enable TIM2
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
    // Reset TIM2
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;
    Delay_Us(10);

    // funGpioInitA();
    // PC3 is T1CH1_N, 10MHz Output alt func, push-pull
    // funPinMode(PA1, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);

    // set TIM2 clock prescaler divider
    TIM2->PSC = 0;
    // set PWM total cycle width
    TIM2->ATRLR = 2000;

    TIM2->CH1CVR = 1000;

    TIM2->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE;
    TIM2->CCER |= TIM_CC1E | (TIM_CC1P);

    // CTLR1: default is up, events generated, edge align
    // enable auto-reload of preload
    TIM2->CTLR1 |= TIM_ARPE;

    // initialize counter
    TIM2->SWEVGR |= TIM_UG;

    // Enable interrupt, I hope... Please, do...
    TIM2->DMAINTENR |= TIM_CC1IE;
    // TIM2->DMAADR != TIM_UIE;

    NVIC_EnableIRQ(TIM2_IRQn); // Event interrupt
    NVIC_SetPriority(TIM2_IRQn, 2 << 4);

    // Enable TIM2
    TIM2->CTLR1 |= TIM_CEN;
}

// Update PWM DAC values; this is called 48-thousand times per second
void TIM2_IRQHandler(void) __attribute__((interrupt));

void TIM2_IRQHandler(void)
{
    TIM2->INTFR &= ~TIM_CC1IF;

    uint32_t mix;

    // static uint16_t phase1, phase2 = 0;
    // phase1 += 1 << 8;
    // phase2 += 1 << 8 + 5;
    //
    // mix = (phase1 + phase2) >> (1 + 8);

    /* Sound generator */
    static uint32_t phase = 0;
    phase += omega;
    if (phase >= (157 << 16))
        phase -= 157 << 16;
    mix = dubreq_stylophone_sound[phase >> 16] + 128;
    /* End of sound generator */

    // AR (no delay or sustain)
    static uint32_t AR_state = 0;
    if (buzzer_state)
    {
        AR_state += (AR_state < (1 << AR_POW)) << 10;
    }
    else
    {
        AR_state -= (AR_state > 0);
    }
    mix = (AR_state * mix) >> AR_POW;

    TIM1->CH2CVR = mix;
}


void adc_init(void)
{
    // ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
    RCC->CFGR0 &= ~(0x1F << 11);;
    // Enable GPIOD and ADC
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1;

    funGpioInitC(); // TODO: redundant?
    funGpioInitD();
    funPinMode(PC4, GPIO_CNF_IN_ANALOG)
    funPinMode(PA2, GPIO_CNF_IN_ANALOG)

    // Reset the ADC to init all regs
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

    // Set up conversion on channels 2 and 5 (or 0 for testing)
    ADC1->RSQR1 = (2 - 1) << 20; // Two channels
    ADC1->RSQR2 = 0;
    ADC1->RSQR3 = (0 << (5 * 0))
        | (2 << (5 * 1));

    // set sampling time for channels 2 and 5 (and 0)
    // 0:7 => 3/9/15/30/43/57/73/241 cycles
    // since it'll be handled in the background, we can afford slower conversions
    ADC1->SAMPTR2 = 7 << (3 * 2) | 7 << (3 * 5) | 7 << (3 * 0);

    // turn on ADC
    ADC1->CTLR2 |= ADC_ADON;

    // Reset calibration
    ADC1->CTLR2 |= ADC_RSTCAL;
    while (ADC1->CTLR2 & ADC_RSTCAL)
    {
    }

    // Calibrate
    ADC1->CTLR2 |= ADC_CAL;
    while (ADC1->CTLR2 & ADC_CAL)
    {
    }

    // Turn on DMA
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

    //DMA1_Channel1 is for ADC
    DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
    DMA1_Channel1->MADDR = (int32_t)adc_buffer;
    DMA1_Channel1->CNTR = 2; // Number of data registers
    DMA1_Channel1->CFGR =
        DMA_M2M_Disable |
        DMA_Priority_High |
        DMA_MemoryDataSize_HalfWord |
        DMA_PeripheralDataSize_HalfWord |
        DMA_MemoryInc_Enable |
        DMA_Mode_Circular |
        DMA_DIR_PeripheralSRC;

    // Turn on DMA channel 1
    DMA1_Channel1->CFGR |= DMA_CFGR1_EN;

    // enable scanning
    ADC1->CTLR1 |= ADC_SCAN;

    // Enable continuous conversion and DMA
    ADC1->CTLR2 |= ADC_CONT | ADC_DMA | ADC_EXTSEL;

    // start conversion
    ADC1->CTLR2 |= ADC_SWSTART;
}

void set_frequency(const uint32_t frequency)
{
    omega = (frequency << 16) / (10 * 24000 / 157);
}

void stop_buzzer()
{
    // TIM1->BDTR &= ~TIM_MOE;
    buzzer_state = false;
}

void start_buzzer()
{
    // TIM1->BDTR |= TIM_MOE;
    buzzer_state = true;
}

enum KeyboardState {
    SUSTAINING_TONE,
    CHANGING_TONE,
};

enum KeyboardState state = SUSTAINING_TONE;

/*
 * entry
 */
int main()
{
    SystemInit();
    Delay_Ms(100);

    // init TIM1 for PWM
    t1pwm_init();
    tim2_init();
    adc_init();
    // funPinMode(OCTAVE_BUTTON,  GPIO_Speed_10MHz | GPIO_CNF_IN_PUPD);
    funGpioInitD();
    funPinMode(OCTAVE_BUTTON, GPIO_CNF_IN_PUPD);
    funDigitalWrite(OCTAVE_BUTTON, 1);

    uint32_t previous_frequency = 0;
    uint32_t sustained_frequency = MAGIC_NUMBER;
    uint32_t last_timestamp = 0;

    Delay_Ms(100);

    // ReSharper disable once CppDFAEndlessLoop
    for (;;)
    {
        /*
         * Sound generator
         */
        uint32_t value = find_bin(adc_buffer[0]);
        uint32_t frequency = MAGIC_NUMBER;
        if (value != -1)
        {
            frequency = frequencies[9 - value];
        }
        else
        {
            value = find_bin(adc_buffer[1]);
            if (value != -1)
            {
                frequency = frequencies[value + 10];
            }
        }

        switch (state)
        {
        case SUSTAINING_TONE:
            if (frequency != sustained_frequency)
            {
                state = CHANGING_TONE;
                last_timestamp = SysTick->CNT;
            }
            break;
        case CHANGING_TONE:
            if (frequency == sustained_frequency)
            {
                state = SUSTAINING_TONE;
                break;
            }
            if (frequency != previous_frequency)
            {
                last_timestamp = SysTick->CNT;
                break;
            }

            // Check if we have waited for at least 5 milliseconds
            if (SysTick->CNT - last_timestamp > 40000)
            {
                state = SUSTAINING_TONE;
                sustained_frequency = frequency;
                break;
            }
            break;
        }
        previous_frequency = frequency;

        if (sustained_frequency != MAGIC_NUMBER)
        {
            start_buzzer();
            set_frequency(!funDigitalRead(OCTAVE_BUTTON) ? sustained_frequency << 1 : sustained_frequency);
        }
        else
        {
            stop_buzzer();
        }
        // Delay_Ms(10);
    }
}
