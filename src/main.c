#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include "rv003usb.h"

#define MAGIC_NUMBER 9
#define OCTAVE_BUTTON PD5

volatile uint16_t adc_buffer[2] = {420, 69};

typedef struct {
	volatile uint8_t len;
	uint8_t buffer[8];
} midi_message_t;

midi_message_t midi_in;

// midi data from device -> host (IN)
void midi_send(uint8_t * msg, uint8_t len)
{
	memcpy( midi_in.buffer, msg, len );
	midi_in.len = len;
}
#define midi_send_ready() (midi_in.len == 0)

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


// midi data from host -> device (OUT)
void midi_receive(uint8_t * msg)
{
    /*
	static uint8_t note;

	if (msg[1] == 0x90 && msg[3] !=0) { // Note-on, velocity nonzero
		note = msg[2];
		tim2_set_period( noteLookup[ note ] );
	}

	// Note-off or note-on with zero velocity
	else if (msg[2] == note && (msg[1]==0x80 || (msg[1]==0x90 && msg[3]==0)) ) {
		tim2_set_period(0);
	}
	*/
}

void switch_tones(const uint8_t old, const uint8_t new)
{

	if (old != MAGIC_NUMBER)
	{
		uint8_t midi_msg[4] = {0x08, 0x90, old, 0};
		while (!midi_send_ready()) {};
		midi_send(midi_msg, 4);
	}
	if (new != MAGIC_NUMBER)
	{
		uint8_t midi_msg[4] = {0x09, 0x90, new, 0x7F};
		while (!midi_send_ready()) {};
		midi_send(midi_msg, 4);
	}
}

enum KeyboardState {
    SUSTAINING_TONE,
    CHANGING_TONE,
};

enum KeyboardState state = SUSTAINING_TONE;

int main()
{
	SystemInit();
	adc_init();
	usb_setup();

    funGpioInitD();
    funPinMode(OCTAVE_BUTTON, GPIO_CNF_IN_PUPD);
    funDigitalWrite(OCTAVE_BUTTON, 1);

    uint8_t sustained_tone = MAGIC_NUMBER;
    uint8_t previous_tone = MAGIC_NUMBER;
    uint32_t last_timestamp = 0;

	while(1)
	{
        uint32_t value = find_bin(adc_buffer[0]);
        uint8_t tone = MAGIC_NUMBER;
        if (value != -1)
        {
            // A4 + tone offset
            tone = 69 + (9 - value);
        }
        else
        {
            value = find_bin(adc_buffer[1]);
            if (value != -1)
            {
                tone = 69 + 10 + value;
            }
        }
	    if (tone != MAGIC_NUMBER && !funDigitalRead(OCTAVE_BUTTON))
	    {
	        tone += 12;
	    }

        switch (state)
        {
        case SUSTAINING_TONE:
            if (tone != sustained_tone)
            {
                state = CHANGING_TONE;
                last_timestamp = SysTick->CNT;
            }
            break;
        case CHANGING_TONE:
            if (tone == sustained_tone)
            {
                state = SUSTAINING_TONE;
                break;
            }
            if (tone != previous_tone)
            {
                last_timestamp = SysTick->CNT;
                break;
            }

            // Check if we have waited for at least 5 milliseconds
            if (SysTick->CNT - last_timestamp > 40000)
            {
                // Change tone
            	switch_tones(sustained_tone, tone);
                sustained_tone = tone;
                state = SUSTAINING_TONE;
                break;
            }
            break;
        }
        previous_tone = tone;
	}
}

void usb_handle_user_in_request( struct usb_endpoint * e, uint8_t * scratchpad, int endp, uint32_t sendtok, struct rv003usb_internal * ist )
{
	if (endp && midi_in.len) {
		usb_send_data( midi_in.buffer, midi_in.len, 0, sendtok );
		midi_in.len = 0;
	} else {
		usb_send_empty( sendtok );
	}
}

void usb_handle_other_control_message( struct usb_endpoint * e, struct usb_urb * s, struct rv003usb_internal * ist )
{
	LogUEvent( SysTick->CNT, s->wRequestTypeLSBRequestMSB, s->lValueLSBIndexMSB, s->wLength );
}

void usb_handle_user_data( struct usb_endpoint * e, int current_endpoint, uint8_t * data, int len, struct rv003usb_internal * ist )
{
	if (len) midi_receive(data);
	if (len==8) midi_receive(&data[4]);
}



