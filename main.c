// TODO not working
#define F_CPU 4000000UL

#include <avr/io.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <util/delay.h>

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static int uart_putchar(char c, FILE *stream) {
  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(USART0.STATUS, USART_DREIF_bp);
  USART0.TXDATAL = c;
  return 0;
}

void initialize() {
    /*PORTA.DIR = 0b01010001; // USART*/
    PORTA.DIR = 0b01110001; // PWM
    PORTC.DIR = 0b00001101;
    PORTD.DIR = 0b00000000;
    PORTF.DIR = 0b00000000;

    USART0.BAUD = 138; // 115200 if F_CPU = 4MHz
    USART0.CTRLA = 0b00000000;
    USART0.CTRLC = 0x03; //default
    // cannot be enabled with PWM LEDs
    /*PORTMUX.USARTROUTEA = 0b00000001;*/
    /*USART0.CTRLB = 0b11000000;*/

    PORTMUX.TCDROUTEA = 0x4;
    TCD0.CMPASET = 2000;
    TCD0.CMPACLR = 2047;
    TCD0.CMPBSET = 4000;
    TCD0.CMPBCLR = 4095;

    CPU_CCP = CCP_IOREG_gc;
    TCD0.FAULTCTRL = TCD_CMPAEN_bm | TCD_CMPBEN_bm;
    loop_until_bit_is_set(TCD0.STATUS, TCD_ENRDY_bp);
    TCD0.CTRLA=1;

    VREF.ADC0REF = 0x5; // Vdd
    VREF.DAC0REF = 0x5; // Vdd

    ADC0.CTRLB = 0x4;
    ADC0.CTRLA = 0b1;

    DAC0.CTRLA = 0b01000001;

    PORTMUX.SPIROUTEA = 0x3;
    SPI0.CTRLB = 0b00000100;
    /*SPI0.CTRLA = 0b00100001;*/

    stdout = &mystdout;
    stderr = &mystdout;
}

const uint8_t flips[15][2] = {{0, 0}, {7, 9}, {5, 11}, {2, 7}, {5, 7}, {4, 13}, {1, 2}, {3, 4}, {11, 6}, {3, 0}, {3, 1}, {4, 12}, {6, 2}, {13, 5}, {3, 6}, {1, 14}};

/*void print_bin(uint16_t u16) {*/
    /*int i = 16;*/
    /*while(i--) {*/
        /*uart_putchar('0' + ((u16 >> i) & 1), NULL);*/
    /*}*/
/*}*/

void pwm_out_sync() {
  loop_until_bit_is_set(TCD0.STATUS, TCD_CMDRDY_bp);
  TCD0.CTRLE = 0b10;
}

uint16_t flip(uint16_t pattern, uint8_t step) {
  if (step == 15) {
    return pattern;
  }
  volatile uint8_t i = 1; //wtf
  uint16_t res = pattern;
  if (step < 15) {
    uint8_t numSteps = 15-step;
    for (; i <= numSteps; i++) {
      uint8_t f1 = 15-flips[i][0];
      uint8_t f2 = 15-flips[i][1];
      uint8_t a = (res >> f1) & 1;
      uint8_t b = (res >> f2) & 1;
      uint16_t x = a ^ b;
      x = (x << f1) | (x << f2);
      res = res ^ x;
    }
  } else {
    uint8_t numSteps = step-15;
    for (; i < numSteps; i++) {
      uint8_t f1 = flips[i][0];
      uint8_t f2 = flips[i][1];
      uint8_t a = (res >> f1) & 1;
      uint8_t b = (res >> f2) & 1;
      uint16_t x = a ^ b;
      x = (x << f1) | (x << f2);
      res = res ^ x;
    }
  }
  return res;
}

uint8_t shift_registers_io(uint8_t out) {
  // 1100
  // ^^_ SH/LD and RCLK
  // |__ CLK
  PORTC.OUTCLR = 0b1100; // both low
  volatile uint8_t i = 0;
  for (; i < 8; i++) {
    PORTC.OUTCLR = 0b1000; //clock low
    uint8_t high = (out >> i) & 1;
    if (high) {
      PORTA.OUTSET = 1;
    } else {
      PORTA.OUTCLR = 1;
    }
    PORTC.OUTSET = 0b1000; //clock high
  }
  PORTA.OUTCLR = 1;
  PORTC.OUTCLR = 0b1000; // clock low
  PORTC.OUTSET = 0b0100; // parallel load, parallel output
  PORTC.OUTSET = 0b1000; // clock high
  uint8_t sample = (PORTA.IN & 0b00000010) >> 1;
  uint8_t buttons = sample;
  PORTC.OUTCLR = 0b1000; // clock low
  i = 0;
  for (; i < 7; i++) {
    PORTC.OUTSET = 0b1000; //clock high
    sample = (PORTA.IN & 0b00000010) >> 1;
    buttons = (buttons << 1) | sample;
    PORTC.OUTCLR = 0b1000; //clock low
  }
  return buttons;
  return 0;
}

#define AIN_CVA 27 
#define AIN_CVB 1
#define AIN_FDA 2
#define AIN_PTA 3
#define AIN_SMA 4
#define AIN_FDB 5
#define AIN_ACV 7
#define AIN_ARD 23 
#define AIN_PTB 22 
#define AIN_SMB 16 

#define GATE_B  0b10000000
#define LED_A_F 0b00100000
#define LED_A_P 0b01000000
#define LED_A_S 0b00010000
#define LED_B_F 0b00000100
#define LED_B_P 0b00001000
#define LED_B_S 0b00000010
#define GATE_A  0b00000001

uint16_t read_adc(uint8_t muxpos) {
    /*ADC0.MUXPOS = 0x7;*/
    ADC0.MUXPOS = muxpos;
    ADC0.COMMAND = 1;
    loop_until_bit_is_set(ADC0.INTFLAGS, ADC_RESRDY_bp);
    uint16_t res = ADC0.RES;
    ADC0.COMMAND = 0;
    return res;
}

void write_dac(uint8_t channel, uint16_t value) {
  SPI0.CTRLA = 0b00100001;
  uint16_t data = (value >> 4) & 0b0000111111111111;
  uint8_t byteA = (data >> 8) & 0xFF;
  byteA = 0b00010000 | byteA | (channel << 7);
  uint8_t byteB = data & 0xFF;
  PORTA.OUTCLR = 0b01000000; //SS_DAC
  SPI0.DATA = byteA;
  loop_until_bit_is_set(SPI0.INTFLAGS, SPI_IF_bp);
  SPI0.DATA = byteB;
  loop_until_bit_is_set(SPI0.INTFLAGS, SPI_IF_bp);
  PORTA.OUTSET = 0b01000000;
  SPI0.CTRLA = 0;
}

uint16_t prngstate = 0;

//TODO initialize state to temp sensor reading?
uint16_t prng() {
  prngstate = (2053 * prngstate + 13849) % 0x10000;
  return prngstate;
}

uint16_t atten_cv_low;
uint16_t atten_cv_high;
uint16_t atten_rand_low;
uint16_t atten_rand_high;
uint16_t fade_a_min;
uint16_t fade_a_range;
uint16_t sample_a_min;
uint16_t sample_a_range;
uint16_t pattern_a_min;
uint16_t pattern_a_range;
uint16_t fade_b_min;
uint16_t fade_b_range;
uint16_t pattern_b_min;
uint16_t pattern_b_range;
uint16_t sample_b_min;
uint16_t sample_b_range;

uint16_t adjust(uint16_t reading, uint16_t min, uint16_t range) {
  uint16_t res = reading;
  if (res <= min) {
    res = 0;
  } else {
    res -= min;
  }
  if (res > range) {
    res = range;
  }
  /*printf("%u %u %u -> %u\n", reading, min, range, res);*/
  return res;
}

//TODO: if corruption becomes a noticeable issue, find some more storage and swap writes between two locations
struct seqstate {
  uint8_t seqA_start;
  uint8_t seqB_start;
  uint16_t seqAL[16]; 
  uint16_t seqAR[16]; 
  uint16_t seqBL[16]; 
  uint16_t seqBR[16]; 
  uint8_t index_a;
  uint8_t index_b;
} state;

int main(void) {
  initialize();
  uint8_t shift_out = 0;
  uint8_t clocklow_a = 0;
  uint8_t clocklow_b = 0;
  if (shift_registers_io(0) == 0xFF) {
    printf("Calibration mode babey!!!!!\n");
    _delay_ms(2000);
    while (!shift_registers_io(0xFF)) {
      fade_a_min = read_adc(AIN_FDA);
      pattern_a_min = read_adc(AIN_PTA);
      sample_a_min = read_adc(AIN_SMA);
      fade_b_min = read_adc(AIN_FDB);
      pattern_b_min = read_adc(AIN_PTB);
      sample_b_min = read_adc(AIN_SMB);
      atten_cv_low = read_adc(AIN_ACV);
      atten_rand_low = read_adc(AIN_ARD);
    }
    printf("Minimums set\n");
    _delay_ms(2000);
    while (!shift_registers_io(0x44)) {
      fade_a_range = read_adc(AIN_FDA)-fade_a_min;
      pattern_a_range = read_adc(AIN_PTA)-pattern_a_min;
      sample_a_range = read_adc(AIN_SMA)-sample_a_min;
      fade_b_range = read_adc(AIN_FDB)-fade_b_min;
      pattern_b_range = read_adc(AIN_PTB)-pattern_b_min;
      sample_b_range = read_adc(AIN_SMB)-sample_b_min;
      atten_cv_high = read_adc(AIN_ACV);
      atten_rand_high = read_adc(AIN_ARD);
    }
    printf("Maximums set\n");
    USERROW_t empty;
    while (NVMCTRL.STATUS & (NVMCTRL_EEBUSY_bm | NVMCTRL_FBUSY_bm));
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLPER_gc);
    while (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm);
    USERROW = empty;
    while (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm);
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
    while (NVMCTRL.STATUS & (NVMCTRL_EEBUSY_bm | NVMCTRL_FBUSY_bm));
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_FLWR_gc);
    *((uint16_t *)(&USERROW.USERROW0)) = fade_a_min;
    *((uint16_t *)(&USERROW.USERROW2)) = pattern_a_min;
    *((uint16_t *)(&USERROW.USERROW4)) = sample_a_min;
    *((uint16_t *)(&USERROW.USERROW6)) = fade_b_min;
    *((uint16_t *)(&USERROW.USERROW8)) = pattern_b_min;
    *((uint16_t *)(&USERROW.USERROW10)) = sample_b_min;
    *((uint16_t *)(&USERROW.USERROW12)) = atten_cv_low;
    *((uint16_t *)(&USERROW.USERROW14)) = atten_rand_low;
    *((uint16_t *)(&USERROW.USERROW16)) = fade_a_range;
    *((uint16_t *)(&USERROW.USERROW18)) = pattern_a_range;
    *((uint16_t *)(&USERROW.USERROW20)) = sample_a_range;
    *((uint16_t *)(&USERROW.USERROW22)) = fade_b_range;
    *((uint16_t *)(&USERROW.USERROW24)) = pattern_b_range;
    *((uint16_t *)(&USERROW.USERROW26)) = sample_b_range;
    *((uint16_t *)(&USERROW.USERROW28)) = atten_cv_high;
    *((uint16_t *)(&USERROW.USERROW30)) = atten_rand_high;
    while (NVMCTRL.STATUS & (NVMCTRL_EEBUSY_bm | NVMCTRL_FBUSY_bm));
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, 0);
    printf("Calibration values saved\n");
  } else {
    fade_a_min = (*((uint16_t *) &USERROW.USERROW0));
    pattern_a_min = (*((uint16_t *) &USERROW.USERROW2));
    sample_a_min = (*((uint16_t *) &USERROW.USERROW4));
    fade_b_min = (*((uint16_t *) &USERROW.USERROW6));
    pattern_b_min = (*((uint16_t *) &USERROW.USERROW8));
    sample_b_min = (*((uint16_t *) &USERROW.USERROW10));
    atten_cv_low = (*((uint16_t *) &USERROW.USERROW12));
    atten_rand_low = (*((uint16_t *) &USERROW.USERROW14));
    fade_a_range = (*((uint16_t *) &USERROW.USERROW16));
    pattern_a_range = (*((uint16_t *) &USERROW.USERROW18));
    sample_a_range = (*((uint16_t *) &USERROW.USERROW20));
    fade_b_range = (*((uint16_t *) &USERROW.USERROW22));
    pattern_b_range = (*((uint16_t *) &USERROW.USERROW24));
    sample_b_range = (*((uint16_t *) &USERROW.USERROW26));
    atten_cv_high = (*((uint16_t *) &USERROW.USERROW28));
    atten_rand_high = (*((uint16_t *) &USERROW.USERROW30));
  }
  uint16_t fade_a_div = fade_a_range/17;
  uint16_t fade_b_div = fade_b_range/17;
  uint16_t sample_a_div = sample_a_range/17;
  uint16_t sample_b_div = sample_b_range/17;
  uint16_t pattern_a_div = pattern_a_range/31;
  uint16_t pattern_b_div = pattern_b_range/31;

  volatile struct seqstate save_buffer_data;
  volatile uint8_t * save_buffer = &state;
  const state_size = sizeof(struct seqstate);
  uint8_t save_bytes_left = 0;
  uint8_t advanced = 0;

  eeprom_read_block(&state, 0, state_size);

  uint16_t cv_in_a_norm = 0;
  uint8_t lfo_up = 1;

  // allows mounting the module upside down and handles upside down pots
  uint8_t invert_atten_cv = atten_cv_high < atten_cv_low; 
  uint8_t invert_atten_rand = atten_rand_high < atten_rand_low; 
  uint16_t atten_cv_min = invert_atten_cv ? atten_cv_high : atten_cv_low;
  uint16_t atten_cv_range = invert_atten_cv ? atten_cv_low - atten_cv_high : atten_cv_high - atten_cv_low;
  uint16_t atten_rand_min = invert_atten_rand ? atten_rand_high : atten_rand_low;
  uint16_t atten_rand_range = invert_atten_rand ? atten_rand_low - atten_rand_high : atten_rand_high - atten_rand_low;

  while (1) {
    if (cv_in_a_norm == 0xFFFF) {
      lfo_up = 0;
    } else if (!cv_in_a_norm) {
      lfo_up = 1;
    }
    if (lfo_up) {
      cv_in_a_norm++;
    } else {
      cv_in_a_norm--;
    }
    DAC0.DATA = cv_in_a_norm;
    uint16_t random_a = prng();
    /*printf("hi lol\n");*/
    uint8_t clock_a = (PORTC.IN >> 1) & 1;
    uint8_t clock_b = (PORTF.IN >> 1) & 1;
    uint8_t process_a = 0;
    uint8_t process_b = 0;
    /*clocklow_a = 1; uint8_t clock_a = 1; _delay_ms(100);*/
    /*clocklow_b = 1; uint8_t clock_b = 1; _delay_ms(100);*/
    if (!clock_a) {
        clocklow_a = 1;
    } else if (clocklow_a) {
      process_a = 1;
      clocklow_a = 0;
    }
    if (!clock_b) {
        clocklow_b = 1;
    } else if (clocklow_b) {
      process_b = 1;
      clocklow_b = 0;
    }
    uint8_t reset = (PORTF.IN >> 6) & 1;
    /*if (clock_a | clock_b) {*/
      /*printf("CA:%u CB:%u R:%u PA:%u PB:%u\n", clock_a, clock_b, reset, process_a, process_b);*/
    /*}*/
    if (!(process_a | process_b)) {

      // every time around that we aren't processing, save the next byte of the state 
      // this allows the saving to be interrupted by the sequencer
      if (!(NVMCTRL.STATUS & (NVMCTRL_EEBUSY_bm | NVMCTRL_FBUSY_bm)) & save_bytes_left > 0) {
        uint8_t offset = state_size - save_bytes_left;
        unsigned char this_byte = save_buffer[offset];
        eeprom_update_byte(offset, this_byte);
        save_bytes_left--;
      }
      continue;
    }
    if (reset && process_a) {
        state.index_a = 0;
    }
    if (reset && process_b) {
        state.index_b = 0;
    }
    // inputs
    uint16_t fade_a = adjust(read_adc(AIN_FDA), fade_a_min, fade_a_range);
    uint16_t pattern_a = adjust(read_adc(AIN_PTA), pattern_a_min, pattern_a_range);
    uint16_t sample_a = adjust(read_adc(AIN_SMA), sample_a_min, sample_a_range);
    uint16_t fade_b = adjust(read_adc(AIN_FDB), fade_b_min, fade_b_range);
    uint16_t pattern_b = adjust(read_adc(AIN_PTB), pattern_b_min, pattern_b_range);
    uint16_t sample_b = adjust(read_adc(AIN_SMB), sample_b_min, sample_b_range);
    uint16_t atten_cv = read_adc(AIN_ACV);
    if (invert_atten_cv) {
      atten_cv = 0xFFFF - atten_cv;
    }
    atten_cv = adjust(atten_cv, atten_cv_min, atten_cv_range);
    uint16_t atten_rand = read_adc(AIN_ARD);
    if (invert_atten_rand) {
      atten_rand = 0xFFFF - atten_rand;
    }
    atten_rand = adjust(atten_rand, atten_rand_min, atten_rand_range);
    uint8_t buttons = shift_registers_io(shift_out);
    uint8_t btn_zero_b = buttons & 1;
    uint8_t btn_hold_b = (buttons >> 1) & 1;
    uint8_t btn_sample_b = (buttons >> 2) & 1;
    uint8_t btn_rand_b = (buttons >> 3) & 1;
    uint8_t btn_rand_a = (buttons >> 4) & 1;
    uint8_t btn_sample_a = (buttons >> 5) & 1;
    uint8_t btn_hold_a = (buttons >> 6) & 1;
    uint8_t btn_zero_a = (buttons >> 7) & 1;

    uint8_t seqA_idx = (state.seqA_start + state.index_a) % 16;
    uint8_t seqB_idx = (state.seqB_start + state.index_b) % 16;

    if (seqA_idx | seqB_idx) {
      advanced = 1;
    }

    uint8_t fade_quant_a = fade_a / fade_a_div;
    if (fade_quant_a > 16) {
      fade_quant_a = 16;
    }
    uint8_t fade_quant_b = fade_b / fade_b_div;
    if (fade_quant_b > 16) {
      fade_quant_b = 16;
    }
    uint8_t sample_quant_a = sample_a / sample_a_div;
    if (sample_quant_a > 16) {
      sample_quant_a = 16;
    }
    uint8_t sample_quant_b = sample_b / sample_b_div;
    if (sample_quant_b > 16) {
      sample_quant_b = 16;
    }
    uint8_t step_a = pattern_a / pattern_a_div;
    if (step_a > 30) {
      step_a = 30;
    }
    uint8_t step_b = pattern_b / pattern_b_div;
    if (step_b > 30) {
      step_b = 30;
    }

    /*printf("[A] rand:%u sample:%u hold:%u zero:%u\n", btn_rand_a, btn_sample_a, btn_hold_a, btn_zero_a);*/
    /*printf("[A] fade:%u pattern:%u sample:%u\n", fade_quant_a, step_a, sample_quant_a);*/
    /*printf("[B] rand:%u sample:%u hold:%u zero:%u\n", btn_rand_b, btn_sample_b, btn_hold_b, btn_zero_b);*/
    /*printf("[B] fade:%u pattern:%u sample:%u\n", fade_quant_b, step_b, sample_quant_b);*/
    /*printf("[ATTEN] cv:%u rand:%u\n", atten_cv, atten_rand);*/

    uint16_t unflipped_a = 0;
    uint16_t unflipped_b = 0;
    uint16_t unflipped_sample_a = 0;
    uint16_t unflipped_sample_b = 0;
    for (uint8_t i = 0; i < 16; i++) {
      if (i < fade_quant_a) {
        unflipped_a = (unflipped_a << 1) | 1;
      }
      if (i < fade_quant_b) {
        unflipped_b = (unflipped_b << 1) | 1;
      }
      if (i < sample_quant_a) {
        unflipped_sample_a = (unflipped_sample_a >> 1) | 32768;
      }
      if (i < sample_quant_b) {
        unflipped_sample_b = (unflipped_sample_b >> 1) | 32768;
      }
    }

    // I think I want to do the unnecessary slow steps even when one side isn't clocked
    // to keep latency consistent for easier syncing with external modules
    uint16_t flipped_a = flip(unflipped_a, step_a);
    uint16_t flipped_b = flip(unflipped_b, step_b);
    uint16_t flipped_sample_a = flip(unflipped_sample_a, 30-step_a);
    uint16_t flipped_sample_b = flip(unflipped_sample_b, 30-step_b);

    uint8_t a_lr = (flipped_a >> state.index_a) & 1;
    uint8_t b_lr = (flipped_b >> state.index_b) & 1;
    uint8_t a_sampling = ((flipped_sample_a >> state.index_a) & 1) | btn_sample_a;
    uint8_t b_sampling = ((flipped_sample_b >> state.index_b) & 1) | btn_sample_b;

    /*shift_out = shift_out & 0b01111110;*/

    float rand_pct = ((float)atten_rand) / atten_rand_range;
    float cv_pct = ((float)atten_cv) / atten_rand_range;
    uint16_t random_b = 0;
    if (btn_rand_b) {
      random_b = prng();
    }
    if (btn_zero_a) {
      random_a = random_a / 2;
    }
    if (btn_zero_b) {
      random_b = random_b / 2;
    }
    random_a = random_a * rand_pct;
    random_b = random_b * rand_pct;

    uint16_t out_a;
    uint16_t out_b;

    uint16_t cv_in_a = read_adc(AIN_CVA) * cv_pct;
    uint16_t cv_in_b = read_adc(AIN_CVB) * cv_pct;

    if (process_a) {
      TCD0.CMPASET = 2047-(cv_in_a >> 5);
      shift_out = shift_out & ~LED_A_F & ~LED_A_P & ~LED_A_S & ~GATE_A;
      shift_out = shift_out | (a_lr ? (LED_A_F | GATE_A) : LED_A_P);
      if (a_sampling) {
        shift_out = shift_out | LED_A_S; 
        if (btn_rand_a) {
          for (uint8_t i = 0; i < 8; i++){
            unflipped_sample_a = (unflipped_sample_a >> 1) | 32768;
          }
          unflipped_sample_a = unflipped_sample_a & 0b1011111111101111;
          if ((flip(unflipped_sample_a, 30-step_a) >> state.index_a) & 1) {
            out_a = cv_in_a;
          } else {
            out_a = random_a;
          }
        } else {
            out_a = cv_in_a;
        }
      } else if (btn_rand_a) {
        out_a = random_a;
      } else if (btn_zero_a) {
        out_a = atten_rand;
      } else {
        out_a = a_lr ? state.seqAL[seqA_idx] : state.seqAR[seqA_idx];
      }

      write_dac(0, out_a);

      if (a_lr) {
        state.seqAL[seqA_idx] = out_a;
      } else {
        state.seqAR[seqA_idx] = out_a;
      }

      state.index_a = (state.index_a + 1) % 16;
      if (btn_hold_a) {
        if (state.seqA_start == 0) {
          state.seqA_start = 16;
        } else {
          state.seqA_start--;
        }
      }
    }

    if (process_b) {
      TCD0.CMPBSET = 4095-(cv_in_b >> 5);
      shift_out = shift_out & ~LED_B_F & ~LED_B_P & ~LED_B_S & ~GATE_B;
      shift_out = shift_out | (b_lr ? (LED_B_F | GATE_B) : LED_B_P);
      if (b_sampling) {
        shift_out = shift_out | LED_B_S; 
        if (btn_rand_b) {
          for (uint8_t i = 0; i < 8; i++) {
            unflipped_sample_b = (unflipped_sample_b >> 1) | 32768;
          }
          unflipped_sample_b = unflipped_sample_b & 0b1011111111101111;
          if ((flip(unflipped_sample_b, 30-step_b) >> state.index_b) & 1) {
            out_b = cv_in_b;
          } else {
            out_b = random_b;
          }
        } else {
            out_b = cv_in_b;
        }
      } else if (btn_rand_b) {
        out_b = random_b;
      } else if (btn_zero_b) {
        out_b = atten_rand;
      } else {
        out_b = b_lr ? state.seqBL[seqB_idx] : state.seqBR[seqB_idx];
      }

      write_dac(1, out_b);

      if (b_lr) {
        state.seqBL[seqB_idx] = out_b;
      } else {
        state.seqBR[seqB_idx] = out_b;
      }

      state.index_b = (state.index_b + 1) % 16;
      if (btn_hold_b) {
        if (state.seqB_start == 0) {
          state.seqB_start = 16;
        } else {
          state.seqB_start--;
        }
      }
    }

    pwm_out_sync();
    shift_registers_io(shift_out);

    /*printf("%u %u %u", index, out_a, out_b);*/
    /*printf("[A] %u + %u\n", state.seqA_start, index_a);*/

    // save when either sequence is at zero and something has happened since last save
    if (advanced & ((!seqB_idx) | !(seqA_idx)) & !save_bytes_left) {
      advanced = 0;
      save_bytes_left = state_size;
      save_buffer_data = state;
    }
  }
}
