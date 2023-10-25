#define F_CPU 24000000UL

#include <pattern_table.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>

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
    _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, 0x9);
    PORTA.DIR = 0b01010001; // USART
    PORTC.DIR = 0b00001101;
    PORTD.DIR = 0b00000000;
    PORTF.DIR = 0b00000000;

    USART0.BAUD = 138; // 115200 if F_CPU = 4MHz
    USART0.CTRLA = 0b00000000;
    USART0.CTRLC = 0x03; //default
    PORTMUX.USARTROUTEA = 0b00000001;
    USART0.CTRLB = 0b01000000;

    VREF.ADC0REF = 0x6;
    VREF.DAC0REF = 0x5;

    ADC0.CTRLA = 0b1;

    //todo these are the lowest values where nothing breaks
    ADC0.CTRLB = 0x5;
    ADC0.CTRLD = 0x2;

    DAC0.CTRLA = 0b01000001;

    PORTMUX.SPIROUTEA = 0x3;
    SPI0.CTRLB = 0b00000100;
    /*SPI0.CTRLA = 0b00100001;*/

    write_dac(0b00100000, 0b11); //power up dac A and B
    write_dac(0b00110000, 0b11); //LDAC pin disabled
    write_dac(0b00111000, 1);    //enable internal reference, set gain = 2

    stdout = &mystdout;
    stderr = &mystdout;
}

/*void print_bin(uint16_t u16) {*/
    /*int i = 16;*/
    /*while(i--) {*/
        /*uart_putchar('0' + ((u16 >> i) & 1), NULL);*/
    /*}*/
/*}*/

const float cv_in_scale_factor_a = (4.0/3.978);
const float cv_in_scale_factor_b = (4.0/3.975);
const float scale_factor_a = 1.0;
const float scale_factor_b = 1.0;
/*const float cv_in_scale_factor_a = 1.0;*/
/*const float cv_in_scale_factor_b = 1.0;*/

const float unit_semitone = 65535.0/60.0;

uint16_t quantize_semitone(uint16_t value) {
  float pct = ((float) value) / 65535.0;
  uint16_t index = lround(pct * 60.0);
  return index * unit_semitone;
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
}

#define AIN_CVA 27 
#define AIN_CVB 1
#define AIN_FDA 2
#define AIN_PTA 3
#define AIN_SMA 4
#define AIN_FDB 5
#define AIN_ACV 25
#define AIN_ARD 23 
#define AIN_PTB 22 
#define AIN_SMB 16 

#define GATE_B  0b10000000
#define LED_B_R 0b00100000
#define LED_B_C 0b01000000
#define LED_B_L 0b00010000
#define LED_A_R 0b00000100
#define LED_A_C 0b00001000
#define LED_A_L 0b00000010
#define GATE_A  0b00000001

uint16_t adc_offset;

uint16_t read_adc(uint8_t muxpos) {
    if (muxpos != AIN_CVA & muxpos != AIN_CVB) {
      VREF.ADC0REF = 0x5;
    }
    ADC0.MUXPOS = muxpos;
    ADC0.COMMAND = 1;
    loop_until_bit_is_set(ADC0.INTFLAGS, ADC_RESRDY_bp);
    uint16_t res = ADC0.RES;
    ADC0.COMMAND = 0;
    VREF.ADC0REF = 0x6;
    if (adc_offset > res) {
      return 0;
    } else {
      return res-adc_offset;
    }
}

void write_dac(uint8_t command, uint16_t data) {
  SPI0.CTRLA = 0b00100001;
  uint8_t byteA = (data >> 8) & 0xFF;
  uint8_t byteB = data & 0xFF;
  PORTA.OUTCLR = 0b01000000; //SS_DAC
  SPI0.DATA = command;
  loop_until_bit_is_set(SPI0.INTFLAGS, SPI_IF_bp);
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
uint16_t pattern_a_min;
uint16_t pattern_a_range;
uint16_t fade_b_min;
uint16_t fade_b_range;
uint16_t pattern_b_min;
uint16_t pattern_b_range;
float cv_in_gain_correction_a;
float cv_in_gain_correction_b;

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

//note: if corruption becomes a noticeable issue, find some more storage and swap writes between two locations
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
  if (shift_registers_io(0) == 0b11110000) {
    printf("Calibration mode babey!!!!!\n");
    adc_offset = 0;
    write_dac(0, 5000);
    write_dac(1, 5000);
    _delay_ms(100);
    uint32_t vals = 0;
    volatile uint8_t i = 0;
    for (; i < 100; i++) {
      vals += read_adc(AIN_CVA);
    }
    uint16_t val1a = vals / 100;
    i = 0; vals = 0;
    for (; i < 100; i++) {
      vals += read_adc(AIN_CVB);
    }
    uint16_t val1b = vals / 100;
    write_dac(0, 50000);
    write_dac(1, 50000);
    _delay_ms(100);
    vals = 0; i = 0;
    for (; i < 100; i++) {
      vals += read_adc(AIN_CVA);
    }
    uint16_t val2a = vals/100;
    vals = 0; i = 0;
    for (; i < 100; i++) {
      vals += read_adc(AIN_CVB);
    }
    uint16_t val2b = vals/100;

    printf("%u %u %u %u\n", val1a, val1b, val2a, val2b);

    adc_offset = val1a - 5000;
    cv_in_gain_correction_a = 50000.0/(float)(val2a - adc_offset);
    cv_in_gain_correction_b = 50000.0/(float)(val2b - adc_offset);

    write_dac(0, 25000);
    write_dac(1, 25000);
    _delay_ms(100);

    uint16_t valA = ((float)read_adc(AIN_CVA)) * cv_in_gain_correction_a;
    uint16_t valB = ((float)read_adc(AIN_CVB)) * cv_in_gain_correction_b;

    printf("%u\n%u\n\n", valA, valB);

    _delay_ms(2000);
    while (!shift_registers_io(0xFF)) {
      fade_a_min = read_adc(AIN_FDA);
      pattern_a_min = read_adc(AIN_PTA);
      fade_b_min = read_adc(AIN_FDB);
      pattern_b_min = read_adc(AIN_PTB);
      atten_cv_low = read_adc(AIN_ACV);
      atten_rand_low = read_adc(AIN_ARD);
    }
    printf("Minimums set\n");
    _delay_ms(2000);
    while (!shift_registers_io(0x44)) {
      fade_a_range = read_adc(AIN_FDA)-fade_a_min;
      pattern_a_range = read_adc(AIN_PTA)-pattern_a_min;
      fade_b_range = read_adc(AIN_FDB)-fade_b_min;
      pattern_b_range = read_adc(AIN_PTB)-pattern_b_min;
      atten_cv_high = read_adc(AIN_ACV);
      atten_rand_high = read_adc(AIN_ARD);
    }
    shift_registers_io(0);
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
    *((uint16_t *)(&USERROW.USERROW4)) = adc_offset;
    *((uint16_t *)(&USERROW.USERROW6)) = fade_b_min;
    *((uint16_t *)(&USERROW.USERROW8)) = pattern_b_min;
    /**((uint16_t *)(&USERROW.USERROW10)) = 0;*/
    *((uint16_t *)(&USERROW.USERROW12)) = atten_cv_low;
    *((uint16_t *)(&USERROW.USERROW14)) = atten_rand_low;
    *((uint16_t *)(&USERROW.USERROW16)) = fade_a_range;
    *((uint16_t *)(&USERROW.USERROW18)) = pattern_a_range;
    *((uint16_t *)(&USERROW.USERROW20)) = val2a;
    *((uint16_t *)(&USERROW.USERROW22)) = fade_b_range;
    *((uint16_t *)(&USERROW.USERROW24)) = pattern_b_range;
    *((uint16_t *)(&USERROW.USERROW26)) = val2b;
    *((uint16_t *)(&USERROW.USERROW28)) = atten_cv_high;
    *((uint16_t *)(&USERROW.USERROW30)) = atten_rand_high;
    while (NVMCTRL.STATUS & (NVMCTRL_EEBUSY_bm | NVMCTRL_FBUSY_bm));
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, 0);
    printf("Calibration values saved\n");
  } else {
    fade_a_min = (*((uint16_t *) &USERROW.USERROW0));
    pattern_a_min = (*((uint16_t *) &USERROW.USERROW2));
    adc_offset = (*((uint16_t *) &USERROW.USERROW4));
    fade_b_min = (*((uint16_t *) &USERROW.USERROW6));
    pattern_b_min = (*((uint16_t *) &USERROW.USERROW8));
    /*blah = (*((uint16_t *) &USERROW.USERROW10));*/
    atten_cv_low = (*((uint16_t *) &USERROW.USERROW12));
    atten_rand_low = (*((uint16_t *) &USERROW.USERROW14));
    fade_a_range = (*((uint16_t *) &USERROW.USERROW16));
    pattern_a_range = (*((uint16_t *) &USERROW.USERROW18));
    uint16_t val2a = (*((uint16_t *) &USERROW.USERROW20));
    fade_b_range = (*((uint16_t *) &USERROW.USERROW22));
    pattern_b_range = (*((uint16_t *) &USERROW.USERROW24));
    uint16_t val2b = (*((uint16_t *) &USERROW.USERROW26));
    atten_cv_high = (*((uint16_t *) &USERROW.USERROW28));
    atten_rand_high = (*((uint16_t *) &USERROW.USERROW30));
    cv_in_gain_correction_a = 50000.0/(float)(val2a - adc_offset);
    cv_in_gain_correction_b = 50000.0/(float)(val2b - adc_offset);
  }
  uint16_t fade_a_div = fade_a_range/num_fades;
  uint16_t fade_b_div = fade_b_range/num_fades;
  uint16_t pattern_a_div = pattern_a_range/num_patterns;
  uint16_t pattern_b_div = pattern_b_range/num_patterns;

  volatile struct seqstate save_buffer_data;
  volatile uint8_t * save_buffer = &state;
  const state_size = sizeof(struct seqstate);
  uint8_t save_bytes_left = 0;
  uint8_t advanced = 0;

  eeprom_read_block(&state, 0, state_size);

  uint16_t cv_in_a_norm = 0;
  uint8_t lfo_up = 1;

  // allows mounting the module upside down
  uint8_t invert_atten_cv = atten_cv_high < atten_cv_low; 
  uint8_t invert_atten_rand = atten_rand_high < atten_rand_low; 
  uint16_t atten_cv_min = invert_atten_cv ? atten_cv_high : atten_cv_low;
  uint16_t atten_cv_range = invert_atten_cv ? atten_cv_low - atten_cv_high : atten_cv_high - atten_cv_low;
  uint16_t atten_rand_min = invert_atten_rand ? atten_rand_high : atten_rand_low;
  uint16_t atten_rand_range = invert_atten_rand ? atten_rand_low - atten_rand_high : atten_rand_high - atten_rand_low;

  /*while (1) {*/
    /*printf("hello\n");*/
  /*}*/

  printf("hello\n");
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
    if (!(process_a | process_b)) {

      // every time around that we aren't processing, save the next byte of the state 
      // this allows the saving to be interrupted by the sequencer
      // todo bottleneck?
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
    uint16_t fade_b = adjust(read_adc(AIN_FDB), fade_b_min, fade_b_range);
    uint16_t pattern_b = adjust(read_adc(AIN_PTB), pattern_b_min, pattern_b_range);
    uint16_t sample_a = read_adc(AIN_SMA);
    uint16_t sample_b = read_adc(AIN_SMB);
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

    uint8_t btn_rand_b = buttons & 1;
    uint8_t btn_zero_b = (buttons >> 1) & 1;
    uint8_t btn_sample_b = (buttons >> 2) & 1;
    uint8_t btn_sample_internal_b = (buttons >> 3) & 1;
    uint8_t btn_sample_internal_a = (buttons >> 4) & 1;
    uint8_t btn_sample_a = (buttons >> 5) & 1;
    uint8_t btn_zero_a = (buttons >> 6) & 1;
    uint8_t btn_rand_a = (buttons >> 7) & 1;

    uint8_t sampling_a = btn_sample_a | btn_sample_internal_a | (sample_a > 4000); //TODO noise issues
    uint8_t sampling_b = btn_sample_b | btn_sample_internal_b | (sample_b > 4000);
    uint8_t sample_ext_a = sample_a > 7500; // 0 = internal, 1 = external
    uint8_t sample_ext_b = sample_b > 7500; // 0 = internal, 1 = external
    if (btn_sample_a) {
      sample_ext_a = 1;
    } else if (btn_sample_internal_a) {
      sample_ext_a = 0;
    }
    if (btn_sample_b) {
      sample_ext_b = 1;
    } else if (btn_sample_internal_b) {
      sample_ext_b = 0;
    }

    uint8_t seqA_idx = (state.seqA_start + state.index_a) % 16;
    uint8_t seqB_idx = (state.seqB_start + state.index_b) % 16;

    if (seqA_idx | seqB_idx) {
      advanced = 1;
    }

    uint8_t fade_quant_a = fade_a / fade_a_div;
    if (fade_quant_a > num_patterns-1) {
      fade_quant_a = 16;
    }
    uint8_t fade_quant_b = fade_b / fade_b_div;
    if (fade_quant_b > num_patterns-1) {
      fade_quant_b = 16;
    }
    uint8_t step_a = pattern_a / pattern_a_div;
    if (step_a > num_patterns-1) {
      step_a = num_patterns-1;
    }
    uint8_t step_b = pattern_b / pattern_b_div;
    if (step_b > num_patterns-1) {
      step_b = num_patterns-1;
    }

    /*printf("[A] rand:%u sample:%u sample_internal:%u zero:%u\n", btn_rand_a, btn_sample_a, btn_sample_internal_a, btn_zero_a);*/
    /*printf("[A] fade:%u pattern:%u sample:%u ext=%u\n", fade_quant_a, step_a, sample_a, sample_ext_a);*/
    /*printf("[B] rand:%u sample:%u sample_internal:%u zero:%u\n", btn_rand_b, btn_sample_b, btn_sample_internal_b, btn_zero_b);*/
    /*printf("[B] fade:%u pattern:%u sample:%u ext=%u\n", fade_quant_b, step_b, sample_b, sample_ext_b);*/
    /*printf("[ATTEN] cv:%u rand:%u\n", atten_cv, atten_rand);*/

    uint16_t flipped_a = pattern_table[step_a][fade_quant_a];
    uint16_t flipped_b = pattern_table[step_b][fade_quant_b];

    uint8_t a_lr = (flipped_a >> (15-state.index_a)) & 1;
    uint8_t b_lr = (flipped_b >> (15-state.index_b)) & 1;

    float rand_pct = fmin(1, ((float)atten_rand) / atten_rand_range);
    float cv_pct = fmin(1, ((float)atten_cv) / atten_rand_range);

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

    // do this as late as possible so we don't catch slew
    uint16_t cv_in_a = ((float)read_adc(AIN_CVA))*cv_in_gain_correction_a;
    uint16_t cv_in_b = ((float)read_adc(AIN_CVB))*cv_in_gain_correction_b;
    cv_in_a *= cv_pct;
    cv_in_b *= cv_pct;

    if (process_a) {
      shift_out = shift_out & ~LED_A_L & ~LED_A_R & ~LED_A_C & ~GATE_A;
      shift_out = shift_out | (a_lr ? (LED_A_R | GATE_A) : LED_A_L);
      if (sampling_a) {
        shift_out = shift_out | LED_A_C; 
      }
      if (sampling_a && sample_ext_a) {
        out_a = quantize_semitone(cv_in_a);
      } else if (btn_rand_a) {
        out_a = quantize_semitone(random_a);
      } else if (btn_zero_a) {
        out_a = quantize_semitone(atten_rand);
      } else {
        out_a = a_lr ? state.seqAR[seqA_idx] : state.seqAL[seqA_idx];
      }
    }

    if (process_b) {
      shift_out = shift_out & ~LED_B_L & ~LED_B_R & ~LED_B_C & ~GATE_B;
      shift_out = shift_out | (b_lr ? (LED_B_R | GATE_B) : LED_B_L);
      if (sampling_b) {
        shift_out = shift_out | LED_B_C; 
      }
      if (sampling_b && sample_ext_b) {
          out_b = quantize_semitone(cv_in_b);
      } else if (btn_rand_b) {
        out_b = quantize_semitone(random_b);
      } else if (btn_zero_b) {
        out_b = quantize_semitone(atten_rand);
      } else {
        out_b = b_lr ? state.seqBR[seqB_idx] : state.seqBL[seqB_idx];
      }
    }

    if (process_a && sampling_a && !(sample_ext_a)) {
      // sampling each other, swap
      if (process_b && sampling_b && !(sample_ext_b)) {
        uint16_t temp = out_a;
        out_a = quantize_semitone(out_b * cv_pct);
        out_b = quantize_semitone(temp * cv_pct);
      } else {
        out_a = quantize_semitone(out_b * cv_pct);
      }
    } else if (process_b && sampling_b && !(sample_ext_b)) {
      out_b = quantize_semitone(out_a * cv_pct);
    }

    if (process_a) {
      write_dac(0, out_a * scale_factor_a);

      if (a_lr) {
        state.seqAR[seqA_idx] = out_a;
      } else {
        state.seqAL[seqA_idx] = out_a;
      }

      state.index_a = (state.index_a + 1) % 16;
    }

    if (process_b) {
      write_dac(1, out_b * scale_factor_b);

      if (b_lr) {
        state.seqBR[seqB_idx] = out_b;
      } else {
        state.seqBL[seqB_idx] = out_b;
      }

      state.index_b = (state.index_b + 1) % 16;
    }

    shift_registers_io(shift_out);

    /*printf("%u %u\n", out_a, out_b);*/
    /*printf("[A] %u + %u\n", state.seqA_start, index_a);*/

    // save when either sequence is at zero and something has happened since last save
    if (advanced & ((!seqB_idx) | !(seqA_idx)) & !save_bytes_left) {
      advanced = 0;
      save_bytes_left = state_size;
      save_buffer_data = state;
    }
  }
}
