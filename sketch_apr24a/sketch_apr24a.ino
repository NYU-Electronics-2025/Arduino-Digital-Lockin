#include "pico/multicore.h"
#include "tusb.h"
#include "hardware/adc.h"
#include <elapsedMillis.h>


const int adc_ratio = 48;  //500 kHz adc -> 10.4 kHz led with 6 bits
const int analog_read_bits = 12;
const int sin_table_bits = 16;  //total bits of sum accumulator = 12 + 16 + log2(48) <= 34

const int sintable[adc_ratio] = { 0, 8554, 16962, 25079, 32767, 39895, 46340, 51992, 56755, 60546, 63302, 64974, 65535, 64974, 63302, 60546, 56755, 51992, 46340, 39895, 32768, 25079, 16962, 8554, 0, -8554, -16962, -25079, -32767, -39895, -46340, -51992, -56755, -60546, -63302, -64974, -65535, -64974, -63302, -60546, -56755, -51992, -46340, -39895, -32768, -25079, -16962, -8554 };
const int costable[adc_ratio] = { 65535, 64974, 63302, 60546, 56755, 51992, 46340, 39895, 32768, 25079, 16962, 8554, 0, -8554, -16962, -25079, -32767, -39895, -46340, -51992, -56755, -60546, -63302, -64974, -65535, -64974, -63302, -60546, -56755, -51992, -46340, -39895, -32768, -25079, -16962, -8554, 0, 8554, 16962, 25079, 32767, 39895, 46340, 51992, 56755, 60546, 63302, 64974 };
//yes, cos is just sin shifted over, but less math to just access it directly

//pins
const int adc_in_pin = 26;
const int output_pin = 0;
const int test_pin_1 = 1;
const int test_pin_2 = 2;

float filt_alpha = 0.01;

float adc_accum_mult = filt_alpha / (adc_ratio << sin_table_bits);  //output comes out scaled same as adc input

//globals
float cos_filt;
float sin_filt;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(output_pin, OUTPUT);
  pinMode(test_pin_1, OUTPUT);
  pinMode(test_pin_2, OUTPUT);
  analogWriteFreq(1000000);
  analogWriteResolution(sin_table_bits);
}
elapsedMillis em = 0;
void loop() {
  analogWrite(LED_BUILTIN, cos_filt);
  if (em > 1000) {
    Serial.print("cos filtered = ");
    Serial.println(cos_filt);
    Serial.print("sin filtered = ");
    Serial.println(sin_filt);
    em = 0;
  }
}

void setup1() {
  setupADC();
  delay(1000);
}

void loop1() {
  adcLoop();
}

void setupADC(void) {
  adc_init();
  adc_gpio_init(adc_in_pin);
  adc_fifo_setup(true, false, 8, false, false);  //do not set up dma
  adc_run(true);
}

elapsedMillis em1 = 0;
void adcLoop(void) {
  static int ctr = 0;
  static int64_t cos_accumulator = 0;
  static int64_t sin_accumulator = 0;
  static bool tp1 = false;
  static bool tp2 = false;

  // if (em1 > 1000) {
  //   Serial.print("adc counter = ");
  //   Serial.println(ctr);
  //   em1 = 0;
  // }

  tp1 = !tp1;
  digitalWrite(test_pin_1, tp1);

  if (adc_fifo_is_empty()) {
    return;
  }
  tp2 = !tp2;
  digitalWrite(test_pin_2, tp2);

  while(!(adc_fifo_is_empty())) {
    uint16_t val = adc_fifo_get() & 0xFFF;
    cos_accumulator += ((int32_t) val) * costable[ctr];
    sin_accumulator += ((int32_t) val)  * sintable[ctr];
    if (++ctr >= adc_ratio) {
      ctr = 0;
      cos_filt = (1 - filt_alpha) * cos_filt + adc_accum_mult * cos_accumulator;
      sin_filt = (1 - filt_alpha) * sin_filt + adc_accum_mult * sin_accumulator;
      cos_accumulator = 0;
      sin_accumulator = 0;
    }
  }

  digitalWrite(output_pin, ctr < adc_ratio >> 1);
  //analogWrite(output_pin,  costable[ctr]/2);
}