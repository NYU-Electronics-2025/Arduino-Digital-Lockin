#include "pico/multicore.h"
#include "tusb.h"
#include "hardware/adc.h"
#include <elapsedMillis.h>


const int adc_ratio = 48;  //500 kHz adc -> 10.4 kHz led 
const int sin_table_bits = 16;  

const int sintable[adc_ratio] = { 0, 8554, 16962, 25079, 32767, 39895, 46340, 51992, 56755, 60546, 63302, 64974, 65535, 64974, 63302, 60546, 56755, 51992, 46340, 39895, 32768, 25079, 16962, 8554, 0, -8554, -16962, -25079, -32767, -39895, -46340, -51992, -56755, -60546, -63302, -64974, -65535, -64974, -63302, -60546, -56755, -51992, -46340, -39895, -32768, -25079, -16962, -8554 };
const int costable[adc_ratio] = { 65535, 64974, 63302, 60546, 56755, 51992, 46340, 39895, 32768, 25079, 16962, 8554, 0, -8554, -16962, -25079, -32767, -39895, -46340, -51992, -56755, -60546, -63302, -64974, -65535, -64974, -63302, -60546, -56755, -51992, -46340, -39895, -32768, -25079, -16962, -8554, 0, 8554, 16962, 25079, 32767, 39895, 46340, 51992, 56755, 60546, 63302, 64974 };
//yes, cos is just sin shifted over, but less math to just access it directly

const float sintable_float[adc_ratio] = {0.00000000,0.13052619,0.25881905,0.38268343,0.50000000,0.60876143,0.70710678,0.79335334,0.86602540,0.92387953,0.96592583,0.99144486,1.00000000,0.99144486,0.96592583,0.92387953,0.86602540,0.79335334,0.70710678,0.60876143,0.50000000,0.38268343,0.25881905,0.13052619,0.00000000,-0.13052619,-0.25881905,-0.38268343,-0.50000000,-0.60876143,-0.70710678,-0.79335334,-0.86602540,-0.92387953,-0.96592583,-0.99144486,-1.00000000,-0.99144486,-0.96592583,-0.92387953,-0.86602540,-0.79335334,-0.70710678,-0.60876143,-0.50000000,-0.38268343,-0.25881905,-0.13052619};
const float costable_float[adc_ratio] = {1.00000000,0.99144486,0.96592583,0.92387953,0.86602540,0.79335334,0.70710678,0.60876143,0.50000000,0.38268343,0.25881905,0.13052619,0.00000000,-0.13052619,-0.25881905,-0.38268343,-0.50000000,-0.60876143,-0.70710678,-0.79335334,-0.86602540,-0.92387953,-0.96592583,-0.99144486,-1.00000000,-0.99144486,-0.96592583,-0.92387953,-0.86602540,-0.79335334,-0.70710678,-0.60876143,-0.50000000,-0.38268343,-0.25881905,-0.13052619,-0.00000000,0.13052619,0.25881905,0.38268343,0.50000000,0.60876143,0.70710678,0.79335334,0.86602540,0.92387953,0.96592583,0.99144486};

uint16_t refval;

//pins
const int adc_in_pin = 27;
const int adc_ref_pin = 26;
const int output_pin = 0;
const int test_pin_1 = 1;
const int test_pin_2 = 2;

double filt_alpha = 0.01;

double second_filt_alpha = 0.01; 

double adc_accum_mult = filt_alpha / (adc_ratio * 65535);  //output comes out 0-2048

//globals
double cos_filt;
double sin_filt;

double cf;
double sf;

int16_t vmax;
int16_t vmin;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(output_pin, OUTPUT);
  pinMode(test_pin_1, OUTPUT);
  pinMode(test_pin_2, OUTPUT);
  analogWriteFreq(10000);
  analogWriteResolution(sin_table_bits);
  Serial.println("ready");
}
elapsedMillis em = 0;
elapsedMicros emfilt = 0;
void loop() {
  if (emfilt > 1000) {
    emfilt = 0;
    cf = cf*(1-second_filt_alpha) + second_filt_alpha*cos_filt;
    sf = sf*(1-second_filt_alpha) + second_filt_alpha*sin_filt;
  }
  if (em > 200) {
    Serial.print("ref = ");
    Serial.println(refval);
    Serial.print("vmax = ");
    Serial.println(vmax);
    Serial.print("vmin = ");
    Serial.println(vmin);
    Serial.print("mag = ");
    double mag = sqrt(cf*cf + sf*sf);
    analogWrite(LED_BUILTIN, mag*(1<<13));
    Serial.print(mag);
    Serial.print(", phase = ");
    Serial.println(atan2(sf, cf)*57.296);
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
  uint32_t refaccum;
  adc_init();
  analogReadResolution(12);
  const uint8_t rbits = 16;
  for (int j = 0; j < 1 << rbits; ++j) {
    refaccum += analogRead(adc_ref_pin);
  }
  refval = (uint16_t) (refaccum >> rbits);
  //refval =  2048;// analogRead(adc_ref_pin);

  adc_set_clkdiv(200);
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
  static int16_t vmax_temp;
  static int16_t vmin_temp;

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
    
    int16_t val = ((int16_t) (adc_fifo_get() & 0xFFF)) - ((int16_t) refval);
    vmax_temp = vmax_temp > val ? vmax_temp : val;
    vmin_temp = vmin_temp < val ? vmin_temp : val;
 //   cos_accumulator += val * costable_float[ctr];
 //   sin_accumulator += val  * sintable_float[ctr];
    //cos_accumulator += val * (ctr < 13 || ctr > 36 ? 1 : -1);
    //sin_accumulator += val * (ctr < 24 ? 1 : -1);
    cos_accumulator += val * costable[ctr];
    sin_accumulator += val*sintable[ctr];
    if (++ctr >= adc_ratio) {
      ctr = 0;
      cos_filt = (1 - filt_alpha) * cos_filt + adc_accum_mult * cos_accumulator;
      sin_filt = (1 - filt_alpha) * sin_filt + adc_accum_mult * sin_accumulator;
      cos_accumulator = 0;
      sin_accumulator = 0;
      vmax = vmax_temp;
      vmin = vmin_temp;
      vmax_temp = -4096;
      vmin_temp = 4096;
    }
    digitalWrite(output_pin, ctr < adc_ratio >> 1);
  }

  //analogWrite(output_pin,  costable[ctr]/2);
}