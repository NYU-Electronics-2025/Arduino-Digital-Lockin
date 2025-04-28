#include "pico/multicore.h"
#include "tusb.h"
#include "hardware/adc.h"
#include <elapsedMillis.h>

#define NUM_CMD_DATA 4
#define CHAR_BUF_SIZE 128
#define VERSION 1


typedef struct {
  char cmd;
  float data[NUM_CMD_DATA];
} CommandT;


//pins
const int adc_in_pin = 27;
const int adc_ref_pin = 26;
const int output_pin = 0;
const int test_pin_1 = 1;
const int test_pin_2 = 2;


const int adc_ratio = 48;  //500 kHz adc -> 10.4 kHz led 
const int sin_table_bits = 16;  
const int sintable[adc_ratio] = { 0, 8554, 16962, 25079, 32767, 39895, 46340, 51992, 56755, 60546, 63302, 64974, 65535, 64974, 63302, 60546, 56755, 51992, 46340, 39895, 32768, 25079, 16962, 8554, 0, -8554, -16962, -25079, -32767, -39895, -46340, -51992, -56755, -60546, -63302, -64974, -65535, -64974, -63302, -60546, -56755, -51992, -46340, -39895, -32768, -25079, -16962, -8554 };
const int costable[adc_ratio] = { 65535, 64974, 63302, 60546, 56755, 51992, 46340, 39895, 32768, 25079, 16962, 8554, 0, -8554, -16962, -25079, -32767, -39895, -46340, -51992, -56755, -60546, -63302, -64974, -65535, -64974, -63302, -60546, -56755, -51992, -46340, -39895, -32768, -25079, -16962, -8554, 0, 8554, 16962, 25079, 32767, 39895, 46340, 51992, 56755, 60546, 63302, 64974 };
//yes, cos is just sin shifted over, but less math to just access it directly

/*
const float sintable_float[adc_ratio] = {0.00000000,0.13052619,0.25881905,0.38268343,0.50000000,0.60876143,0.70710678,0.79335334,0.86602540,0.92387953,0.96592583,0.99144486,1.00000000,0.99144486,0.96592583,0.92387953,0.86602540,0.79335334,0.70710678,0.60876143,0.50000000,0.38268343,0.25881905,0.13052619,0.00000000,-0.13052619,-0.25881905,-0.38268343,-0.50000000,-0.60876143,-0.70710678,-0.79335334,-0.86602540,-0.92387953,-0.96592583,-0.99144486,-1.00000000,-0.99144486,-0.96592583,-0.92387953,-0.86602540,-0.79335334,-0.70710678,-0.60876143,-0.50000000,-0.38268343,-0.25881905,-0.13052619};
const float costable_float[adc_ratio] = {1.00000000,0.99144486,0.96592583,0.92387953,0.86602540,0.79335334,0.70710678,0.60876143,0.50000000,0.38268343,0.25881905,0.13052619,0.00000000,-0.13052619,-0.25881905,-0.38268343,-0.50000000,-0.60876143,-0.70710678,-0.79335334,-0.86602540,-0.92387953,-0.96592583,-0.99144486,-1.00000000,-0.99144486,-0.96592583,-0.92387953,-0.86602540,-0.79335334,-0.70710678,-0.60876143,-0.50000000,-0.38268343,-0.25881905,-0.13052619,-0.00000000,0.13052619,0.25881905,0.38268343,0.50000000,0.60876143,0.70710678,0.79335334,0.86602540,0.92387953,0.96592583,0.99144486};
*/

uint16_t refval;

double filt_alpha = 0.01;

double second_filt_alpha = 0.01; 

double adc_accum_mult = filt_alpha / (adc_ratio * (1<<sin_table_bits));  //output comes out 0-2048

//globals
double cos_filt;
double sin_filt;

double cf;
double sf;

int16_t vmax;
int16_t vmin;

uint8_t adc_fifo_max_level = 0;

int32_t adc_loop_time = 0;
int32_t adc_loop_time_with_counter_rollover = 0;

float led_mult = 10;

int adc_clock_divisor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(output_pin, OUTPUT);
  pinMode(test_pin_1, OUTPUT);
  pinMode(test_pin_2, OUTPUT);
  analogWriteFreq(10000);
  analogWriteResolution(8);
  Serial.println("ready");
}


int report_interval = 0;
elapsedMillis time_since_report;
elapsedMicros emfilt = 0;

int diagnostic_interval = 0;
elapsedMillis time_since_diagnostic;

void loop() {
  if (emfilt > 1000) {
    emfilt -= 1000;
    cf = cf*(1-second_filt_alpha) + second_filt_alpha*cos_filt;
    sf = sf*(1-second_filt_alpha) + second_filt_alpha*sin_filt;
    analogWrite(LED_BUILTIN, sqrt(cf*cf + sf*sf)*led_mult);
  }
  pollSerial();
}

void sendReport(){
    Serial.print("mag = ");
    double mag = sqrt(cf*cf + sf*sf);
    Serial.print(mag, 4);
    Serial.print(", phase = ");
    Serial.print(atan2(sf, cf)*57.296);
    Serial.print(", cos = ");
    Serial.print(cf, 4);
    Serial.print(", sin = ");
    Serial.println(sf, 4);
  }

void sendDiagnosticReport() {
    Serial.print("ref = ");
    Serial.println(refval);
    Serial.print("vmax = ");
    Serial.println(vmax);
    Serial.print("vmin = ");
    Serial.println(vmin);
    Serial.print("adc clock divisor = ");
    Serial.println(adc_clock_divisor);
    Serial.print("max adc fifo level since last report = ");
    Serial.println(adc_fifo_max_level);
    adc_fifo_max_level = 0; 
    Serial.print("adc loop time (with rollover) in us: ");
    Serial.print(adc_loop_time);
    Serial.print(" (");
    Serial.print(adc_loop_time_with_counter_rollover);
    Serial.println(")");
} 

void setup1() {
  setupADC();
  delay(1000);
  adc_fifo_drain();
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

  adc_gpio_init(adc_in_pin);
  adc_fifo_setup(true, false, 8, false, false);  //do not set up dma
  setADCClockDiv(239); //base ADC frequency of 200 kHz
}

void setADCClockDiv(int div) {
  adc_clock_divisor = div;
  adc_run(false);
  adc_set_clkdiv(div);
  adc_run(true);

}

elapsedMicros emu = 0;

void adcLoop(void) {
  static int ctr = 0;
  static int64_t cos_accumulator = 0;
  static int64_t sin_accumulator = 0;
  static bool tp1 = false;
  static bool tp2 = false;
  static int16_t vmax_temp;
  static int16_t vmin_temp;
  static int64_t ca;
  static int64_t sa;
  static uint8_t math_ctr = 0;

  if (adc_fifo_is_empty()) {
    //spread math out over multiple steps
    // cos_filt = (1 - filt_alpha) * cos_filt + adc_accum_mult * cos_accumulator;
    // sin_filt = (1 - filt_alpha) * sin_filt + adc_accum_mult * sin_accumulator;
    switch (math_ctr) {
      case 0:
        cos_filt = (1 - filt_alpha) * cos_filt;
        math_ctr++;
        return;
        break;
      case 1:
        cos_filt += adc_accum_mult * ca;
        math_ctr++;
        return;
        break;
      case 2:
        sin_filt = (1 - filt_alpha) * sin_filt;
        math_ctr++;
        return;
        break;
      case 3:
        sin_filt += adc_accum_mult * sa;
        math_ctr++;
        return;
        break;
      default:
      return;
      break;
    }
    return;
  }
  uint8_t fifo_level = adc_fifo_get_level();
  adc_fifo_max_level = fifo_level > adc_fifo_max_level ? fifo_level : adc_fifo_max_level;
  tp2 = !tp2;
  digitalWrite(test_pin_2, tp2);

  int16_t val = ((int16_t) (adc_fifo_get() & 0xFFF)) - ((int16_t) refval);
  vmax_temp = vmax_temp > val ? vmax_temp : val;
  vmin_temp = vmin_temp < val ? vmin_temp : val;
  cos_accumulator += val * costable[ctr];
  sin_accumulator += val*sintable[ctr];
  adc_loop_time = emu;
  if (++ctr >= adc_ratio) {
    ctr = 0;
    ca = cos_accumulator;
    sa = sin_accumulator;
    cos_accumulator = 0;
    sin_accumulator = 0;
    vmax = vmax_temp;
    vmin = vmin_temp;
    vmax_temp = -4096;
    vmin_temp = 4096;
    adc_loop_time_with_counter_rollover = emu;
    math_ctr = 0;
  }
  digitalWrite(output_pin, ctr < adc_ratio >> 1);
  emu = 0;
}


int readLineSerial(char buff[], int buffersize, unsigned int timeout) {
  int i = 0;
  if (!Serial || !tud_cdc_connected()) {
    return 0;
  }
  if (!Serial.available()) {
    return 0;
  }
  elapsedMicros t0;
  while (i < buffersize - 1 && (Serial.available() || t0 < timeout)) {
    if (!Serial.available()) {
      continue;
    }
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (i > 0) { //discard newline characters at beginning of string (in case of \n\r)
        buff[i + 1] = '\0';
        return i; //positive return value
      }
    } else {
      buff[i++] = c;
    }
  }
  return -i; //0 if nothing read, negative value if read was incomplete
}

void pollSerial() {
  if (!Serial) {
    return;
  }
  processSerialLine();
   if (report_interval > 0 && time_since_report > report_interval){
    time_since_report -= report_interval;
    sendReport();
  }
  if (diagnostic_interval > 0 && time_since_diagnostic > diagnostic_interval){
    time_since_diagnostic -= diagnostic_interval;
    sendDiagnosticReport();
  }
}

void processSerialLine() {
  char buff[CHAR_BUF_SIZE];
  int rv;
  rv = readLineSerial(buff, CHAR_BUF_SIZE, 500);  //changed from 500 to 500k for debugging
  // if (rv < 0) {
  //   sendMessage("line reading failed", 1);
  //   return;
  // }
  if (rv == 0) {
    return;
  }
  //  restarted = false; //received a command
  int wsoff = 0;
  for (wsoff = 0; isspace(buff[wsoff]) && wsoff < rv; ++wsoff)
    ;  //get rid of leading whitespace
  CommandT c;
  c.cmd = 'X';
  c.data[0] = c.data[1] = c.data[2] = c.data[3] = 0;
  sscanf(buff + wsoff, "%c %f %f %f %f", &c.cmd, c.data, c.data + 1, c.data + 2, c.data + 3);  //change if num_data_bytes changes
  parseCommand(c);
}

void parseCommand(CommandT c) {
  digitalWrite(LED_BUILTIN, LOW);
  switch (toupper(c.cmd)) {
    case 'F':  //set filter alpha
      filt_alpha = c.data[0];
      adc_accum_mult = filt_alpha / (adc_ratio * (1<<sin_table_bits));
      if (c.data[1] > 0) {
        second_filt_alpha = c.data[1];
      } else {
        second_filt_alpha = filt_alpha;
      }
    return;
    case 'D':  //set adc divisor
      setADCClockDiv(c.data[0]); 
      return;
    case 'G':  //send Diagnostic Report
      sendDiagnosticReport();
      diagnostic_interval = c.data[0];
      time_since_diagnostic = 0;
      return;
    case 'R':  //get report or report every xx seconds
      sendReport();
      report_interval = c.data[0];
      time_since_report = 0;
      return;
    case 'L': //set multiplier for built in led brightness from mag
      led_mult = c.data[0];
      return;
    default:
      digitalWrite(LED_BUILTIN, HIGH);
  }
}
