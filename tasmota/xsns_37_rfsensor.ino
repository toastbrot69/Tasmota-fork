/*
  xsns_37_rfsensor.ino - RF sensor receiver for Tasmota

  Copyright (C) 2020  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  Interrupt handler, pulse-ringbuffer/ook and below decoders added 2020 by
  Thorsten Pohlmann, tasm.tp<at>pohlmaenner.com

  OOK WT450H
  WT450H (temp, hum, batt)
  OOK LaCrosse TX3
  LaCrosse TX3 (temp, hum)
  OOK infactory
  Infactory (Pearl) (temp, hum, batt)
*/

#ifdef USE_RF_SENSOR
/*********************************************************************************************\
 * RF receiver based on work by Paul Tonkes (www.nodo-domotica.nl)
 *
 * Supported 434MHz receiver is Aurel RX-4M50RR30SF
 * Supported 868MHz receiver is Aurel RX-AM8SF
 *
 * Connect one of above receivers with a 330 Ohm resistor to any GPIO
 *
 * USE_THEO_V2    Add support for 434MHz Theo V2 sensors as documented on https://sidweb.nl
 * USE_ALECTO_V2  Add support for 868MHz Alecto V2 sensors like ACH2010, WS3000 and DKW2012 weather stations
\*********************************************************************************************/

/*
  Interrupt handler, pulse-ringbuffer, decoders and mqtt-push added 2020 by
  Thorsten Pohlmann, tasm.tp<at>pohlmaenner.com

  WT450H (temp, hum, batt)
  LaCrosse TX3 (temp, hum)
  Infactory (Pearl) (temp, hum, batt)

  Required hardware: slightly modified Sonoff-rf-bridge or any 433/868MHz receiver with OOK/carrier pin
  Tested with:
    -- A Sonoff-RF-Bridge (433MHz) can be used with a minor hardware-patch, no Portisch-Firmware needed.
       Also the "normal" or Portisch features for decoding/sending are unharmed. 
      - Wire a 160...680 Ohm resistor from pin 10 of the EFM8BB1 to any free gpio of the ESP. 
        This might be gpio 4/5 if the USB lines have been cut (R2), or one of the free gpio 12/14 
        pads on the bottom-side. Iam using gpio 4.
        See https://github.com/xoseperez/espurna/wiki/Hardware-Itead-Sonoff-RF-Bridge---Direct-Hack
    -- A CC1101 board, this would require an extra library (lSatan)   

  Commands: sensor37 debug 0..3 0: no debug output
                                1: pulse-duration output for protocol reverse-engineering
                                2: output from the various OOK decoders
                                3: 1+2   
  
  Howto add a new decoder:
  =========================
  
  1.: implement: int your_protocol_decoder(bool initial_ook, uint16_t* pulses, int len) 
                                                initial_ook: pin state during 1st pulse
                                                return: != 0 if pulses contained valid data
  
             - you may reuse one of the OOK decoders (time-pulses -> 1/0)
             int decodeOOKxxx(int min_bits, uint16_t* pulses, int len, ...)
                              min_bits: minimum number of valid bits in a row needed for protocol parser
                              return: offset to pulses where parsing ended
 
             - stores bits as 0/1 in ook_data.data, number of bit: ook_data.num_data
             - returns at 1st invalid pulse (if enough bits in ook_data)
             - may then be called again by decoder with proper offset
  
             use jsonStart/Temp/.../jsonEnd + mqttSend() to send data
                            
  
  2.: Add your_protocol_decoder() to checkPulses()
  
  
*/


#define XSNS_37                   37

//#define USE_CC1101
#ifdef USE_CC1101
/*
  To use a CC1101 board in ELECHOUSE/lSatan OOK-mode connect it like 
  SCK_PIN   = D1 Mini: D5 (GPIO 14)
  MISO_PIN  = D1 Mini: D6 (GPIO 12)
  MOSI_PIN  = D1 Mini: D7 (GPIO 13)
  SS_PIN    = D1 Mini: D8 (GPIO 15)
  GDO0      = pin template config "OOK RX"
*/

  #include "../lib/SmartRC-CC1101-Driver-Lib/ELECHOUSE_CC1101_SRC_DRV.h"
  static float cc1101_freq = 433.920;

void initCC1101(void)
{
  // check if the fixed CC1101 pins are being used by other sensors
  if(GetPin(10) + GetPin(11) + GetPin(12) + GetPin(13) == GPIO_NONE)
  {
    ELECHOUSE_cc1101.Init(); // Initialize the cc1101. 

    ELECHOUSE_cc1101.SetRx(cc1101_freq); //Sets receive on and changes the frequency.
    snprintf_P(log_data, sizeof(log_data), "CC1101 freq: %i.%03i", (int)cc1101_freq, (int)(cc1101_freq*100)%100);
  }
  else
  {
    snprintf_P(log_data, sizeof(log_data), "CC1101 default pins occupied");
  }
  AddLog(LOG_LEVEL_INFO);

}

#endif


//#define USE_THEO_V2                      // Add support for 434MHz Theo V2 sensors as documented on https://sidweb.nl
//#define USE_ALECTO_V2                    // Add support for 868MHz Alecto V2 sensors like ACH2010, WS3000 and DKW2012

// definitions
#define MIN_OOKLEN       400
#define MAX_OOKLEN       8000
#define MIN_VALID_PULSES 20

#define MAX_PULSES_NUM   256

typedef struct pulses_t
{
  int       num_pulses;
  uint8_t   initial_ook;
  uint16_t  pulse[MAX_PULSES_NUM];
}pulses_t;

// buffer for pulse measurement
#define SIZE_PULSES_ARR   8

static pulses_t* pulses_arr;

// ring buffer pulse_arr
static int          cur_pulses_rd = 0;
static volatile int cur_pulses_wr = 0;

static uint8_t raw433_pin = 0; 
static uint8_t pin_interrupt = 0;

typedef enum
{
  deb_pulses = 1<<0,
  deb_ook    = 1<<1,
  deb_ookval = 1<<2,
  deb_cache  = 1<<3,
}debug_e;

static unsigned int debug_val = 0; //deb_ook | deb_pulses;

static void ICACHE_RAM_ATTR interruptHandler();

static void RfPulseInit(void)
{
  raw433_pin = Pin(GPIO_RF_SENSOR);
  pinMode(raw433_pin, INPUT);
  
  uint8_t interrupt = digitalPinToInterrupt(raw433_pin);
  if (interrupt == 0) 
  {
    snprintf_P(log_data, sizeof(log_data), "RfPulse init (pin: %i): no interrupt", raw433_pin);
    AddLog(LOG_LEVEL_ERROR);

    return;
  }

  pulses_arr = (pulses_t*)malloc(sizeof(pulses_t) * SIZE_PULSES_ARR);
  if (interrupt == 0) 
  {
    snprintf_P(log_data, sizeof(log_data), "RfPulse init (pin: %i): no memory (%i bytes)", raw433_pin, sizeof(pulses_t) * SIZE_PULSES_ARR);
    AddLog(LOG_LEVEL_ERROR);

    return;
  }
  memset(pulses_arr, 0, sizeof(pulses_t) * SIZE_PULSES_ARR);
  
#ifdef USE_CC1101
  initCC1101();
#endif

  pin_interrupt = interrupt;

  attachInterrupt(interrupt, interruptHandler, CHANGE);
}

// interrupt context: 
// - measure time since last pin-change
// - add time to pulse_arr[x]
// - mark pulse_arr[x] as good if >= MIN_VALID_PULSES
static void ICACHE_RAM_ATTR interruptHandler() 
{
  static unsigned long last = micros();
  unsigned long now = micros();

  int dif = now - last; // time since last irq
  last = now;

  int trigger = 0;

  pulses_t& p = pulses_arr[cur_pulses_wr]; // current ring buffer entry

  if(dif < MIN_OOKLEN || dif > MAX_OOKLEN) // is a invalid (too short/long) pulse
  {
    if(p.num_pulses >= MIN_VALID_PULSES)
    {
      trigger = 1; // if there are enough good pulses in a row mark buffer as valid
    }
    else
    {
      p.num_pulses = 0; // error, restart buffer
      return;
    }
  }
  
  p.pulse[p.num_pulses++] = dif;

  if(!trigger && p.num_pulses < MAX_PULSES_NUM)
    return;

 // snprintf_P(log_data, sizeof(log_data), "RfPulse irq (pin: %i): frame %i, %i pulses", raw433_pin, cur_pulses_wr, p.num_pulses);
 // AddLog(LOG_LEVEL_INFO);

 // prepare next write buffer 
  if(++cur_pulses_wr >= SIZE_PULSES_ARR)
     cur_pulses_wr = 0;

  int pin = digitalRead(raw433_pin); // current pin state

  pulses_arr[cur_pulses_wr].initial_ook = pin;
  pulses_arr[cur_pulses_wr].num_pulses = 0;
}

///////////////////////////////////////////////////////////////////

#define LV_NUM      10

typedef struct last_values_t
{
  last_values_t()
  {
    when = 0;
    subtype = 0;
    topic[0] = 0;
    json[0] = 0;
  }

  unsigned long when;
  int           subtype;
  char          topic[60];
  char          json[200];
}last_values_t;

static last_values_t last_values[LV_NUM];

#define LV_TOO_FAST (5*1000)
#define LV_TIMEOUT  (30*60*1000)

// finds the exactly same but old enough,
//       otherwise an expired one
// return = 0: too fast, ignore
// return != 0: value set
static int setLastValue(struct last_values_t* nv)
{
  unsigned long now = millis();
  int t;

  for(t = 0; t < LV_NUM; t++)
  {
    last_values_t& e = last_values[t];

    if(nv->subtype == e.subtype && strcmp(nv->topic, e.topic) == 0)
    {
      int dif = now - e.when;

      if(dif < LV_TOO_FAST)
      {
        if(debug_val & deb_cache)
        {
          snprintf_P(log_data, sizeof(log_data), "setLastValue(#%i, %s, %i): %i repeat", t, e.topic, e.subtype, dif);
          AddLog(LOG_LEVEL_INFO);
        }
        return 0;
      }

      if(debug_val & deb_cache)
      {
        snprintf_P(log_data, sizeof(log_data), "setLastValue(#%i, %s, %i): %i refresh", t, e.topic, e.subtype, dif);
        AddLog(LOG_LEVEL_INFO);
      }

      e = *nv;
      e.when = now;

      return 1;
    }
  }

  int maxdif = 0;
  int maxix = 0;

  for(t = 0; t < LV_NUM; t++)
  {
    last_values_t& e = last_values[t];

    if(e.topic[0] && e.json[0]) // non empty entry
    {
      int dif = now - e.when;
      if(dif >= maxdif)
      {
        maxdif = dif;
        maxix = t; 
      }
    }
    else
    {
      maxix = t;
      break;
    }
    
  }

  if(debug_val & deb_cache)
  {
    snprintf_P(log_data, sizeof(log_data), "setLastValue(#%i, %s, %i): %i replace", maxix, nv->topic, nv->subtype, now - last_values[maxix].when);
    AddLog(LOG_LEVEL_INFO);
  }

  last_values[maxix]       = *nv;
  last_values[maxix].when  = now;

  return 1;
}

static void cleanupLastValues(void)
{
  unsigned long now = millis();
  int t;

  for(t = 0; t < LV_NUM; t++)
  {
    last_values_t& e = last_values[t];

    if(e.topic[0] == 0)
      continue;

    int dif = now - e.when;

    if(dif < LV_TIMEOUT)
      continue;

    if(debug_val & deb_cache)
    {
      snprintf_P(log_data, sizeof(log_data), "cleanupLastValues(#%i, %s, %i): %i expired", t, e.topic, e.subtype, now - e.when);
      AddLog(LOG_LEVEL_INFO);
    }

    e.topic[0] = 0;
    e.json[0] = 0;
  }
  
}
// mqtt output:
// mqtt: /tele/<tasmota_device>/OOK/device-21 = {"Time":"2020-08-02T12:26:40","type":"my decoder","Temperature":27.0,"Humidity":53.0,"BatteryGood":1}
//                                  111111111                                         2222222222

// last_values_t lvt;
// jsonStart(&lvt, "my decoder", "device-%i", device.addr) // prepare mqtt frame
//                  2222222222    111111111
//  optional: jsonTemperature(&lvt, 27.0)                  // add temperature to mqtt frame
//  optional: jsonHumidity(&lvt, 53.0)
//  optional: jsonBatteryGood(&lvt, true)
// jsonEnd(&lvt)                                         
// mqttSend(&lvt)

static void mqttSend(const last_values_t* v)
{
  ResponseTime_P(PSTR(",%s"), v->json);

  char prefix[100];
  sprintf(prefix, "OOK/%s", v->topic);

  ResponseJsonEnd();
  MqttPublishPrefixTopic_P(TELE, prefix);
}

void jsonStart(last_values_t* v, const char* type, const char* sensor, ...)
{
  va_list args;
  va_start(args, sensor);

  vsnprintf_P(v->topic, sizeof(v->topic), sensor, args);
  va_end(args);

  snprintf_P(v->json, sizeof(v->json), PSTR("\"type\":\"%s\","), type);
}

void jsonTemperature(last_values_t* v, float temp)
{
  char str[33];
  dtostrfd(temp, Settings.flag2.temperature_resolution, str);
  
  int mlen = strlen(v->json);
  snprintf_P(v->json + mlen, sizeof(v->json) - mlen, PSTR("\"" D_JSON_TEMPERATURE "\":%s,"  ), str);
}

void jsonHumidity(last_values_t* v, float hum)
{
  char str[33];
  dtostrfd(hum, Settings.flag2.temperature_resolution, str);
  int mlen = strlen(v->json);
  snprintf_P(v->json + mlen, sizeof(v->json) - mlen, PSTR("\"" D_JSON_HUMIDITY "\":%s,"  ), str);
}

void jsonBatteryGood(last_values_t* v, bool good)
{
  int mlen = strlen(v->json);
  snprintf_P(v->json + mlen, sizeof(v->json) - mlen, PSTR("\"BatteryGood\":%i,"  ), good ? 1 : 0);
}

void jsonEnd(last_values_t* v)
{
  int len = strlen(v->json);
  if(len)
    v->json[--len] = 0; // get rid of last ','
}

///////////////////////////////////////////////////////////////////

// CRC 4 table creation (nibble)
void createCRC4(uint8_t poly, uint8_t arr[16], uint8_t init)
{
  for(int t = 0; t < 16; t++)
  {
    int crc = init;
    int data = t;

    for(int t = 0; t < 4; t++)
		{
			crc <<= 1;
			data <<= 1;

			if((crc & 0x10) != (data & 0x10))
			{
				crc ^= poly;	
			}
		}
    arr[t] = crc;
  }
}

// crc4 NIBBLE (4 bit) calc, you must "crc &= 0x0f" yourself
// data is a 4 bit nibble!
void crc4(uint8_t poly[16], unsigned int data, unsigned int& crc)
{
  int ix = (crc ^ data) & 0xf;

  crc = poly[ix];
}

///////////////////////////////////////////////////////////////////

static struct
{
  int     num_data;
  uint8_t data[MAX_PULSES_NUM];
}ook_data;

///////////////////////////////////////////////////////////////////


static const char S_OOK_DEBUG1[]  PROGMEM = "pulses: %u, ook-bits: %u(min %u)";
//                                           0123456789abcdef0
static const char S_OOK_DEBUG2[]  PROGMEM = "|0              |16              |32              |48              |64              |80";

static void print_ook_debug(char* pr, int pr_len, int len, int min_bits)
{
  if(debug_val & deb_ook)
  {
    if(pr_len >= min_bits/2)
    {
      pr[pr_len] = 0;

      snprintf_P(log_data, sizeof(log_data), S_OOK_DEBUG2, pr);
      AddLog(LOG_LEVEL_INFO);

      snprintf_P(log_data, sizeof(log_data), S_OOK_DEBUG1, len, ook_data.num_data, min_bits);
      AddLog(LOG_LEVEL_INFO);

      snprintf_P(log_data, sizeof(log_data), "%s", pr);
      AddLog(LOG_LEVEL_INFO);
   }
  }
}

#define OOK_DEBUG_INIT char pr[MAX_PULSES_NUM*2+100]; int pr_len = 0


///////////////////////////////////////////////////////////////////

// OOK infactory
// _____                      _____                                          _____
//      |____________________|     |________________________________________|     |____
//|-500-|-------2000---------|-500-|----------------4000--------------------|-500-|----
//|-------------0------------|------------------------1---------------------|----------
// 0: short hi, 2000 low
// 1: short hi, 4000 low
static int decodeOOKI(int min_bits, uint16_t* pulses, int len, int min_hi, int max_hi, int min_0, int max_0, int min_1, int max_1)
{
  ook_data.num_data = 0;
  int s = 0;

  OOK_DEBUG_INIT;

  int ret = 0;

  for(int t = 0; t < len; t++)
  {
    int val = *pulses++;
    ++ret;

    if(s == 0)
    {
      if(val > min_hi && val < max_hi)
      {
     //   pr[pr_len++] = '>';
         s = 1;
      }
      else
      {
        pr[pr_len++] = '<';
        if(ook_data.num_data >= min_bits)
          break;

        pr_len = 0;
        ook_data.num_data = 0;
      }
    }
    else
    {
      if(val >= min_1 && val <= max_1)
      {
        pr[pr_len++] = '1';
        ook_data.data[ook_data.num_data++] = 1;
      }
      else if(val >= min_0 && val <= max_0)
      {
        pr[pr_len++] = '0';
        ook_data.data[ook_data.num_data++] = 0;      
      }
      else
      {
        pr[pr_len++] = 'X';
        if(ook_data.num_data >= min_bits)
          break;

        pr_len = 0;
        ook_data.num_data = 0;
        --t;
      }

      s = 0;
    }
  }

  print_ook_debug(pr, pr_len, len, min_bits);

  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OOK WT450H
// ______        ______        ________________                  __
//       |______|      |______|                |________________| 
//|-1000-|-1000-|-1000-|-1000-|------2000------|------2000------|
//|------1------|------1------|--------0-------|--------0-------|
// 0: long pulse (hi-time)
// 1: two short pulses (lo-time)
static int decodeOOK(int min_bits, uint16_t* pulses, int len, int min_1, int max_1, int min_0, int max_0)
{
  ook_data.num_data = 0;
  int s = 0;

  OOK_DEBUG_INIT;

  int ret = 0;

  for(int t = 0; t < len; t++)
  {
    int val = *pulses++;
    ++ret;

    if(val > min_0 && val < max_0)
    {
      pr[pr_len++] = '0';
      ook_data.data[ook_data.num_data++] = 0;
      s = 0;
    }
    else if(val > min_1 && val < max_1)
    {
      if(++s == 2)
      {
        pr[pr_len++] = '1';
        ook_data.data[ook_data.num_data++] = 1;
        s = 0;
      }
    }
    else
    {
      if(val >= max_0)
        pr[pr_len++] = s ? 'h' : 'H';
      else if(val <= min_1)
        pr[pr_len++] = s ? 'l' : 'L';
      else
        pr[pr_len++] = s ? 'm' : 'M';

      if(ook_data.num_data >= min_bits)
        break;

      pr_len = 0;
      ook_data.num_data = 0;
      s = 0;
    }
  }

  print_ook_debug(pr, pr_len, len, min_bits);

  return ret;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// LaCrosse TX3: https://www.f6fbb.org/domo/sensors/tx3_th.php
// _____            ______________
//      |__________|              |__________|
//|-550-|---1000---|-----1400-----|---1000---|
//|-------1--------|------------0------------|
// 0: long/long 1400/1000
// 1: short/long 550/1000
static int decodeOOK2(int min_bits, uint16_t* pulses, int len, int min_1, int max_1, int break_min, int break_max, int min_0, int max_0)
{
  // increasing values:
  // eg min_1=400, max_1=700, break_min=800, break_max=1200, min_0=1300, max_0=1500

  ook_data.num_data = 0;
  int s = 0;

  OOK_DEBUG_INIT;

  int ret = 0;

  for(int t = 0; t < len; t++)
  {
    int val = *pulses++;
    ++ret;

    if(s == 0) // 1st bit-half
    {
      if(val >= min_1 && val <= max_1)
      {
        ook_data.data[ook_data.num_data] = 1;
        ++s;
      }
      else if(val >= min_0 && val <= max_0)
      {
        ook_data.data[ook_data.num_data] = 0;
        ++s;
      }
      else
      {
        pr[pr_len++] = 'X'; 
        if(ook_data.num_data >= min_bits)
          break;
        pr_len = 0;
        ook_data.num_data = 0;
      }
    }
    else // 2nd bit-half
    {
      if(val >= break_min)
      {
        pr[pr_len++] =  ook_data.data[ook_data.num_data] ? '1' : '0';
        ++ook_data.num_data;

        if(val >= break_max)
        {
          pr[pr_len++] = 'P';
          break;
        }
      }
      else
      {
        --t; // retry as 1st half
        pr[pr_len++] = 'Y';
        if(ook_data.num_data >= min_bits)
          break;
        pr_len = 0;
        ook_data.num_data = 0;
      }

      s = 0;
    }
  }

  print_ook_debug(pr, pr_len, len, min_bits);

  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct 
{
  bool valid;
  uint8_t type;
  uint8_t addr;
  float   value;
}lacrosse_tx3_t;


static int decodeLaCrosse(bool initial_ook, uint16_t* pulses, int len)
{
  /*
// LaCrosse TX3: https://www.f6fbb.org/domo/sensors/tx3_th.php
  44 bits

  4-bit blocks:
  0000 preamble
  1010 preamble
  xxxx type (0000: temp, 1110: humidity)
  xxxx addr
  xxxP addr + parity (v1+v2+v3)
  tttt v1
  xxxx v2
  cccc v3
  tttt copy of v1
  xxxx copy of v2
  crc1 (nibblesum & 0x0f)

  value = v1*10 + v2 + v3 / 10
  temp = value - 50
  hum = value
  */

  int ret = 0;

  while(len > 0)
  {
    int offset = decodeOOK2(43, pulses, len, 440, 640, 900, 1100, 1200, 1600);
    pulses += offset;
    len -= offset;

    if(ook_data.num_data < 43) 
      continue;

    lacrosse_tx3_t pdata;

    // ook_num = 44 + x
    // loop tru 44 bit-blocks: 0..43, 1..44, x..44+x
    for(int offs = 0; offs <= ook_data.num_data - 43; offs++)
    {
      uint8_t* pos = &ook_data.data[offs];

      // header = 1100

      int parity = 0, t, v;
      unsigned int val;

      #define READ(NUM) val = 0; for(t = 0; t < NUM; t++) {v = *pos++; val <<= 1; val |= v; parity ^= v; }

      pdata.valid = false;

      // 1st bit might get lost in the receiver...    
      READ(7);

      if(val != 0x05) // bin 00000101
      {
        if(val != 0x0a) // 00001010
        {
          if(debug_val & deb_ookval)
          {
            snprintf_P(log_data, sizeof(log_data), "LaCrosse TX3: pos0 != 00001010 (%u%u%u%u(%x)", (val&8)>0, (val&4)>0, (val&2)>0, (val&1)>0, val);
            AddLog(LOG_LEVEL_INFO);
          }
          continue;
        }
        offs += 7;
      }
      else
      {
        val <<= 1; val |= *pos++; 
        if(val != 0x0a) // 00001010
        {
          if(debug_val & deb_ookval)
          {
            snprintf_P(log_data, sizeof(log_data), "LaCrosse TX3: pos0 != 00001010 (%u%u%u%u(%x)", (val&8)>0, (val&4)>0, (val&2)>0, (val&1)>0, val);
            AddLog(LOG_LEVEL_INFO);
          }
          continue;
        }
        offs += 8;
      }

      // pos 8
      unsigned int chk = val; // 0x0a

      READ(4);
      chk += val;

      if(val == 0) // temperature
      {
      }
      else if(val == 0x0e) // humidity
      {
      }
      else
      {
        offs += 12;

        if(debug_val & deb_ookval)
        {
          snprintf_P(log_data, sizeof(log_data), "LaCrosse TX3: nib 3 unknown type(%x)", (val&8)>0, (val&4)>0, (val&2)>0, (val&1)>0, val);
          AddLog(LOG_LEVEL_INFO);
        }
        continue;
      }

      pdata.type = val;

      // pos 12
      READ(4); chk += val;
      pdata.addr = val << 3;

      // pos 16
      READ(4); chk += val;
      pdata.addr |= val >> 1;
      
      int p1 = val & 1;

      // pos 20
      parity = 0;
      READ(4); chk += val;
      unsigned int m = val * 10;

      // pos 24
      READ(4); chk += val;
      m += val;
      pdata.value = m;

      // pos 28
      READ(4); chk += val;
      pdata.value += (float)val / 10;

      if(p1 == parity)
      {
        pdata.valid = true;
      }
      else
      {
        if(debug_val & deb_ookval)
        {
          snprintf_P(log_data, sizeof(log_data), "LaCrosse TX3: parity mismatch");
          AddLog(LOG_LEVEL_INFO);
        }
        continue;
      }
      
      // pos 32
      READ(4); chk += val;
      unsigned int m2 = val * 10;

      // pos 36
      READ(4); chk += val;
      m2 += val;

      if(m != m2)
      {
        if(debug_val & deb_ookval)
        {
          snprintf_P(log_data, sizeof(log_data), "LaCrosse TX3: val1 != val2 (%u, %u)", m, m2);
          AddLog(LOG_LEVEL_INFO);
        }
        continue;
      }

      if(pdata.type == 0) // temp
        pdata.value -= 50;
      
      // pos 40
      READ(4);
      chk &= 0xf;

      if(val != chk)
      {
        if(debug_val & deb_ookval)
        {
          snprintf_P(log_data, sizeof(log_data), "LaCrosse TX3: !chk %02x != %02x", val, chk);
          AddLog(LOG_LEVEL_INFO);
        }
        continue;
      }
      else
      {
        pdata.valid = true;
      }

      offs += 43-8;

      const char* type = "Temperature";

      last_values_t lv;

      jsonStart(&lv, PSTR("LaCrosse TX3"), PSTR("LaCrosse-TX3-%u"), pdata.addr);
      
      lv.subtype = pdata.type;

      if(pdata.type)
      {
        type = "Humidity";
        jsonHumidity(&lv, pdata.value);
      }
      else
      {
        jsonTemperature(&lv, pdata.value);
      }

      jsonEnd(&lv);

      if(setLastValue(&lv))
      {
        snprintf_P(log_data, sizeof(log_data), "LaCrosse TX3: type:%u addr:%u %s:%i.%i", pdata.type, pdata.addr, type, (int)pdata.value, (int)(pdata.value*10)%10);
        AddLog(LOG_LEVEL_INFO);
        mqttSend(&lv);
      }
      else
      {
        if(debug_val & deb_cache)
        {
          snprintf_P(log_data, sizeof(log_data), "ignored LaCrosse TX3: type:%u addr:%u", pdata.type, pdata.addr, type);
          AddLog(LOG_LEVEL_INFO);
        }
      }
      
      ++ret;
      break;
    }
  }

  return ret;

  #undef READ
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct xsns_100_rawOOK
{
  bool valid;
  uint8_t id;
  uint8_t channel;
  bool  batt_low;
  float temp;
  float hum;
}infactory_t;


static int decodeInfactory(bool initial_ook, uint16_t* pulses, int len)
{
  /*
  Thanks to rtl_433 
  40bits
  iiii iiii | cccc ub?? | tttt tttt | tttt hhhh | hhhh ??nn
- i: identification // changes on battery switch
- c: CRC-4 // CCITT checksum BUT: cccc is replaced by ??nn; crc = crc4(nibble[0]..nibble[7], crc ^= nibble[8]
- u: unknown // (sometimes set at power-on, but not always)
- b: battery low // flag to indicate low battery voltage
- h: Humidity // BCD-encoded, each nibble is one digit, 'A0' means 100%rH
- t: Temperature // in °F as binary number with one decimal place + 90 °F offset
- n: Channel // Channel number 1 - 3

  */
  static uint8_t poly[16];
  if(poly[1] == 0)
  {
    createCRC4(0x03, poly, 0);
  }

  infactory_t pdata;
 
  int ret = 0;

  while(len > 0)
  {
    int offset = decodeOOKI(40, pulses, len, 400, 800, 1400, 2600, 3400, 4600);
    pulses += offset;
    len -= offset;

    if(ook_data.num_data < 40) 
      continue;

    // ook_num = 40 + x
    // loop tru 40 bit-blocks: 0..39, 1..40, 2..41, x..40+x
  
    for(int offs = 0; offs <= ook_data.num_data - 40; offs++)
    {
      uint8_t* pos = &ook_data.data[offs];

      int t, v;
      unsigned int val, crc = 0;

      uint8_t n[10];

      for(t = 0; t < 10; t++)
      {
        val = 0; 
        for(v = 0; v < 4; v++) 
        {
          val <<= 1; 
          val |= *pos++; 
        } 
        n[t] = val;
      }

      pdata.valid = false;

      unsigned int crc1 = n[2];
      n[2] = n[9]; // cccc = ??nn

      for(t = 0; t < 8; t++)
        crc4(poly, n[t], crc);

      crc ^= n[8];
      crc &= 0xf;

      if(crc1 == crc)
      {
        pdata.valid = true;

        pdata.id = n[0] << 4 + n[1];

        pdata.batt_low = (n[3] & 4) != 0;

        pdata.temp = (n[4] << 8) + (n[5] << 4) + n[6];
        pdata.temp /= 10;
        pdata.temp -= 90; // Fahrenheit
        pdata.temp = (pdata.temp - 32) / 1.8;

        pdata.hum = n[7] * 10 + n[8];

        pdata.channel = n[9] & 3;
      }
      else
      {
        pdata.valid = false;
      }
    }

    if(pdata.valid)
    {
      last_values_t lv;

      jsonStart(&lv, PSTR("Infactory"), PSTR("Infactory-%u-%u"), pdata.id, pdata.channel);
      jsonTemperature(&lv, pdata.temp);
      jsonHumidity(&lv, pdata.hum);
      jsonBatteryGood(&lv, !pdata.batt_low);
      jsonEnd(&lv);

      if(setLastValue(&lv))
      {
        snprintf_P(log_data, sizeof(log_data), "Infactory: id:%u channel:%u humidity:%i temperature:%i.%i", pdata.id, pdata.channel, (int)pdata.hum, (int)pdata.temp, (int)(pdata.temp*10)%10);
        AddLog(LOG_LEVEL_INFO);

        mqttSend(&lv);
      }
      else
      {
        if(debug_val & deb_cache)
        {
          snprintf_P(log_data, sizeof(log_data), "ignored Infactory: id:%u channel:%u", pdata.id, pdata.channel);
          AddLog(LOG_LEVEL_INFO);
        }
      }

      ++ret;
      break;
    }
  }
  return ret;

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
  bool valid;
  unsigned long when;
  uint8_t house;
  uint8_t channel;
  bool battery_weak;
  float hum;
  float temp;
}pdata_t;

// 1: pulse 800..1200, 0: 1800..2200
static int decodeWT450H(bool initial_ook, uint16_t* pulses, int len)
{
  // Thanks to contribution of Johan Adler and Øyvind Kaurstad. 
  // http://ala-paavola.fi/jaakko/doku.php?id=wt450h
/*
  b00 - b03  (4 bits): Constant, 1100, probably preamble
  b04 - b07  (4 bits): House code 
  b08 - b09  (2 bits): Channel code - 1 
  b10 - b12  (3 bits): Constant, 110 battery ok, 111 battery weak
  b13 - b19  (7 bits): Relative humidity (here 0111011 = 59 %)
  b20 - b34 (15 bits): Temperature fixpoint 8.7 (temp + 50.0) * 128)
  b35 - b35  (1 bit) : Parity (xor of all bits should give 0)
*/

  int ret = 0;

  while(len > 0)
  {
    int offset = decodeOOK(35,  pulses, len, 800, 1200, 1800, 2200);
    pulses += offset;
    len -= offset;

    if(ook_data.num_data < 35)
      continue;
  
    pdata_t pdata;

    // ook_num = 36 + x
    // loop tru 36 bit-blocks: 0..35, 1..36, x..36+x

    for(int offs = 0; offs <= ook_data.num_data - 35; offs++)
    {
   // snprintf_P(log_data, sizeof(log_data), "WT450H: offs %i (%i)", offs, num_bits);
   // AddLog(LOG_LEVEL_INFO);

      uint8_t* pos = &ook_data.data[offs];

      // header = 1100

    // 1st bit might get lost in the receiver...    

      unsigned int val = *pos++; 
      val <<= 1; val |= *pos++; 
      val <<= 1; val |= *pos++; 

      if(val != 6) // bin 110
      {
        if(val != 4) // 100
        {
          if(debug_val & deb_ookval)
          {
            snprintf_P(log_data, sizeof(log_data), "WT450H: pos0 != 100 (%u%u%u%u(%x)", (val&8)>0, (val&4)>0, (val&2)>0, (val&1)>0, val);
            AddLog(LOG_LEVEL_INFO);
          }
          continue;
        }
        offs += 3;
      }
      else
      {
        val <<= 1; val |= *pos++; 
        if(val != 12) // 1100
        {
          if(debug_val & deb_ookval)
          {
            snprintf_P(log_data, sizeof(log_data), "WT450H: pos0 != 1100 (%u%u%u%u(%x)", (val&8)>0, (val&4)>0, (val&2)>0, (val&1)>0, val);
            AddLog(LOG_LEVEL_INFO);
          }
          continue;
        }
        offs += 4;
      }
      
      // header offs 10 = 110
      val = pos[6]; 
      val <<= 1; val |= pos[7]; 
      val <<= 1; val |= pos[8]; 

  //   snprintf_P(log_data, sizeof(log_data), "WT450H: header2: %u%u%u%u(%x)", (val&8)>0, (val&4)>0, (val&2)>0, (val&1)>0, val);
  //   AddLog(LOG_LEVEL_INFO);

      if(val < 6) // 110 or 111
      {
        if(debug_val & deb_ookval)
        {
          snprintf_P(log_data, sizeof(log_data), "WT450H: pos10 != 110 (%u%u%u%u(%x)", (val&8)>0, (val&4)>0, (val&2)>0, (val&1)>0, val);
          AddLog(LOG_LEVEL_INFO);
        }
        continue;
      }

      pdata.battery_weak = val & 1;

      int parity = 0, t, v;

      #define READ(NUM) val = 0; for(t = 0; t < NUM; t++) {v = *pos++; val <<= 1; val |= v; parity ^= v; }

      // 4 bit "house"
      READ(4)
      pdata.house = val;

      // 2 bit "channel"
      READ(2)
      pdata.channel = val;

      pos += 3; // header offs 10

      // 7 bit "humidity"
      READ(7)
      pdata.hum = val;

      // 15 bit "temperature"
      READ(15)

      // is fract 8.7 -> (temp + 50.0) * 128

      pdata.temp = (float)val / 128;
      pdata.temp -= 50;
  /*    
      // wanting xx.y
      val -= 6400;
      pdata.temp_fract = val * 10;
      pdata.temp_fix = val >> 7;
      
      pdata.temp_fract >>= 7;
      pdata.temp_fract %= 10; // 8.7 -> ,x
  */

      if(parity != *pos)
      {
        if(debug_val & deb_ookval)
        {
          snprintf_P(log_data, sizeof(log_data), "WT450H: parity mismatch");
          AddLog(LOG_LEVEL_INFO);
        }

        offs += 12-4;
      }
      else
      {
        pdata.valid = true;
        pdata.when = micros();
        
        last_values_t lv;
        jsonStart(&lv, PSTR("WT450H"), PSTR("WT450H-%u-%u"), pdata.house, pdata.channel);
        jsonTemperature(&lv, pdata.temp);
        jsonHumidity(&lv, pdata.hum);
        jsonBatteryGood(&lv, !pdata.battery_weak);
        jsonEnd(&lv);

        if(setLastValue(&lv))
        {
          snprintf_P(log_data, sizeof(log_data), "WT450H: house:%u channel:%u battery_weak:%i humidity:%i temperature:%i.%i", pdata.house, pdata.channel, (int)pdata.battery_weak, (int)pdata.hum, (int)pdata.temp, (int)(pdata.temp*10)%10);
          AddLog(LOG_LEVEL_INFO);

          mqttSend(&lv);
        }
        else
        {
          if(debug_val & deb_cache)
          {
            snprintf_P(log_data, sizeof(log_data), "ignored WT450H: house:%u channel:%u", pdata.house, pdata.channel);
            AddLog(LOG_LEVEL_INFO);
          }
        }

        ++ret;
        break;
      }
    }
  }

  return ret;

  #undef READ
}


/////////////////////////////////////////////////////////////////////////////////////

#define RFSNS_VALID_WINDOW        1800   // Number of seconds for sensor to respond (1800 = 30 minutes)

//#define RFSNS_LOOPS_PER_MILLI     1900   // (345 voor 16MHz ATMega) Voor 80MHz NodeMCU (ESP-12E). Getest met TheoV2 Protocol.
#define RFSNS_RAW_BUFFER_SIZE     180    // (256) Maximum number of RF pulses that can be captured
#define RFSNS_MIN_RAW_PULSES      112    // (16) =8 bits. Minimaal aantal ontvangen bits*2 alvorens cpu tijd wordt besteed aan decodering, etc.
                                         //   Zet zo hoog mogelijk om CPU-tijd te sparen en minder 'onzin' te ontvangen.
#define RFSNS_MIN_PULSE_LENGTH    300    // (50) Pulsen korter dan deze tijd uSec. worden als stoorpulsen beschouwd.
#define RFSNS_RAWSIGNAL_SAMPLE    50     // Sample grootte / Resolutie in uSec waarmee ontvangen Rawsignalen pulsen worden opgeslagen
#define RFSNS_SIGNAL_TIMEOUT      10     // Pulse timings in mSec. Beyond this value indicate end of message
#define RFSNS_SIGNAL_REPEAT_TIME  500    // (500) Tijd in mSec. waarbinnen hetzelfde event niet nogmaals via RF mag binnenkomen. Onderdrukt ongewenste herhalingen van signaal

typedef struct RawSignalStruct                   // Variabelen geplaatst in struct zodat deze later eenvoudig kunnen worden weggeschreven naar SDCard
{
  int  Number;                           // aantal bits, maal twee omdat iedere bit een mark en een space heeft.
  uint8_t Repeats;                          // Aantal maal dat de pulsreeks verzonden moet worden bij een zendactie.
  uint8_t Multiply;                         // Pulses[] * Multiply is de echte tijd van een puls in microseconden
  unsigned long Time;                    // Tijdstempel wanneer signaal is binnengekomen (millis())
  uint8_t Pulses[RFSNS_RAW_BUFFER_SIZE+2];  // Tabel met de gemeten pulsen in microseconden gedeeld door rfsns_raw_signal->Multiply. Dit scheelt helft aan RAM geheugen.
                                         // Om legacy redenen zit de eerste puls in element 1. Element 0 wordt dus niet gebruikt.
} raw_signal_t;

raw_signal_t *rfsns_raw_signal = nullptr;
uint8_t rfsns_any_sensor = 0;

/*********************************************************************************************\
 * Fetch signals from RF pin
\*********************************************************************************************/

bool RfSnsFetchSignal(struct pulses_t* p, bool StateSignal)
{
  int pulse_ix = 0;
  if(!!p->initial_ook != StateSignal)  // match the first pulse's bit
    ++pulse_ix;

  // Als het een herhalend signaal is, dan is de kans groot dat we binnen hele korte tijd weer in deze
  // routine terugkomen en dan midden in de volgende herhaling terecht komen. Daarom wordt er in dit
  // geval gewacht totdat de pulsen voorbij zijn en we met het capturen van data beginnen na een korte
  // rust tussen de signalen. Op deze wijze wordt het aantal zinloze captures teruggebracht.

  // google translator says:
  // If it is a repeating signal then chances are that we will be back in this
  // routine and then end up in the middle of the next iteration. Therefore, in this
  // case waited until the pulses are over and we start capturing data after a short
  // rest between signals. In this way, the number of pointless captures is reduced.

  // Thorstens opinion: this test should be moved to the final decoder, 
  // since it could be the last one in a series of frames decoded (when the first ones were faulty/indecodable)
  // and an OTHER sensor follows immediatly


  unsigned long PulseLength = 0;
  /*
  if (rfsns_raw_signal->Time) {                                           //  Eerst een snelle check, want dit bevindt zich in een tijdkritisch deel...
                                                                          // First a quick check, because this is in a time-critical part ...
    if (rfsns_raw_signal->Repeats && (rfsns_raw_signal->Time + RFSNS_SIGNAL_REPEAT_TIME) > millis()) {  // ...want deze check duurt enkele micro's langer!
                                                                                                        // ... because this check takes a few microphones longer!
      PulseLength = micros() + RFSNS_SIGNAL_TIMEOUT *1000;                // Wachttijd
      while (((rfsns_raw_signal->Time + RFSNS_SIGNAL_REPEAT_TIME) > millis()) && (PulseLength > micros())) {
        if ((*portInputRegister(Fport) & Fbit) == FstateMask) {
          PulseLength = micros() + RFSNS_SIGNAL_TIMEOUT *1000;
        }
      }
      while (((rfsns_raw_signal->Time + RFSNS_SIGNAL_REPEAT_TIME) > millis()) && ((*portInputRegister(Fport) & Fbit) != FstateMask));
    }
  }
*/
  int RawCodeLength = 1;                                                  // We starten bij 1, dit om legacy redenen. Vroeger had element 0 een speciaal doel.
                                                                          // We start at 1, for legacy reasons. Element 0 used to have a special purpose.
  bool Ftoggle = false;
  unsigned long maxtime = RFSNS_SIGNAL_TIMEOUT * 1000; // maximum length of signal in us
  unsigned long numtime = 0;                           // us since first bit
  rfsns_raw_signal->Multiply = RFSNS_RAWSIGNAL_SAMPLE;                    // Ingestelde sample groote.

  do {                                                                    // lees de pulsen in microseconden en plaats deze in de tijdelijke buffer rfsns_raw_signal
                                                                          // read the pulses in microseconds and put them in the temporary buffer rfsns_raw_signal
    PulseLength = p->pulse[pulse_ix++];                       // Bevat nu de pulslengte in microseconden
    Ftoggle = !Ftoggle;
    if (PulseLength < RFSNS_MIN_PULSE_LENGTH)
    {
      numtime = 0;
      RawCodeLength = 0;
    }
    else
    {
      numtime += PulseLength;
      rfsns_raw_signal->Pulses[RawCodeLength++] = PulseLength / (unsigned long)rfsns_raw_signal->Multiply;  // sla op in de tabel rfsns_raw_signal
    }
  }
  while(pulse_ix < p->num_pulses && RawCodeLength < RFSNS_RAW_BUFFER_SIZE && numtime <= maxtime);   // Zolang nog ruimte in de buffer, geen timeout en geen stoorpuls
                                                                          // As long as space in the buffer, no timeout and no interference pulse

  if ((RawCodeLength >= RFSNS_MIN_RAW_PULSES) && (RawCodeLength < RFSNS_RAW_BUFFER_SIZE -1)) {
    rfsns_raw_signal->Repeats = 0;                                        // Op dit moment weten we nog niet het type signaal, maar de variabele niet ongedefinieerd laten.
                                                                          // At this point we don't know the signal type yet, but don't leave the variable undefined.
    rfsns_raw_signal->Number = RawCodeLength -1;                          // Aantal ontvangen tijden (pulsen *2)
                                                                          // Number of times received (pulses * 2)

    rfsns_raw_signal->Pulses[rfsns_raw_signal->Number] = 0;               // Laatste element bevat de timeout. Niet relevant.
                                                                          // Last element contains the timeout. Non relevant.
    rfsns_raw_signal->Time = millis();
    return true;
  }
  else
    rfsns_raw_signal->Number = 0;


  return false;
}

#ifdef USE_THEO_V2
/*********************************************************************************************\
 * Theo V2 protocol
 * Dit protocol zorgt voor ontvangst van Theo sensoren met protocol V2
 *
 * Auteur             : Theo Arends
 * Support            : www.sidweb.nl
 * Datum              : 17 Apr 2014
 * Versie             : 0.1 - Initiele versie
 **********************************************************************************************
 * Technische informatie:
 *
 * Theo Sensor V2 type 1 Message Format (7 Bytes, 57 bits):
 *   Checksum Type Chl BsVoltag Temperature       Light
 * S AAAAAAAA BBBBBCCC DEFFFFFF GGGGGGGG GGGGGGGG HHHHHHHH HHHHHHHH
 * idx: 0        1        2        3        4        5        6
 *
 * Theo Sensor V2 type 2 Message Format (7 Bytes, 57 bits):
 *   Checksum Type Chl BsVoltag Temperature       Humidity
 * S AAAAAAAA BBBBBCCC DEFFFFFF GGGGGGGG GGGGGGGG HHHHHHHH HHHHHHHH
 * idx: 0        1        2        3        4        5        6
\*********************************************************************************************/

#define RFSNS_THEOV2_MAX_CHANNEL    2      // Max number of ATTiny sensor channels supported

#define RFSNS_THEOV2_PULSECOUNT     114
#define RFSNS_THEOV2_RF_PULSE_MID   1000   // PWM: Pulsen langer zijn '1'

typedef struct {
  uint32_t time;
  int16_t temp;
  uint16_t lux;
  uint8_t volt;
} theo_v2_t1_t;

typedef struct {
  uint32_t time;
  int16_t temp;
  uint16_t hum;
  uint8_t volt;
} theo_v2_t2_t;

theo_v2_t1_t *rfsns_theo_v2_t1 = nullptr;
theo_v2_t2_t *rfsns_theo_v2_t2 = nullptr;

void RfSnsInitTheoV2(void)
{
  rfsns_theo_v2_t1 = (theo_v2_t1_t*)malloc(RFSNS_THEOV2_MAX_CHANNEL * sizeof(theo_v2_t1_t));
  rfsns_theo_v2_t2 = (theo_v2_t2_t*)malloc(RFSNS_THEOV2_MAX_CHANNEL * sizeof(theo_v2_t2_t));
  rfsns_any_sensor++;
}

void RfSnsAnalyzeTheov2(void)
{
  if (rfsns_raw_signal->Number != RFSNS_THEOV2_PULSECOUNT) { return; }

  uint8_t Checksum;  // 8 bits Checksum over following bytes
  uint8_t Channel;   // 3 bits channel
  uint8_t Type;      // 5 bits type
  uint8_t Voltage;   // 8 bits Vcc like 45 = 4.5V, bit 8 is batt low
  int Payload1;      // 16 bits
  int Payload2;      // 16 bits

  uint8_t b, bytes, bits, id;

  uint8_t idx = 3;
  uint8_t chksum = 0;
  for (bytes = 0; bytes < 7; bytes++) {
    b = 0;
    for (bits = 0; bits <= 7; bits++)
    {
      if ((rfsns_raw_signal->Pulses[idx] * rfsns_raw_signal->Multiply) > RFSNS_THEOV2_RF_PULSE_MID) {
        b |= 1 << bits;
      }
      idx += 2;
    }
    if (bytes > 0) { chksum += b; }  // bereken checksum

    switch (bytes) {
    case 0:
      Checksum = b;
      break;
    case 1:
      id = b;
      Channel = b & 0x7;
      Type = (b >> 3) & 0x1f;
      break;
    case 2:
      Voltage = b;
      break;
    case 3:
      Payload1 = b;
      break;
    case 4:
      Payload1 = (b << 8) | Payload1;
      break;
    case 5:
      Payload2 = b;
      break;
    case 6:
      Payload2 = (b << 8) | Payload2;
      break;
    }
  }

  if (Checksum != chksum) { return; }
  if ((Channel == 0) || (Channel > RFSNS_THEOV2_MAX_CHANNEL)) { return; }
  Channel--;

  rfsns_raw_signal->Repeats = 1;  // het is een herhalend signaal. Bij ontvangst herhalingen onderdukken

  int Payload3 = Voltage & 0x3f;

  switch (Type) {
  case 1:   // Temp / Lux
    rfsns_theo_v2_t1[Channel].time = LocalTime();
    rfsns_theo_v2_t1[Channel].volt = Payload3;
    rfsns_theo_v2_t1[Channel].temp = Payload1;
    rfsns_theo_v2_t1[Channel].lux = Payload2;
    break;
  case 2:  // Temp / Hum
    rfsns_theo_v2_t2[Channel].time = LocalTime();
    rfsns_theo_v2_t2[Channel].volt = Payload3;
    rfsns_theo_v2_t2[Channel].temp = Payload1;
    rfsns_theo_v2_t2[Channel].hum = Payload2;
    break;
  }

  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("RFS: TheoV2, ChkCalc %d, Chksum %d, id %d, Type %d, Ch %d, Volt %d, BattLo %d, Pld1 %d, Pld2 %d"),
    chksum, Checksum, id, Type, Channel +1, Payload3, (Voltage & 0x80) >> 7, Payload1, Payload2);
}

void RfSnsTheoV2Show(bool json)
{
  bool sensor_once = false;

  for (uint32_t i = 0; i < RFSNS_THEOV2_MAX_CHANNEL; i++) {
    if (rfsns_theo_v2_t1[i].time) {
      char sensor[10];
      snprintf_P(sensor, sizeof(sensor), PSTR("TV2T1C%d"), i +1);
      char voltage[33];
      dtostrfd((float)rfsns_theo_v2_t1[i].volt / 10, 1, voltage);

      if (rfsns_theo_v2_t1[i].time < LocalTime() - RFSNS_VALID_WINDOW) {
        if (json) {
          ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_RFRECEIVED "\":\"%s\",\"" D_JSON_VOLTAGE "\":%s}"),
            sensor, GetDT(rfsns_theo_v2_t1[i].time).c_str(), voltage);
        }
      } else {
        char temperature[33];
        dtostrfd(ConvertTemp((float)rfsns_theo_v2_t1[i].temp / 100), Settings.flag2.temperature_resolution, temperature);

        if (json) {
          ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_TEMPERATURE "\":%s,\"" D_JSON_ILLUMINANCE "\":%d,\"" D_JSON_VOLTAGE "\":%s}"),
            sensor, temperature, rfsns_theo_v2_t1[i].lux, voltage);
#ifdef USE_DOMOTICZ
          if ((0 == tele_period) && !sensor_once) {
            DomoticzSensor(DZ_TEMP, temperature);
            DomoticzSensor(DZ_ILLUMINANCE, rfsns_theo_v2_t1[i].lux);
            sensor_once = true;
          }
#endif  // USE_DOMOTICZ
#ifdef USE_WEBSERVER
        } else {
          WSContentSend_PD(HTTP_SNS_TEMP, sensor, temperature, TempUnit());
          WSContentSend_PD(HTTP_SNS_ILLUMINANCE, sensor, rfsns_theo_v2_t1[i].lux);
#endif  // USE_WEBSERVER
        }
      }
    }
  }

  sensor_once = false;
  for (uint32_t i = 0; i < RFSNS_THEOV2_MAX_CHANNEL; i++) {
    if (rfsns_theo_v2_t2[i].time) {
      char sensor[10];
      snprintf_P(sensor, sizeof(sensor), PSTR("TV2T2C%d"), i +1);
      char voltage[33];
      dtostrfd((float)rfsns_theo_v2_t2[i].volt / 10, 1, voltage);

      if (rfsns_theo_v2_t2[i].time < LocalTime() - RFSNS_VALID_WINDOW) {
        if (json) {
          ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_RFRECEIVED" \":\"%s\",\"" D_JSON_VOLTAGE "\":%s}"),
            sensor, GetDT(rfsns_theo_v2_t2[i].time).c_str(), voltage);
        }
      } else {
        float temp = ConvertTemp((float)rfsns_theo_v2_t2[i].temp / 100);
        float humi = ConvertHumidity((float)rfsns_theo_v2_t2[i].hum / 100);

        if (json) {
          ResponseAppend_P(PSTR(",\"%s\":{"), sensor);
          ResponseAppendTHD(temp, humi);
          ResponseAppend_P(PSTR(",\"" D_JSON_VOLTAGE "\":%s}"), voltage);

          if ((0 == tele_period) && !sensor_once) {
#ifdef USE_DOMOTICZ
            DomoticzTempHumPressureSensor(temp, humi);  //
#endif  // USE_DOMOTICZ
#ifdef USE_KNX
            KnxSensor(KNX_TEMPERATURE, temp);
            KnxSensor(KNX_HUMIDITY, humi);
#endif  // USE_KNX
            sensor_once = true;
          }
#ifdef USE_WEBSERVER
        } else {
          WSContentSend_THD(sensor, temp, humi);
#endif  // USE_WEBSERVER
        }
      }
    }
  }
}

#endif  // USE_THEO_V2 ************************************************************************

#ifdef USE_ALECTO_V2
/*********************************************************************************************\
 * Alecto V2 protocol
 * Dit protocol zorgt voor ontvangst van Alecto weerstation buitensensoren
 *
 * Auteur             : Nodo-team (Martinus van den Broek) www.nodo-domotica.nl
 *                      Support ACH2010 en code optimalisatie door forumlid: Arendst
 * Support            : www.nodo-domotica.nl
 * Datum              : 25 Jan 2013
 * Versie             : 1.3
 **********************************************************************************************
 * Technische informatie:
 * DKW2012 Message Format: (11 Bytes, 88 bits):
 * AAAAAAAA AAAABBBB BBBB__CC CCCCCCCC DDDDDDDD EEEEEEEE FFFFFFFF GGGGGGGG GGGGGGGG HHHHHHHH IIIIIIII
 *                         Temperature Humidity Windspd_ Windgust Rain____ ________ Winddir  Checksum
 * A = start/unknown, first 8 bits are always 11111111
 * B = Rolling code
 * C = Temperature (10 bit value with -400 base)
 * D = Humidity
 * E = windspeed (* 0.3 m/s, correction for webapp = 3600/1000 * 0.3 * 100 = 108))
 * F = windgust (* 0.3 m/s, correction for webapp = 3600/1000 * 0.3 * 100 = 108))
 * G = Rain ( * 0.3 mm)
 * H = winddirection (0 = north, 4 = east, 8 = south 12 = west)
 * I = Checksum, calculation is still under investigation
 *
 * WS3000 and ACH2010 systems have no winddirection, message format is 8 bit shorter
 * Message Format: (10 Bytes, 80 bits):
 * AAAAAAAA AAAABBBB BBBB__CC CCCCCCCC DDDDDDDD EEEEEEEE FFFFFFFF GGGGGGGG GGGGGGGG HHHHHHHH
 *                         Temperature Humidity Windspd_ Windgust Rain____ ________ Checksum
 *
 * DCF Time Message Format: (NOT DECODED!)
 * AAAAAAAA BBBBCCCC DDDDDDDD EFFFFFFF GGGGGGGG HHHHHHHH IIIIIIII JJJJJJJJ KKKKKKKK LLLLLLLL MMMMMMMM
 *          11                 Hours   Minutes  Seconds  Year     Month    Day      ?        Checksum
 * B = 11 = DCF
 * C = ?
 * D = ?
 * E = ?
 * F = Hours BCD format (7 bits only for this byte, MSB could be '1')
 * G = Minutes BCD format
 * H = Seconds BCD format
 * I = Year BCD format (only two digits!)
 * J = Month BCD format
 * K = Day BCD format
 * L = ?
 * M = Checksum
\*********************************************************************************************/

#define RFSNS_DKW2012_PULSECOUNT       176
#define RFSNS_ACH2010_MIN_PULSECOUNT   160 // reduce this value (144?) in case of bad reception
#define RFSNS_ACH2010_MAX_PULSECOUNT   160

#define D_ALECTOV2                     "AlectoV2"

const char kAlectoV2Directions[] PROGMEM = D_TX20_NORTH "|"
                                           D_TX20_NORTH D_TX20_NORTH D_TX20_EAST "|"
                                           D_TX20_NORTH D_TX20_EAST "|"
                                           D_TX20_EAST D_TX20_NORTH D_TX20_EAST "|"
                                           D_TX20_EAST "|"
                                           D_TX20_EAST D_TX20_SOUTH D_TX20_EAST "|"
                                           D_TX20_SOUTH D_TX20_EAST "|"
                                           D_TX20_SOUTH D_TX20_SOUTH D_TX20_EAST "|"
                                           D_TX20_SOUTH "|"
                                           D_TX20_SOUTH D_TX20_SOUTH D_TX20_WEST "|"
                                           D_TX20_SOUTH D_TX20_WEST "|"
                                           D_TX20_WEST D_TX20_SOUTH D_TX20_WEST "|"
                                           D_TX20_WEST "|"
                                           D_TX20_WEST D_TX20_NORTH D_TX20_WEST "|"
                                           D_TX20_NORTH D_TX20_WEST "|"
                                           D_TX20_NORTH D_TX20_NORTH D_TX20_WEST;

typedef struct {
  uint32_t time;
  float temp;
  float rain;
  float wind;
  float gust;
  uint8_t type;
  uint8_t humi;
  uint8_t wdir;
} alecto_v2_t;

alecto_v2_t *rfsns_alecto_v2 = nullptr;
uint16_t rfsns_alecto_rain_base = 0;

void RfSnsInitAlectoV2(void)
{
  rfsns_alecto_v2 = (alecto_v2_t*)malloc(sizeof(alecto_v2_t));
  rfsns_any_sensor++;
}

void RfSnsAnalyzeAlectov2()
{
  if (!(((rfsns_raw_signal->Number >= RFSNS_ACH2010_MIN_PULSECOUNT) &&
         (rfsns_raw_signal->Number <= RFSNS_ACH2010_MAX_PULSECOUNT)) || (rfsns_raw_signal->Number == RFSNS_DKW2012_PULSECOUNT))) { return; }

  uint8_t c = 0;
  uint8_t rfbit;
  uint8_t data[9] = { 0 };
  uint8_t msgtype = 0;
  uint8_t rc = 0;
  int temp;
  uint8_t checksum = 0;
  uint8_t checksumcalc = 0;
  uint8_t maxidx = 8;
  unsigned long atime;
  float factor;
  char buf1[16];

  if (rfsns_raw_signal->Number > RFSNS_ACH2010_MAX_PULSECOUNT) { maxidx = 9; }
  // Get message back to front as the header is almost never received complete for ACH2010
  uint8_t idx = maxidx;
  for (uint32_t x = rfsns_raw_signal->Number; x > 0; x = x-2) {
    if (rfsns_raw_signal->Pulses[x-1] * rfsns_raw_signal->Multiply < 0x300) {
      rfbit = 0x80;
    } else {
      rfbit = 0;
    }
    data[idx] = (data[idx] >> 1) | rfbit;
    c++;
    if (c == 8) {
      if (idx == 0) { break; }
      c = 0;
      idx--;
    }
  }

  checksum = data[maxidx];
  checksumcalc = RfSnsAlectoCRC8(data, maxidx);

  msgtype = (data[0] >> 4) & 0xf;
  rc = (data[0] << 4) | (data[1] >> 4);

  if (checksum != checksumcalc) { return; }
  if ((msgtype != 10) && (msgtype != 5)) { return; }

  rfsns_raw_signal->Repeats = 1;  // het is een herhalend signaal. Bij ontvangst herhalingen onderdukken

//  Test set
//  rfsns_raw_signal->Number = RFSNS_DKW2012_PULSECOUNT;  // DKW2012
//  data[8] = 11;                                        // WSW

  factor = 1.22;  // (1.08)
//  atime = rfsns_raw_signal->Time - rfsns_alecto_time;
//  if ((atime > 10000) && (atime < 60000)) factor = (float)60000 / atime;
//  rfsns_alecto_time = rfsns_raw_signal->Time;
//  Serial.printf("atime %d, rfsns_alecto_time %d\n", atime, rfsns_alecto_time);

  rfsns_alecto_v2->time = LocalTime();
  rfsns_alecto_v2->type = (RFSNS_DKW2012_PULSECOUNT == rfsns_raw_signal->Number);
  rfsns_alecto_v2->temp = (float)(((data[1] & 0x3) * 256 + data[2]) - 400) / 10;
  rfsns_alecto_v2->humi = data[3];
  uint16_t rain = (data[6] * 256) + data[7];
  // check if rain unit has been reset!
  if (rain < rfsns_alecto_rain_base) { rfsns_alecto_rain_base = rain; }
  if (rfsns_alecto_rain_base > 0) {
    rfsns_alecto_v2->rain += ((float)rain - rfsns_alecto_rain_base) * 0.30;
  }
  rfsns_alecto_rain_base = rain;
  rfsns_alecto_v2->wind = (float)data[4] * factor;
  rfsns_alecto_v2->gust = (float)data[5] * factor;
  if (rfsns_alecto_v2->type) {
    rfsns_alecto_v2->wdir = data[8] & 0xf;
  }

  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("RFS: " D_ALECTOV2 ", ChkCalc %d, Chksum %d, rc %d, Temp %d, Hum %d, Rain %d, Wind %d, Gust %d, Dir %d, Factor %s"),
    checksumcalc, checksum, rc, ((data[1] & 0x3) * 256 + data[2]) - 400, data[3], (data[6] * 256) + data[7], data[4], data[5], data[8] & 0xf, dtostrfd(factor, 3, buf1));
}

void RfSnsAlectoResetRain(void)
{
  if ((RtcTime.hour == 0) && (RtcTime.minute == 0) && (RtcTime.second == 5)) {
    rfsns_alecto_v2->rain = 0;  // Reset Rain
  }
}

/*********************************************************************************************\
 * Calculates CRC-8 checksum
 * reference http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/
 *           http://lucsmall.com/2012/04/30/weather-station-hacking-part-3/
 *           https://github.com/lucsmall/WH2-Weather-Sensor-Library-for-Arduino/blob/master/WeatherSensorWH2.cpp
 \*********************************************************************************************/
uint8_t RfSnsAlectoCRC8(uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint32_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (mix) { crc ^= 0x31; }
      inbyte <<= 1;
    }
  }
  return crc;
}

#ifdef USE_WEBSERVER
const char HTTP_SNS_ALECTOV2[] PROGMEM =
  "{s}" D_ALECTOV2 " " D_RAIN "{m}%s " D_UNIT_MILLIMETER "{e}"
  "{s}" D_ALECTOV2 " " D_TX20_WIND_SPEED "{m}%s " D_UNIT_KILOMETER_PER_HOUR "{e}"
  "{s}" D_ALECTOV2 " " D_TX20_WIND_SPEED_MAX "{m}%s " D_UNIT_KILOMETER_PER_HOUR "{e}";
const char HTTP_SNS_ALECTOV2_WDIR[] PROGMEM =
  "{s}" D_ALECTOV2 " " D_TX20_WIND_DIRECTION "{m}%s{e}";
#endif

void RfSnsAlectoV2Show(bool json)
{
  if (rfsns_alecto_v2->time) {
    if (rfsns_alecto_v2->time < LocalTime() - RFSNS_VALID_WINDOW) {
      if (json) {
        ResponseAppend_P(PSTR(",\"" D_ALECTOV2 "\":{\"" D_JSON_RFRECEIVED "\":\"%s\"}"), GetDT(rfsns_alecto_v2->time).c_str());
      }
    } else {
      float temp = ConvertTemp(rfsns_alecto_v2->temp);
      float humi = ConvertHumidity((float)rfsns_alecto_v2->humi);

      char rain[33];
      dtostrfd(rfsns_alecto_v2->rain, 2, rain);
      char wind[33];
      dtostrfd(rfsns_alecto_v2->wind, 2, wind);
      char gust[33];
      dtostrfd(rfsns_alecto_v2->gust, 2, gust);
      char wdir[4];
      char direction[20];
      if (rfsns_alecto_v2->type) {
        GetTextIndexed(wdir, sizeof(wdir), rfsns_alecto_v2->wdir, kAlectoV2Directions);
        snprintf_P(direction, sizeof(direction), PSTR(",\"Direction\":\"%s\""), wdir);
      }

      if (json) {
        ResponseAppend_P(PSTR(",\"" D_ALECTOV2 "\":{"));
        ResponseAppendTHD(temp, humi);
        ResponseAppend_P(PSTR(",\"Rain\":%s,\"Wind\":%s,\"Gust\":%s%s}"), rain, wind, gust, (rfsns_alecto_v2->type) ? direction : "");

        if (0 == tele_period) {
#ifdef USE_DOMOTICZ
        // Use a rules to send data to Domoticz where also a local BMP280 is connected:
        // on tele-alectov2#temperature do var1 %value% endon on tele-alectov2#humidity do var2 %value% endon on tele-bmp280#pressure do publish domoticz/in {"idx":68,"svalue":"%var1%;%var2%;0;%value%;0"} endon
        // on tele-alectov2#wind do var1 %value% endon on tele-alectov2#gust do publish domoticz/in {"idx":69,"svalue":"0;N;%var1%;%value%;22;24"} endon"}
        // on tele-alectov2#rain do publish domoticz/in {"idx":70,"svalue":"0;%value%"} endon
#endif  // USE_DOMOTICZ
        }
#ifdef USE_WEBSERVER
      } else {
        WSContentSend_THD(D_ALECTOV2, temp, humi);
        WSContentSend_PD(HTTP_SNS_ALECTOV2, rain, wind, gust);
        if (rfsns_alecto_v2->type) {
          WSContentSend_PD(HTTP_SNS_ALECTOV2_WDIR, wdir);
        }
#endif  // USE_WEBSERVER
      }
    }
  }
}
#endif  // USE_ALECTO_V2 **********************************************************************

void RfSnsInit(void)
{
  rfsns_raw_signal = (raw_signal_t*)(malloc(sizeof(raw_signal_t)));
  if (rfsns_raw_signal) {
    memset(rfsns_raw_signal, 0, sizeof(raw_signal_t));  // Init defaults to 0
#ifdef USE_THEO_V2
    RfSnsInitTheoV2();
#endif
#ifdef USE_ALECTO_V2
    RfSnsInitAlectoV2();
#endif
    if (rfsns_any_sensor) {
      pinMode(Pin(GPIO_RF_SENSOR), INPUT);
    } else {
      free(rfsns_raw_signal);
      rfsns_raw_signal = nullptr;
    }
  }
}

void RfSnsAnalyzeRawSignal(void)
{
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("RFS: Pulses %d"), (int)rfsns_raw_signal->Number);

#ifdef USE_THEO_V2
    RfSnsAnalyzeTheov2();
#endif
#ifdef USE_ALECTO_V2
    RfSnsAnalyzeAlectov2();
#endif
}

void RfSnsEverySecond(void)
{
#ifdef USE_ALECTO_V2
  RfSnsAlectoResetRain();
#endif
}

void RfSnsShow(bool json)
{
#ifdef USE_THEO_V2
  RfSnsTheoV2Show(json);
#endif
#ifdef USE_ALECTO_V2
  RfSnsAlectoV2Show(json);
#endif
}


/////////////////////////

// user context: scan tru pulse buffers
void checkPulses(void)
{
  cleanupLastValues();

  // limit parsing to some frames (irq may be filling fast)

  for(int t = 0; t < SIZE_PULSES_ARR + 2; t++)
  {
    if(cur_pulses_wr == cur_pulses_rd)
      return;

    pulses_t& p = pulses_arr[cur_pulses_rd];

    if(debug_val & deb_pulses)
    {
      snprintf_P(log_data, sizeof(log_data), "[%i] pulses: num %u, start: %i", cur_pulses_rd, p.num_pulses, (int)p.initial_ook);
      AddLog(LOG_LEVEL_INFO);

      int pn = p.num_pulses;
      uint16_t* tp = p.pulse;

      while(pn > 0)
      {
        for(int i = 0; i < 32; i++)
        {
          snprintf_P(log_data + 5*i, sizeof(log_data)-5*i, "%4u ", *tp);
          ++tp;
          if(--pn <= 0)
            break;
        }

        AddLog(LOG_LEVEL_INFO);
      }
    }

    if(++cur_pulses_rd >= SIZE_PULSES_ARR)
      cur_pulses_rd = 0;

    // add your decoder here...

    if(decodeInfactory(p.initial_ook, p.pulse, p.num_pulses))
      continue;
    if(decodeLaCrosse(p.initial_ook, p.pulse, p.num_pulses))
      continue;
    if(decodeWT450H(p.initial_ook, p.pulse, p.num_pulses))
      continue;

    if (RfSnsFetchSignal(&p, HIGH)) 
    {
      RfSnsAnalyzeRawSignal();
    }

  }
}

//////////////

const char OOK_DEBUG[] PROGMEM = "{\"OOK_DEBUG\":{%u}}";

static bool ook_Command(void)
{
  bool ret = false;
  
  if (XdrvMailbox.data_len == 0) 
  {
    return ret;
  }
  
  int paramcount = 0;

  for (uint32_t ca=0;ca<XdrvMailbox.data_len;ca++) 
  {
    if ((' ' == XdrvMailbox.data[ca]) || ('=' == XdrvMailbox.data[ca]) ||  (',' == XdrvMailbox.data[ca])) 
    { 
      XdrvMailbox.data[ca] = ','; 
      ++paramcount; 
    }
  }

  UpperCase(XdrvMailbox.data, XdrvMailbox.data);
  
  char sub_string[XdrvMailbox.data_len];

  if (!strcmp(subStr(sub_string, XdrvMailbox.data, ",", 1),"DEBUG")) 
  { // Note 1 used for param number
    ret = true;
    if(paramcount)
    {
      debug_val = (unsigned int)atoi(subStr(sub_string, XdrvMailbox.data, ",", 2));  // Note 2 used for param number
    }
    snprintf_P(log_data, sizeof(log_data), "debug: %u", debug_val);
    AddLog(LOG_LEVEL_INFO);

    Response_P(OOK_DEBUG, debug_val);
  }
  return ret;
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns37(uint8_t function)
{
  bool result = false;

  if (PinUsed(GPIO_RF_SENSOR) && (FUNC_INIT == function)) 
  {
    RfPulseInit();
    RfSnsInit();
  }
  else if (rfsns_raw_signal) {
    switch (function) {
      case FUNC_LOOP:
       
        break;
      case FUNC_EVERY_SECOND:
        RfSnsEverySecond();
        checkPulses ();
        break;
      case FUNC_JSON_APPEND:
        RfSnsShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        RfSnsShow(0);
        break;
#endif  // USE_WEBSERVER
    case FUNC_COMMAND_SENSOR:
    {
      if (XSNS_37 == XdrvMailbox.index) 
      {
        result = ook_Command();
      }
    }
    break;

    }
  }
  return result;
}

#endif  // USE_RF_SENSOR
