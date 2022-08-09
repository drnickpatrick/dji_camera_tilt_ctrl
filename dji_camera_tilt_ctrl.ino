struct spd_ctrl{
  float spd;
  byte  byt;
};

struct dur_dist{
  float dur;
  float dist;
};

// 0-77 index
const spd_ctrl spd_up[] PROGMEM =  {
  {2.43522904  ,177},
  {2.708766985  ,178},
  {2.850833303  ,178},
  {3.146030365  ,180},
  {3.377479454  ,181},
  {3.700082791  ,182},
  {3.700082791  ,182},
  {4.126901187  ,183},
  {4.305261053  ,185},
  {4.771064024  ,186},
  {4.867730561  ,186},
  {5.369238851  ,188},
  {5.473257834  ,188},
  {5.578543326  ,188},
  {5.902116295  ,190},
  {6.124379453  ,190},
  {6.237510615  ,193},
  {6.585043357  ,194},
  {6.823631998  ,194},
  {7.192100051  ,195},
  {7.444939152  ,196},
  {8.102900004  ,197},
  {8.102900004  ,197},
  {8.656645798  ,198},
  {9.088418349  ,199},
  {9.840337228  ,201},
  {9.99567139   ,202},
  {10.63390077  ,203},
  {11.13063898  ,204},
  {11.6432148   ,205},
  {11.99388726  ,205},
  {12.5335637   ,206},
  {12.90259682  ,208},
  {13.85823557  ,209},
  {14.05512401  ,210},
  {15.06901547  ,211},
  {15.48853427  ,212},
  {16.57305886  ,213},
  {16.57305886  ,213},
  {17.47820197  ,214},
  {18.17944119  ,215},
  {19.39174489  ,217},
  {19.64084707  ,218},
  {20.92021105  ,219},
  {21.44797396  ,220},
  {21.98503055  ,220},
  {22.80824657  ,221},
  {23.08740053  ,222},
  {23.93927264  ,224},
  {25.40779826  ,225},
  {25.70891989  ,226},
  {26.93851955  ,226},
  {27.8874169   ,228},
  {29.188813    ,229},
  {29.52070453  ,229},
  {30.87479303  ,230},
  {31.91853667  ,231},
  {33.3483674   ,233},
  {33.7127199   ,233},
  {35.57643844  ,234},
  {36.72865717  ,236},
  {37.90675413  ,236},
  {38.30525864  ,237},
  {39.92861746  ,238},
  {40.75806573  ,240},
  {42.88426583  ,241},
  {43.31861341  ,241},
  {45.536535    ,242},
  {46.44547523  ,244},
  {48.77307686  ,245},
  {48.77307686  ,245},
  {51.18078288  ,246},
  {52.16662333  ,247},
  {54.68894163  ,249},
  {55.20339361  ,249},
  {57.82624097  ,250},
  {58.89920661  ,252},
  {61.64200951  ,253},
  };

// 0-77 index
const spd_ctrl spd_down[] PROGMEM =  {
  {2.93870224, 79},
  {3.284818786, 78},
  {3.427868119, 77},
  {3.79757432,  76},
  {3.873652747, 75},
  {4.26518345,  74},
  {4.427147352  ,73},
  {4.846012239  ,72},
  {4.846012239  ,72},
  {5.196001162  ,70},
  {5.467528428  ,69},
  {5.842034673  ,68},
  {5.937942866  ,68},
  {6.431585101  ,66},
  {6.533191411  ,66},
  {6.739347732  ,65},
  {6.949483852  ,62},
  {7.163665609  ,64},
  {7.272294175  ,64},
  {7.83114308   ,60},
  {8.416923164  ,58},
  {9.030663143  ,55},
  {9.284218338  ,55},
  {9.938830415  ,54},
  {9.938830415  ,54},
  {10.48437945  ,53},
  {10.90663817  ,52},
  {11.48753485  ,50},
  {11.78579635  ,50},
  {12.39827486  ,49},
  {12.87185405  ,47},
  {13.35786897  ,46},
  {13.52267584  ,46},
  {14.19613228  ,44},
  {14.5415131   ,44},
  {15.43072994  ,42},
  {15.61305597  ,42},
  {16.54755206  ,40},
  {16.93217489  ,40},
  {17.92136903  ,38},
  {17.92136903  ,38},
  {18.7416942   ,37},
  {19.37418282  ,36},
  {20.46183599  ,34},
  {20.68446652  ,34},
  {21.59226259  ,33},
  {22.29147774  ,32},
  {22.52809203  ,32},
  {23.24867785  ,29},
  {23.73810314  ,29},
  {24.48595781  ,28},
  {25.50904757  ,26},
  {25.76950883  ,26},
  {27.10035963  ,25},
  {27.64617431  ,23},
  {29.04497303  ,22},
  {29.04497303  ,22},
  {30.49354044  ,21},
  {31.08713307  ,20},
  {32.6071049   ,18},
  {32.91733485  ,18},
  {34.50011526  ,17},
  {35.148142    ,15},
  {36.13636774  ,14},
  {36.80607134  ,14},
  {37.82712101  ,13},
  {38.8681613   ,12},
  {40.28769559  ,10},
  {41.01110255  ,10},
  {42.48552528  ,9 },
  {43.61575927  ,8 },
  {45.15571632  ,6 },
  {45.15571632  ,6 },
  {47.1344043   ,5 },
  {47.94281034  ,4 },
  {50.00672868  ,2 },
  {50.42693303  ,2 },
  {52.56551077  ,1 },
  };

const size_t SPD_SZ = 78;
////////////////////////////////////////////////////////////////////////////////////////////////////
#include "mavlink.h"

#include <HampelFilter.h>

#define PWM_IN_PIN 12

#define BAUD_DBG 57600
#define BAUD_PIX 57600

// uncomment for productive use: no logging, mavlink communication via hardware Serial
#define PROF_PROD

//const------------------------------------------------
const byte STOP_BYT = 127;
const unsigned long wait_for_camera_ready = 15000;
const char param_speed_id[16] = "CAM_MAX_ROLL";     // pixhawk's parameter to store tilt speed [1-5]
const char param_smoth_id[16] = "CAM_MIN_INTERVAL"; // pixhawk's parameter to store tilt smoothness [1-5]
//-----------------------------------------------------
#ifdef PROF_PROD
#define LOG(_q)      // for prod.
#else
#define LOG(_q) _q // for debug
#endif
//#define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed

#define LOG2(_q1, _q2) LOG(Serial.print(_q1); Serial.println(_q2);)

#ifdef SOFT_SERIAL_DEBUGGING
  // Library to use serial debugging with a second board
  #include <SoftwareSerial.h>
  SoftwareSerial pxSerial(9,10);   // RX, TX
#endif

volatile bool LED_STATE = 0; // builtin led state

// PWM handling vars -----------------------------------
//                              default  sz  thres
HampelFilter hfilt = HampelFilter(0.00, 5, 1.0);


// ISR handled variables -------------------------------
volatile unsigned long tm_last = 0; // last time the ISR proceed
volatile float target_angle = 90; // degr (0-90)
volatile float current_angle = 90; // degr(0.0-90.0)
volatile byte dac_value = STOP_BYT;
volatile int dir = 0;
//volatile bool target_changed = 0;
volatile float current_target_speed = -1;
volatile float dist_acc = 0.0; // distance during accelerating

// parameters ----------------------------------------------------------
int tilt_speed = 4; // [1-5]->[9.0-20.0] // tild speed
int tilt_smoth = 2; // [1-5]->[20.0-3.0] // acceleration limit
float target_speed = -1;
float acceleration = -1;
const float min_speed = 3.0;
bool camera_ready = false;

// Mavlink stuff (C) https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566
bool tilt_speed_set = false;
bool tilt_smoth_set = false;
byte try_counter = 0;
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 10;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

//------------------------------------------------------------------------
void setup() {
  LOG(Serial.println("Setup");)

#ifdef SOFT_SERIAL_DEBUGGING
  Serial.begin(BAUD_DBG);
  pxSerial.begin(BAUD_PIX);
#else
  Serial.begin(BAUD_PIX);
#endif

  cli();                      //stop interrupts for till we make the settings
  //1. First we reset the control register to amke sure we start with everything disabled.
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
 
  //2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. 
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  
  //*3. We enable compare match mode on register A
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 

/*Calculations (for 500ms): 
  System clock 16 Mhz and Prescalar 256;
  Timer 1 speed = 16Mhz/256 = 62.5 Khz    
  Pulse time = 1/62.5 Khz =  16us  
  Count up to = 500ms / 16us = 31250 (so this is the value the OCR register should have)*/  
  //4. Set the value of register A to 31250
  OCR1A = 3125;             //Finally we set compare register A to this value  

// set up DAC pins
  DDRD = B11111100;  //Digital pins 2 to 7 as output
#ifdef SOFT_SERIAL_DEBUGGING
  DDRB = B00101101; //Digital pin 8 to 13 as output except 12 - for PWM input, and 9 for pxSerial RX
#else
  DDRB = B00101111; //Digital pin 8 to 13 as output except 12 - for PWM input
#endif

  wrt( STOP_BYT );

  digitalWrite(PWM_IN_PIN, HIGH); // set up PWM pin & interrupt
  
  tm_last = micros( );
  sei();                     //Enable back the interrupts 
}

void loop() {

  if(camera_ready){
    unsigned int pwm_pulse = pulseIn(PWM_IN_PIN, HIGH);
    float pwmf = 0;
    int angle = 0;
    if(pwm_pulse > 800 and pwm_pulse < 2100){
      digitalWrite(LED_BUILTIN, HIGH);
//    Hampel filter
      float ang = mymap(float(pwm_pulse), 1882.0, 1090.0, 90.0, 0.0);
      hfilt.write(ang);
      angle = hfilt.readMedian();

      set_new_target( angle );
      digitalWrite(LED_BUILTIN, LOW);
    }
    LOG(Serial.print(pwm_pulse);
        Serial.print('\t');
        Serial.print(pwmf);
        Serial.print('\t');
        Serial.println(angle); )
/*    else{
      int a2v = analogRead(A2);
      int new_targ_ang = map(a2v, 0, 1024, 0, 90);
      set_new_target( float(new_targ_ang) );
    }*/
    delay(100);
  }
  else{ // check camera ready conditions
#ifdef PROF_PROD
/*  if(!tilt_speed_set or !tilt_smoth_set){
    if(try_counter < 7){ // request parameters via mavlink
      digitalWrite(LED_BUILTIN, HIGH);
      mav_req_params( );
      digitalWrite(LED_BUILTIN, LOW);
    }
  }*/
#endif
    if(tilt_speed_set and tilt_smoth_set){
      blink_value(tilt_speed);
      blink_value(tilt_smoth);
  LOG(Serial.println("All the parameters are set!");)
      set_params();
    }

    if(millis() > wait_for_camera_ready){
      set_params();
      camera_ready = true;
      LOG(Serial.println("Camera is ready!");)
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void set_params(){
  // set target speed & acceleration
  target_speed = map(tilt_speed, 1, 5, 900, 2000)/100.0; // [1-5]->[9,0-20.0]
  LOG2("target_speed:", target_speed)
  acceleration = map(tilt_smoth, 1, 5, 2000, 300)/100.0; // [1-5]->[20.0-3.0]
  LOG2("acceleration:", acceleration)
}

void set_new_target(float new_target){
  if(new_target > 90.0)new_target = 90.0;
  if(new_target < 0.0) new_target = 0.0;
//  if(my_abs(new_target - target_angle) > 3.0){
    target_angle = new_target;
//    target_changed = true;
//  }
}


//Camera movement handling ISR
//With the settings above, this IRS will trigger each 100ms (328P 3.3v 8MHz). -----------------
ISR(TIMER1_COMPA_vect)
{
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt

  float new_speed = -1;
  byte new_byt = 126; // never-used value

  int a0v = analogRead(0); // get DAC voltage
  float current_speed = 0;
  if(dir > 0){
    current_speed = calc_spd_up( a0v );
  }
  if(dir < 0){
    current_speed = calc_spd_down( a0v );
  }

  dur_dist dd = calc_dist(current_speed);

  if(dir == 0){
    if(my_abs(target_angle - current_angle) > 2.0){
      /*LOG(Serial.print("target changed:");
          Serial.print(current_angle);
          Serial.print('\t');
          Serial.print(target_angle);
          Serial.println(); )*/
      // calc direction & speed
      if(current_angle < target_angle) new_byt = go_up( min_speed );
      if(current_angle > target_angle) new_byt = go_down( min_speed );
      if(new_byt != 126){
        current_target_speed = min_speed;
/*        LOG(Serial.print("new target byt got:");
            Serial.print(new_byt);
            Serial.println(); )*/
      }
    }
  }
  else{
    if(dir > 0){
      if(current_angle >= target_angle || current_angle >= 90.0 ) new_byt = STOP_BYT; // we reach target - stop here
    }
    if(dir < 0){
      if(current_angle <= target_angle || current_angle <= 0.0 ) new_byt = STOP_BYT; // we reach target - stop here
    }

    if(new_byt != STOP_BYT and current_target_speed > 0){ // smooth movememts
      // calculate the remaining angle distance and decide is it right time for slowing down
      float rem_dist = my_abs(current_angle - target_angle);
      if(rem_dist <= dist_acc){ //decelerating
        new_speed = current_target_speed - acceleration * dd.dur;
      }
      else{ // accelerating
        if(current_target_speed > 0 && current_target_speed < target_speed){
          new_speed = current_target_speed + acceleration * dd.dur;
          dist_acc += dd.dist; // accumulate the distance during acceleration
        }
      }

      if(new_speed > 0.0){
        if(dir > 0) new_byt = go_up(new_speed);
        else        new_byt = go_down(new_speed);

        current_target_speed = new_speed;
/*        LOG(Serial.print("N:");
            Serial.print(new_speed);
            Serial.print('\t');
            Serial.print(current_speed);
            Serial.println();)*/
      }
    }

  }
  calc_dist(current_speed); // calc again to take into account duration of the ISR
  if(new_byt != 126){
    wrt(new_byt);
//    LED_STATE = !LED_STATE;      //Invert LED state
//    digitalWrite(LED_BUILTIN,LED_STATE);  //Write new state
  }
}

// calculate the distance the camera moves since the last ISR
dur_dist calc_dist(const float current_speed){
  dur_dist result;
// get duration since last call in ms
  unsigned long tm_cur = micros( ); // current time
  unsigned long dur_us = get_dur(tm_cur, tm_last);
  float dur = float(dur_us)/1000000.0; // duration in S
  tm_last = tm_cur; // assign new last value

// calculate distance (degr) based on current speed and set new value of current angle
  float dist = current_speed * dur;

/*LOG(Serial.print("dur_ms/dur/dist:");
    Serial.print(dur_us);
    Serial.print('\t');
    Serial.print(dur);
    Serial.print('\t');
    Serial.print(dist);
    Serial.println();)*/

  if(dir > 0){
    current_angle += dist;
  }
  if(dir < 0){
    current_angle -= dist;
  }

  result.dur = dur;
  result.dist = dist;
  return result;
}

unsigned long get_dur(const unsigned long tm_cur, const unsigned long tm_prv){ // calculate duration in ms from tm_cur to tm_last
  if(tm_prv > 0){
    unsigned long dur = 0; 
    if( tm_cur >= tm_prv) dur = (tm_cur - tm_prv); // normal flow
    else dur = ( 0xFFFFFFFF - tm_prv + tm_cur ); // counter is overflowed
    return dur;
  }
  else return 0;
}

byte go_up(float sp){
   const spd_ctrl ctrl = find_speed( spd_up, sp );
   if(ctrl.spd > 0) return ctrl.byt;
   else return STOP_BYT;
}

byte go_down(float sp){
   const spd_ctrl ctrl = find_speed( spd_down, sp );
   if(ctrl.spd > 0) return ctrl.byt;
   else return STOP_BYT;
}

float my_abs(const float v){
  if(v >= 0.0)return v;
  else return -v;
}

const spd_ctrl get_spd_itm(const void * array_ptr, const int idx){
  spd_ctrl result;
  result.spd = pgm_read_float( array_ptr + sizeof(spd_ctrl)*idx);
  result.byt = pgm_read_byte( array_ptr + sizeof(spd_ctrl)*idx + sizeof(float));
  return result;
}

const char mask_d = B11111100;
const char mask_b = B00000011;

void wrt(byte value)
{
  byte d = ( value << 2 ) & mask_d; // we need to shift value left twice for D
  byte b = ( value >> 6 ) & mask_b; // and shift right 6 times for B

  PORTB = b; // output B first
  PORTD = d; // D second
  dac_value = value;

  if(value > STOP_BYT) dir = 1;
  else
  if(value < STOP_BYT) dir = -1;
  else{
    dir = 0;
    current_target_speed = -1;
    dist_acc = 0;
  }
}


const spd_ctrl find_speed( const spd_ctrl * arr, float val )
{
  spd_ctrl result;
  result.spd = -1; // negative search result

  int middle = -1;
  int first = 0;
  int last = SPD_SZ - 1;
  while( first <= last )
  {
    middle = (first + last)/2;
    result = get_spd_itm(arr, middle);
    if ( result.spd == val ) break;
    if ( result.spd < val ) first = middle+1;    
    else last = middle;
    if ( first == last ) break;
  }
  return result;
}

// (C) https://arachnoid.com/polysolve/
float calc_spd_down(const unsigned int a0v){
  if(a0v >= 125 && a0v <= 370){ // measured interpolation interval - see xsl
    float x = a0v;
    return 1.3762263750998270e+002 - 8.5108736772488736e-001 * x + 1.8224621955865404e-003 * (x*x) -1.3716224613745268e-006 * (x*x*x);
  }
  else return 0.0;
}

float calc_spd_up(const unsigned int a0v){
  if(a0v >= 635 && a0v <= 875){ // measured interpolation interval - see xsl
    float x = a0v;
    return -3.9222084180623028e+002 + 1.9719667508699341e+000 * x - 3.3712554941848546e-003 * (x*x) + 1.9572386600605156e-006 * (x*x*x);
  }
  else return 0.0;
}

void blink_value(const int val){
  delay(1000);
  for(int i=1; i<= val; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

float mymap(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


bool array_cmp(const char *a, const char *b){
   for (int i = 0; i < 16; i++){
     if (a[i] != b[i]) return false;
     if(a[i]==0 or b[i]==0)break;
   }
   return true;
}

//--------------Mavlink stuff (C) https://github.com/flrs/HampelFilter

void mav_send_msg(uint8_t buf[], const long len){
#ifdef SOFT_SERIAL_DEBUGGING
  pxSerial.write(buf, len);
#else
  Serial.write(buf, len);
#endif
}

// request parameter's values via mavlink.
void mav_req_params(){

  /* The default UART header for your MCU */ 
//  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
//  int compid = 158;                ///< The component sending the message
//  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
//  uint8_t system_type = MAV_TYPE_GENERIC;
//  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
//  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
//  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
//  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
//  mavlink_message_t msg;
//  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
//  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
//  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();

  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;

//    mav_send_msg(buf, len);

    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
#ifdef SOFT_SERIAL_DEBUGGING
      Serial.println("Streams requested!");
#endif
      if(!tilt_speed_set){
        Mav_Request_Data(param_speed_id);
      }else{
        if(!tilt_smoth_set){
          Mav_Request_Data(param_smoth_id);
        }
      }
      num_hbs_pasados=0;
      try_counter++;
#ifdef SOFT_SERIAL_DEBUGGING
  Serial.print("try_counter:");
  Serial.println(try_counter);
#endif
    }
    LED_STATE = !LED_STATE;
    digitalWrite(LED_BUILTIN, LED_STATE);
  }
  // Check reception buffer
  comm_receive();
}

void Mav_Request_Data(const char* param_name)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
//  const int  maxStreams = 2;
//  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA2};
//  const uint16_t MAVRates[maxStreams] = {0x02,0x05};

    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    
/*  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    mavlink_msg_param_request_read_pack(2, 200, &msg, 1, 0, "SERIAL1_PROTOCOL", -1)
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
#ifdef SOFT_SERIAL_DEBUGGING
    pxSerial.write(buf,len);
#else
    Serial.write(buf, len);
#endif
  }*/

// request parameter
    mavlink_msg_param_request_read_pack(2, 200, &msg, 1, 0, param_name, -1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mav_send_msg(buf, len);
}


void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;
 
  // Echo for manual debugging
//#ifdef SOFT_SERIAL_DEBUGGING
// Serial.println("---Start---");
//#endif

#ifdef SOFT_SERIAL_DEBUGGING
  while(pxSerial.available()>0) {
    uint8_t c = pxSerial.read();
#else
  while(Serial.available()>0) {
    uint8_t c = Serial.read();
#endif

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
#ifdef SOFT_SERIAL_DEBUGGING
            Serial.println("PX HB");
#endif
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
#ifdef SOFT_SERIAL_DEBUGGING
            Serial.print("PX SYS STATUS: ");
            Serial.print("[Bat (V): ");
            Serial.print(sys_status.voltage_battery);
            Serial.print("], [Bat (A): ");
            Serial.print(sys_status.current_battery);
            Serial.print("], [Comms loss (%): ");
            Serial.print(sys_status.drop_rate_comm);
            Serial.println("]");
#endif
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
#ifdef SOFT_SERIAL_DEBUGGING
            Serial.println("PX PARAM_VALUE");
            Serial.println(param_value.param_value);
            Serial.println(param_value.param_count);
            Serial.println(param_value.param_index);
            Serial.println(param_value.param_id);
            Serial.println(param_value.param_type);
            Serial.println("------ Fin -------");
#endif
            if( array_cmp(param_speed_id, param_value.param_id) ){
              tilt_speed = param_value.param_value;
              if(tilt_speed > 5)tilt_speed = 5;
              if(tilt_speed < 1)tilt_speed = 1;
              tilt_speed_set = true;
#ifdef SOFT_SERIAL_DEBUGGING
            Serial.println("tild_speed set!");
#endif
            }
            if( array_cmp(param_smoth_id, param_value.param_id) ){
              tilt_smoth = param_value.param_value;
              if(tilt_smoth > 5)tilt_smoth = 5;
              if(tilt_smoth < 1)tilt_smoth = 1;
              tilt_smoth_set = true;
#ifdef SOFT_SERIAL_DEBUGGING
            Serial.println("tild_smoth set!");
#endif
            }

          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
#ifdef SOFT_SERIAL_DEBUGGING
            //mySerial.println("PX RAW IMU");
            //mySerial.println(raw_imu.xacc);
#endif
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
#ifdef SOFT_SERIAL_DEBUGGING
            //mySerial.println("PX ATTITUDE");
            //mySerial.println(attitude.roll);
//            if(attitude.roll>1) leds_modo = 0;
//            else if(attitude.roll<-1) leds_modo = 2;
//            else leds_modo=1;
#endif
          }
          break;

       default:
#ifdef SOFT_SERIAL_DEBUGGING
          Serial.print("--- Other: ");
          Serial.print("[ID: ");
          Serial.print(msg.msgid);
          Serial.print("], [seq: ");
          Serial.print(msg.seq);
          Serial.println("]");
#endif
          break;
      }
    }
  }
}
