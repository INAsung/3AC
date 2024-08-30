#include <CapacitiveSensor.h>

// source code for 3 alternative choice task
// you have to define all configurations according to your setup
// before get started, you must have to calibrate lickometer and motor extension length
// for lickometer calibration, set 'print_lickometer_raw' true and set status -1. Then, see each spout's value, determine float central_threshold[6] & lateral_threshold[2].

// made by Eunju Sung

//////////////// pinmap configurations ////////////////////
// motors (output)
const int lateral1A = 22 ;
const int lateral1B = 23 ;
const int lateral2A = 24 ;
const int lateral2B = 25 ;
const int centralA = 26 ;
const int centralB = 27 ;
const int steppin = 28 ;
const int dirpin = 29;

// solenoid valve control (output)
const int lateral_valve1 = 14;
const int lateral_valve2 = 15;
const int central_valve1 = 16;
const int central_valve2 = 17;
const int central_valve3 = 18;
const int central_valve4 = 19;
const int central_valve5 = 20;
const int central_valve6 = 21;

// lickometers (input)
// smaller number - resistor (red line) / bigger number - wire
const int lickL1A = 38;
const int lickL1B = 39;
const int lickL2A = 40;
const int lickL2B = 41;
const int lick1A = 42;
const int lick1B = 43;
const int lick2A = 44;
const int lick2B = 45;
const int lick3A = 46;
const int lick3B = 47;
const int lick4A = 48;
const int lick4B = 49;
const int lick5A = 50;
const int lick5B = 51;
const int lick6A = 52;
const int lick6B = 53;
CapacitiveSensor lickL1 = CapacitiveSensor(lickL1A, lickL1B);
CapacitiveSensor lickL2 = CapacitiveSensor(lickL2A, lickL2B);
CapacitiveSensor lick1 = CapacitiveSensor(lick1A, lick1B);
CapacitiveSensor lick2 = CapacitiveSensor(lick2A, lick2B);
CapacitiveSensor lick3 = CapacitiveSensor(lick3A, lick3B);
CapacitiveSensor lick4 = CapacitiveSensor(lick4A, lick4B);
CapacitiveSensor lick5 = CapacitiveSensor(lick5A, lick5B);
CapacitiveSensor lick6 = CapacitiveSensor(lick6A, lick6B);
CapacitiveSensor lateral_lickometers[2] = {lickL1, lickL2};
CapacitiveSensor central_lickometers[6] = {lick1, lick2, lick3, lick4, lick5, lick6};
//////////////////////////////////////////////////////


//////////////// output flags  ////////////////////
int status= 0; // -1 for lickometer calibration, 0 for training
bool print_lickometer_raw = true; // for lickometer calibration ## don't set it false. lickometers go wrong..
//////////////////////////////////////////////////////////////


//////////////// training configurations ////////////////////
// spout setting
const int central_reward = 3; // index from zero. spout for central decision
const int central_reward_pin = 19; // see above pinmap
bool lateral_enabled[2] = {false, false};
bool central_enabled[6] = {false, false, false, true, false, false};// do not set true for central_reward spout
const int lick_refractory_tick = 2; // cannot detect new lick for {lick_refractory_tick} ticks (sometimes capsense report single touch as two touch...)
const int capsensor_sensitivity = 40;
float central_threshold[6] = {50, 50, 80, 80, 50, 50};
float lateral_threshold[2] = {100, 50};
const float threshold_default = 10000000;

// trial schedule (unit: milli sec)
const int trial = 60; // number of trial in one session
const float ready2go = 5*1e3;
const float ITI = 15*1e3;
const float valveopen = 0.05*1e3;
const float valve2valve = 0.2*1e3;
const int samplingnum = 1; // give reward 'rewardnum' times, time gap between each delivery is 'valve2valve' 
const float sampling_offset = 0.5*1e3; // time for delay between spout extrude and sampling liquid delivery
const float central_ex = 1.5*1e3; // should be measured according to the distance you want to extrude
const float central_in = 1.6*1e3; // should be measured according to the distance you want to extrude
const float lateral_ex = 1.5*1e3; // should be measured according to the distance you want to extrude
const float lateral_in = 1.5*1e3; // should be measured according to the distance you want to extrude
int bottle_select_step = 160;// for 18 degree
const float start2central_in = 0*1e3;
///////////////////////////////////////////////////////

// 1. start, wait for start2central_in
// 2. central motor start to go forward, motor on for central_ex
// 3. give water unconditionally. valve open for valveopen
// 4. central motor goes back, motor on for central_in

//////// trial parameters (renewed every trial) ////////
unsigned long status_start_time = 0;
unsigned long trial_start_time = 0;
unsigned long now_time = 0;
int trial_count = 1;
int sampling_count = 0;
unsigned long time_consumed = 0; // time consumed for moving step motor (spout selection) 

struct Result{
  bool lickstarted = false;
  bool nowlick = false;
  long lickthres = threshold_default;
  int tick2lick = -1;
};

Result central_lick_status[6];
Result lateral_lick_status[2];
long central_vals_now[6] = {0};
long lateral_vals_now[2] = {0};
long central_vals_prev[6] = {0};
long lateral_vals_prev[2] = {0};
int central_lick_cnt[6] = {0};
int lateral_lick_cnt[2] = {0};
float central_prev_lick_started[6] = {0};
float lateral_prev_lick_started[2] = {0};
//////////////////////////////////////////////////////



////////////  define utility functions  //////////////
void delay_sec(int delay_millisec){
  delay(delay_millisec * 1000);
}

void trial_printer(int trial_count, unsigned long now_time){
  Serial.println("");
  Serial.println("////////////////////////////////////////////////////////////////////////////////////");
  Serial.println(String("//////////     Trial #" + String(trial_count) + " started.  //////////" ));
  Serial.println("////////////////////////////////////////////////////////////////////////////////////");
}

void decision_printer(int lateral_choice, int answer, char outcome){
  Serial.println(String("decision: " + String(lateral_choice) + " answer: " + String(answer) + ". outcome: " + String(outcome) ));
}

Result detect_lick(bool flag, long now_val, long prev_val, long lickthres, bool nowlick, int tick2lick, int thres) {
  Result result;

  // clock tick from start of the lick
  // tick2lick=-1 : ready to lick
  if (tick2lick > -1 && (tick2lick < lick_refractory_tick || tick2lick == lick_refractory_tick)) {
    tick2lick += 1;
  } else if (tick2lick > lick_refractory_tick) {
    tick2lick = -1;
  }
  result.tick2lick = tick2lick;

  // detect lick
  // lick started
  if (tick2lick == -1 && nowlick == false && now_val - prev_val > thres) {
    result.nowlick = true;
    result.lickthres = now_val;
    result.lickstarted = true;
    result.tick2lick = 0;
  }
  // prolonged lick
  else if (nowlick == true && now_val > lickthres) {
    result.nowlick = true;
    result.lickstarted = false;
    result.lickthres = lickthres;
  } else if (nowlick == true && (now_val < lickthres || now_val == lickthres)) {
    result.nowlick = false;
    result.lickstarted = false;
    result.lickthres = threshold_default;
  }
  return result;
}

void print_lick(bool flag, char* prefix, int idx, bool lickstarted, int& cnt, float& prev_lick_started, unsigned long now_time) {
  String name;
  name = String(String(prefix) + String(idx));
  if (flag && lickstarted) {
    cnt += 1;
    Serial.print(name);
    Serial.print(" lick detected!     count: ");
    Serial.print(cnt);
    Serial.print("     lick2lick: ");
    Serial.println(now_time - prev_lick_started);
    prev_lick_started = now_time;
  }
}

//////////////////////////////////////////////////////


void setup() {
// pin I/O 
pinMode ( lateral1A, OUTPUT ) ;
pinMode ( lateral2A, OUTPUT ) ;
pinMode ( lateral1B, OUTPUT ) ;
pinMode ( lateral2B, OUTPUT ) ;
pinMode ( centralA, OUTPUT ) ;
pinMode ( centralB, OUTPUT ) ;
pinMode ( steppin, OUTPUT ) ;
pinMode ( dirpin, OUTPUT ) ;

pinMode ( lateral_valve1, OUTPUT );
pinMode ( lateral_valve2, OUTPUT );
pinMode ( central_valve1, OUTPUT ) ;
pinMode ( central_valve2, OUTPUT ) ;
pinMode ( central_valve3, OUTPUT ) ;
pinMode ( central_valve4, OUTPUT ) ;
pinMode ( central_valve5, OUTPUT ) ;
pinMode ( central_valve6, OUTPUT ) ;

// begin serial
Serial.begin(57600);

// seed
randomSeed(analogRead(0));

// setup for lickometer (begin capsensors)
  for (int i=0; i<2; i++){
    if (lateral_enabled[i]){
      lateral_lickometers[i].set_CS_AutocaL_Millis(0xFFFFFFFF);
      Serial.print(String("\t" + String("lateral") + String(i+1) + "\t" + String(lateral_threshold[i])));
    }
  }
  for (int i=0; i<6; i++){
    if (central_enabled[i]){
      central_lickometers[i].set_CS_AutocaL_Millis(0xFFFFFFFF);
      Serial.print(String("\t" + String("central") + String(i+1) + "\t" + String(central_threshold[i])));
    }
  }
  Serial.println("");
  Serial.println("/////////////////////////////////////////////////////////");
  Serial.println("/////////          session started              /////////");
  Serial.println("/////////////////////////////////////////////////////////");
}

void loop() {

  // update time
  now_time = millis();
  
  // update lickometers and detect licks
  for (int i=0; i<6; i++){
    if (central_enabled[i]){
      central_vals_now[i] = central_lickometers[i].capacitiveSensor(capsensor_sensitivity);
    }
  }
  for (int i=0; i<2; i++){
    if (lateral_enabled[i]){
      lateral_vals_now[i] = lateral_lickometers[i].capacitiveSensor(capsensor_sensitivity);
    }
  }

  for (int i=0; i<6; i++){
    if (central_enabled[i]){
      central_lick_status[i] = detect_lick(central_enabled[i], central_vals_now[i], central_vals_prev[i], central_lick_status[i].lickthres, central_lick_status[i].nowlick, central_lick_status[i].tick2lick, central_threshold[i]);
      print_lick(central_enabled[i], "central", i+1, central_lick_status[i].lickstarted, central_lick_cnt[i], central_prev_lick_started[i], now_time);
    }
  }
  for (int i=0; i<2; i++){
    if (lateral_enabled[i]){
      lateral_lick_status[i] = detect_lick(lateral_enabled[i], lateral_vals_now[i], lateral_vals_prev[i], lateral_lick_status[i].lickthres, lateral_lick_status[i].nowlick, lateral_lick_status[i].tick2lick, lateral_threshold[i]);
      print_lick(lateral_enabled[i], "lateral", i+1, lateral_lick_status[i].lickstarted, lateral_lick_cnt[i], lateral_prev_lick_started[i], now_time);      
    }
  }


  if (print_lickometer_raw) {
    Serial.print("@ ");
    for (int i=0; i<6; i++){
      if (central_enabled[i]){
        Serial.print(String("\t" + String("central") + String(i+1) + "\t"));
        Serial.print(central_vals_now[i]);
        Serial.print("\t");
      }
    }
    for (int i=0; i<2; i++){
      if (lateral_enabled[i]){
        Serial.print(String("\t" + String("lateral") + String(i+1) + "\t"));
        Serial.print(lateral_vals_now[i]);    
      }
    }
    Serial.println("");
  }

  // main body for trial
  switch(status){

    case -1: // not working
      {
        break;
      }

    case 0: { //#0 wait 5s to start
      if(now_time - status_start_time > ready2go){
        Serial.println("stage 1 now start");
        status += 1;
        status_start_time = now_time;
      }
      break;
    }

    case 1: { //#1 wait for start2central_in
      if(now_time - status_start_time > start2central_in){
          trial_printer(trial_count,now_time);

        status += 1;
        status_start_time = now_time;
        sampling_count = 0;
        time_consumed = 0;
      }
      break;
    }

    case 2: { //#2 central motor extrude
      digitalWrite(centralA, HIGH);
      digitalWrite(centralB, LOW);
      if(now_time - status_start_time > central_ex){
        digitalWrite(centralA, LOW);
        digitalWrite(centralB, LOW);
        status += 1;
        status_start_time = now_time;
      }
      break;
    }


    case 3: { //#2.5 wait for offset
      if(now_time - status_start_time > sampling_offset){
        status += 1;
        status_start_time = now_time;
      }
      break;    
    }

    case 4: { //#3-1 delivery sampling

      digitalWrite(central_reward_pin, HIGH);
    
      if(now_time - status_start_time > valveopen){
        digitalWrite(central_reward_pin, LOW);
        sampling_count += 1;
        status += 1;
        status_start_time = now_time;
      }
    break;
    }

    case 5: { //#3-2 delivery to delivery offset
      if(now_time - status_start_time > valve2valve){
        if (sampling_count < samplingnum){
          status -= 1;
          status_start_time = now_time;
          } 
        else {
          status += 1;
          status_start_time = now_time;
          }
      }
      break;
    }

    case 6: { //#3.5 wait for offset
      if(now_time - status_start_time > sampling_offset){
        status += 1;
        status_start_time = now_time;
      }
      break;    
    }

    case 7: {//#4 intrude central motor
      digitalWrite(centralA, LOW);
      digitalWrite(centralB, HIGH);
      if(now_time - status_start_time > central_in){
        Serial.println(now_time - status_start_time);
        digitalWrite(centralA, LOW);
        digitalWrite(centralB, LOW);
        status += 1;
        status_start_time = now_time;
      }
      break;
    }

    case 8: { 
      if(now_time - status_start_time > ITI - time_consumed){
        status = 1; // go back to begin of the trial
        status_start_time = now_time;
        trial_count += 1;
        for (int i = 0; i < 6; i++){
          central_prev_lick_started[i] = now_time;
          central_lick_cnt[i] = 0;
        }
        for (int i = 0; i < 2; i++){
          lateral_prev_lick_started[i] = now_time;
          lateral_lick_cnt[i] = 0;
        }   

        Serial.println("###################################################");        
        Serial.println(String("trial # "+ String(trial_count) + "ended."));        
        Serial.println("###################################################");        
        trial_start_time = now_time;     
      }
    }
  }

  // Update previous values for the next iteration
  for (int i = 0; i < 6; i++) {
    central_vals_prev[i] = central_vals_now[i];
  }
  for (int i = 0; i < 2; i++) {
    lateral_vals_prev[i] = lateral_vals_now[i];
  }
}
