#include <CapacitiveSensor.h>
#include <Key.h>
#include <Keypad.h>

// source code for 3 alternative choice task
// you have to define all configurations according to your setup
// before get started, you must have to calibrate lickometer and motor extension length
// for lickometer calibration, set 'print_lickometer_raw' true and set status -1. Then, see each spout's value, determine float central_threshold[6] & lateral_threshold[2].

// made by Eunju Sung

//////////////// pinmap configurations ////////////////////
// motors (output)
const int lateral1A = 22;
const int lateral1B = 23;
const int lateral2A = 24;
const int lateral2B = 25;
const int centralA = 26;
const int centralB = 27;
const int encoderPinA = 2;
const int encoderPinB = 3;
const int motorDirPin = 8;  // L298 Input 1
const int motorPWMPin = 9;  // L298 Input 2
const int central_dA = 6;
const int central_dB = 7;


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
const int lickL1A = 32;
const int lickL1B = 33;
const int lickL2A = 36;
const int lickL2B = 37;
const int lick1A = 40;
const int lick1B = 41;
const int lick2A = 30;
const int lick2B = 31;
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
CapacitiveSensor lateral_lickometers[2] = { lickL1, lickL2 };
CapacitiveSensor central_lickometers[6] = { lick1, lick2, lick3, lick4, lick5, lick6 };
//////////////////////////////////////////////////////


//////////////// output flags  ////////////////////
int status = 0;                     // -1 for lickometer calibration, 0 for training
bool print_lickometer_raw = false;  // for lickometer calibration ## don't set it false. lickometers go wrong..
//////////////////////////////////////////////////////////////


//////////////// training configurations ////////////////////
// spout setting
const int central_spout = 2;        // index from zero
const int central_spout_pin = 18;   // index from zero
const int central_reward = 1;       // index from zero. spout for central decision
const int central_reward_pin = 17;  // see above pinmap
bool lateral_enabled[2] = { true, true };
bool central_enabled[6] = { false, true, false, false, false, false };
const int lick_refractory_tick = 2;  // cannot detect new lick for {lick_refractory_tick} ticks (sometimes capsense report single touch as two touch...)
const int capsensor_sensitivity = 40;
float central_threshold[6] = { 60, 70, 60, 70, 50, 50 };
float lateral_threshold[2] = { 80, 100 };
const float threshold_default = 10000000;

// trial schedule (unit: milli sec)
const int trial = 500;  // number of trial in one session
const float ready2go = 5 * 1e3;
const float ITI = 6 * 1e3;
const float timeout = 6 * 1e3;
const float valveopen = 0.1 * 1e3;
const float valve2valve = 0.2 * 1e3;
const int samplingnum = 1;  // give delivery 'samplingnum' times, time gap between each delivery is 'valve2valve'
const int rewardnum = 1;    // give reward 'rewardnum' times, time gap between each delivery is 'valve2valve'
const float time4decision = 2 * 1e3;
const float wait4nolick = 1 * 1e3;
const float valve_offset = 0.5 * 1e3;     // time for delay between spout extrude and sampling liquid delivery
const float sampling_offset = 0.3 * 1e3;  // time for delay between spout extrude and sampling liquid delivery
const float central_ex = 1.5 * 1e3;       // should be measured according to the distance you want to extrude
const float central_in = 1.6 * 1e3;       // should be measured according to the distance you want to extrude
const float central_d_ex = 1.5 * 1e3;     // should be measured according to the distance you want to extrude
const float central_d_in = 1.6 * 1e3;     // should be measured according to the distance you want to extrude
const float lateral_ex = 1.48 * 1e3;      // should be measured according to the distance you want to===================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================== extrude
const float lateral_in = 1.6 * 1e3;       // should be measured according to the distance you want to===================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================== extrude
int bottle_select_step = 160;             // for 18 degree. M1, M2, M3 HiGH (1/16)
int stepmotor_offset = 40;                // step motor has little offset if the direciton change. should be measured carefully.
const float start2central_in = 0 * 1e3;
//////////////////////////////////////////////////////

// 1. start, wait for start2central_in
// 2. central motor start to go forward, motor on for central_ex
// 3. gives water when a lick is detected
// * what if the mouse do not lick central spout? -> considered as "no sampling", trial ends, new trial starts.
// 4. central motor start to go back, motor on for central_in
// 5. lateral motors and central decision motor start to go forward, motor on for lateral_ex and central_d_ex each
// 6. give water when a lick is detected. valve open for valveopen
// * to prevent bias, water is not given for dominant choice (>50% choice or more than 4 consecutive choice)
// * if the mouse does not stick to one spout within a trial, lateral motors go back immediately.
// 7. all motors go back, motor on for lateral_in and central_d_in each


//////// trial parameters (renewed every trial) ////////
unsigned long status_start_time = 0;
unsigned long trial_start_time = 0;
unsigned long now_time = 0;
int trial_count = 1;
int central_count;
int lateral1_count;
int lateral2_count;
int no_decision_count;
int no_sampling_count;
int switch_count;
int sampling_count = 0;
int reward_count = 0;
int decision;
int decision_prev = -1;
unsigned long time_consumed = 0;  // time consumed for moving step motor (spout selection)
bool go_delivery = false;
struct Result {
  bool lickstarted = false;
  bool nowlick = false;
  long lickthres = threshold_default;
  int tick2lick = -1;
};

Result central_lick_status[6];
Result lateral_lick_status[2];
long central_vals_now[6] = { 0 };
long lateral_vals_now[2] = { 0 };
long central_vals_prev[6] = { 0 };
long lateral_vals_prev[2] = { 0 };
int central_lick_cnt[6] = { 0 };
int lateral_lick_cnt[2] = { 0 };
float central_prev_lick_started[6] = { 0 };
float lateral_prev_lick_started[2] = { 0 };
//////////////////////////////////////////////////////



////////////  define utility functions  //////////////
void delay_sec(int delay_millisec) {
  delay(delay_millisec * 1000);
}

void trial_printer(int trial_count, unsigned long now_time) {
  Serial.println("");
  Serial.println(String("//////////     Trial #" + String(trial_count) + " started       //////////"));
}

void decision_printer(int lateral_choice, int answer, char outcome) {
  Serial.println(String("decision: " + String(lateral_choice) + " answer: " + String(answer) + ". outcome: " + String(outcome)));
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
    Serial.print("  time: ");
    Serial.print(millis() - trial_start_time);
    Serial.print("  lick2lick: ");
    Serial.println(now_time - prev_lick_started);
    prev_lick_started = now_time;
  }
}

//////////////////////////////////////////////////////


void setup() {
  // pin I/O
  pinMode(lateral1A, OUTPUT);
  pinMode(lateral2A, OUTPUT);
  pinMode(lateral1B, OUTPUT);
  pinMode(lateral2B, OUTPUT);
  pinMode(centralA, OUTPUT);
  pinMode(centralB, OUTPUT);
  pinMode(central_dA, OUTPUT);
  pinMode(central_dB, OUTPUT);

  pinMode(lateral_valve1, OUTPUT);
  pinMode(lateral_valve2, OUTPUT);
  pinMode(central_valve1, OUTPUT);
  pinMode(central_valve2, OUTPUT);
  pinMode(central_valve3, OUTPUT);
  pinMode(central_valve4, OUTPUT);
  pinMode(central_valve5, OUTPUT);
  pinMode(central_valve6, OUTPUT);

  // begin serial
  Serial.begin(57600);

  // seed
  randomSeed(analogRead(0));

  // setup for lickometer (begin capsensors)
  for (int i = 0; i < 2; i++) {
    if (lateral_enabled[i]) {
      lateral_lickometers[i].set_CS_AutocaL_Millis(0xFFFFFFFF);
      Serial.print(String("\t" + String("lateral") + String(i + 1) + "\t" + String(lateral_threshold[i])));
    }
  }
  for (int i = 0; i < 6; i++) {
    if (central_enabled[i]) {
      central_lickometers[i].set_CS_AutocaL_Millis(0xFFFFFFFF);
      Serial.print(String("\t" + String("central") + String(i + 1) + "\t" + String(central_threshold[i])));
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
  for (int i = 0; i < 6; i++) {
    if (central_enabled[i]) {
      central_vals_now[i] = central_lickometers[i].capacitiveSensor(capsensor_sensitivity);
    }
  }
  for (int i = 0; i < 2; i++) {
    if (lateral_enabled[i]) {
      lateral_vals_now[i] = lateral_lickometers[i].capacitiveSensor(capsensor_sensitivity);
    }
  }

  for (int i = 0; i < 6; i++) {
    if (central_enabled[i]) {
      central_lick_status[i] = detect_lick(central_enabled[i], central_vals_now[i], central_vals_prev[i], central_lick_status[i].lickthres, central_lick_status[i].nowlick, central_lick_status[i].tick2lick, central_threshold[i]);
      print_lick(central_enabled[i], "central", i + 1, central_lick_status[i].lickstarted, central_lick_cnt[i], central_prev_lick_started[i], now_time);
    }
  }
  for (int i = 0; i < 2; i++) {
    if (lateral_enabled[i]) {
      lateral_lick_status[i] = detect_lick(lateral_enabled[i], lateral_vals_now[i], lateral_vals_prev[i], lateral_lick_status[i].lickthres, lateral_lick_status[i].nowlick, lateral_lick_status[i].tick2lick, lateral_threshold[i]);
      print_lick(lateral_enabled[i], "lateral", i + 1, lateral_lick_status[i].lickstarted, lateral_lick_cnt[i], lateral_prev_lick_started[i], now_time);
    }
  }


  if (print_lickometer_raw) {
    Serial.print("@ ");
    for (int i = 0; i < 6; i++) {
      if (central_enabled[i]) {
        Serial.print(String("\t" + String("central") + String(i + 1) + "\t"));
        Serial.print(central_vals_now[i]);
        Serial.print("\t");
      }
    }
    for (int i = 0; i < 2; i++) {
      if (lateral_enabled[i]) {
        Serial.print(String("\t" + String("lateral") + String(i + 1) + "\t"));
        Serial.print(lateral_vals_now[i]);
      }
    }
    Serial.println("");
  }

  // main body for trial
  switch (status) {

    case -1:  // not working
      {
        break;
      }

    case 0:
      {  //#0 wait 5s to start
        if (now_time - status_start_time > ready2go) {
          trial_printer(1, now_time);
          status += 1;
          status_start_time = now_time;
        }
        break;
      }

    case 1:
      {  //#1 wait for start2central_in
        if (now_time - status_start_time > start2central_in) {
          status += 1;
          status_start_time = now_time;
          trial_start_time = status_start_time;
          reward_count = 0;
          sampling_count = 0;
          time_consumed = 0;
        }
        break;
      }

    case 2:
      {  //#2 central motor extrude
        digitalWrite(centralA, HIGH);
        digitalWrite(centralB, LOW);
        if (now_time - status_start_time > central_ex) {
          Serial.println("///// sampling spout ready /////");
          digitalWrite(centralA, LOW);
          digitalWrite(centralB, LOW);
          status += 1;
          status_start_time = now_time;
        }
        break;
      }


    case 3:
      {  //#2.5 wait for lick
        if (central_lick_status[central_spout].lickstarted) {
          go_delivery = true;
          status += 1;
          status_start_time = now_time;
          break;
        }

        if (now_time - status_start_time > wait4nolick) {  // made no lick
          no_sampling_count += 1;
          go_delivery = false;
          status += 3;
          status_start_time = now_time;
        }
        break;
      }

    case 4:
      {  //#3-1 delivery sampling
        digitalWrite(central_spout_pin, HIGH);
        if (now_time - status_start_time > valveopen) {
          digitalWrite(central_spout_pin, LOW);
          status += 1;
          status_start_time = now_time;
        }
        break;
      }


    case 5:
      {  //#3.5 wait for offset
        if (now_time - status_start_time > valve_offset) {
          status += 1;
          status_start_time = now_time;
        }
        break;
      }

    case 6:
      {  //#4 intrude central motor
        digitalWrite(centralA, LOW);
        digitalWrite(centralB, HIGH);
        if (now_time - status_start_time > central_in) {
          digitalWrite(centralA, LOW);
          digitalWrite(centralB, LOW);
          if (go_delivery) {
            status += 1;
          } else {
            status = 16;
          }
          status_start_time = now_time;
        }
        break;
      }


    case 7:
      {  //#6 two lateral motors & central reward extrude
        digitalWrite(lateral1A, HIGH);
        digitalWrite(lateral1B, LOW);
        digitalWrite(lateral2A, HIGH);
        digitalWrite(lateral2B, LOW);
        digitalWrite(central_dA, HIGH);
        digitalWrite(central_dB, LOW);

        if (central_d_ex > lateral_ex) {
          if (now_time - status_start_time > lateral_ex) {
            digitalWrite(lateral1A, LOW);
            digitalWrite(lateral1B, LOW);
            digitalWrite(lateral2A, LOW);
            digitalWrite(lateral2B, LOW);
            status += 1;
            status_start_time = now_time;
          }
          break;
        } else if (central_d_ex < lateral_ex) {
          if (now_time - status_start_time > central_d_ex) {
            digitalWrite(central_dA, LOW);
            digitalWrite(central_dB, LOW);
            status += 2;
            status_start_time = now_time;
          }
          break;
        } else {
          if (now_time - status_start_time > central_d_ex) {
            Serial.println("///// delivery spouts ready /////");
            digitalWrite(lateral1A, LOW);
            digitalWrite(lateral1B, LOW);
            digitalWrite(lateral2A, LOW);
            digitalWrite(lateral2B, LOW);
            digitalWrite(central_dA, LOW);
            digitalWrite(central_dB, LOW);
            status += 3;
            status_start_time = now_time;
          }
          break;
        }
      }

    case 8:
      {  //#6 two lateral motors & central reward extrude (case: central_duration > lateral_duration)
        digitalWrite(central_dA, HIGH);
        digitalWrite(central_dB, LOW);
        if (now_time - status_start_time > central_d_ex - lateral_ex) {
          Serial.println("///// delivery spouts ready /////");

          digitalWrite(central_dA, LOW);
          digitalWrite(central_dB, LOW);
          status += 2;
          status_start_time = now_time;
        }
        break;
      }

    case 9:
      {  //#6 two lateral motors & central reward extrude (case: lateral_duration > central_duration)
        digitalWrite(lateral1A, HIGH);
        digitalWrite(lateral1B, LOW);
        digitalWrite(lateral2A, HIGH);
        digitalWrite(lateral2B, LOW);
        if (now_time - status_start_time > lateral_in - central_d_ex) {
          Serial.println("///// delivery spouts ready /////");
          digitalWrite(lateral1A, LOW);
          digitalWrite(lateral1B, LOW);
          digitalWrite(lateral2A, LOW);
          digitalWrite(lateral2B, LOW);
          status += 1;
          status_start_time = now_time;
        }
        break;
      }

    case 10:
      {                                            //#3-1 delivery sampling
        if (lateral_lick_status[0].lickstarted) {  // made decision! (lateral 1)
          if (trial_count - no_decision_count > 10 && ((float)lateral1_count / ((float)trial_count - (float)no_decision_count) > 0.5 || (switch_count > 4 && decision_prev == 1))) {
            status += 3;
          } else {
            status += 1;
          }

          decision = 1;
          lateral1_count += 1;
          if (decision == decision_prev) {
            switch_count += 1;
          } else {
            switch_count = 1;
          }
          status_start_time = now_time;
        }


        else if (lateral_lick_status[1].lickstarted) {  // made decision! (lateral 2)
          if (trial_count - no_decision_count > 10 && ((float)lateral2_count / ((float)trial_count - (float)no_decision_count) > 0.5 || (switch_count > 4 && decision_prev == 2))) {
            status += 3;
          } else {
            status += 1;
          }
          decision = 2;
          lateral2_count += 1;
          if (decision == decision_prev) {
            switch_count += 1;
          } else {
            switch_count = 1;
          }
          status_start_time = now_time;
        }

        else if (central_lick_status[central_reward].lickstarted) {  // made decision! (lateral 2)
          if (trial_count - no_decision_count > 10 && ((float)central_count / ((float)trial_count - (float)no_decision_count) > 0.5 || (switch_count > 4 && decision_prev == 3))) {
            status += 3;
          } else {
            status += 1;
          }
          decision = 3;
          central_count += 1;
          if (decision == decision_prev) {
            switch_count += 1;
          } else {
            switch_count = 1;
          }
          status_start_time = now_time;
        }

        else if (now_time - status_start_time > wait4nolick) {  // made no decision...
          decision = 0;
          no_decision_count += 1;
          status += 3;
          status_start_time = now_time;
        }
        break;
      }

    case 11:
      {  // # 7-1.1. get reward. reward valve open.
         // valve open (only picked motor)
        if (decision == 1 && (lateral_lick_status[1].lickstarted || central_lick_status[central_reward].lickstarted)) {
          status += 2;
          status_start_time = now_time;
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
        } else if (decision == 2 && (lateral_lick_status[0].lickstarted || central_lick_status[central_reward].lickstarted)) {
          status += 2;
          status_start_time = now_time;
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
        } else if (decision == 3 && (lateral_lick_status[0].lickstarted || lateral_lick_status[1].lickstarted)) {
          status += 2;
          status_start_time = now_time;
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
        }

        if (decision == 1) {
          digitalWrite(lateral_valve1, HIGH);
        } else if (decision == 2) {
          digitalWrite(lateral_valve2, HIGH);
        } else if (decision == 3) {
          digitalWrite(central_reward_pin, HIGH);
        }

        if (now_time - status_start_time > valveopen) {
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
          status += 1;
          reward_count += 1;
          status_start_time = now_time;
        }
        break;
      }


    case 12:
      {  // # 7-1.2 time for licking
        if (decision == 1 && (lateral_lick_status[1].lickstarted || central_lick_status[central_reward].lickstarted)) {
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
          status += 1;
          status_start_time = now_time;
        } else if (decision == 2 && (lateral_lick_status[0].lickstarted || central_lick_status[central_reward].lickstarted)) {
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
          status += 1;
          status_start_time = now_time;
        } else if (decision == 3 && (lateral_lick_status[0].lickstarted || lateral_lick_status[1].lickstarted)) {
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
          status += 1;
          status_start_time = now_time;
        }

        if (now_time - status_start_time > valve_offset) {
          status_start_time = now_time;
          digitalWrite(lateral_valve1, LOW);
          digitalWrite(lateral_valve2, LOW);
          digitalWrite(central_reward_pin, LOW);
          status += 1;
          status_start_time = now_time;
        }
        break;
      }


    case 13:
      {  //#8 two lateral motors & central reward intrude
        digitalWrite(lateral_valve1, LOW);
        digitalWrite(lateral_valve2, LOW);
        digitalWrite(central_reward_pin, LOW);
        digitalWrite(lateral1A, LOW);
        digitalWrite(lateral1B, HIGH);
        digitalWrite(lateral2A, LOW);
        digitalWrite(lateral2B, HIGH);
        digitalWrite(central_dA, LOW);
        digitalWrite(central_dB, HIGH);

        if (central_d_in > lateral_in) {
          if (now_time - status_start_time > lateral_in) {
            digitalWrite(lateral1A, LOW);
            digitalWrite(lateral1B, LOW);
            digitalWrite(lateral2A, LOW);
            digitalWrite(lateral2B, LOW);
            status += 1;
            status_start_time = now_time;
          }
          break;
        } else if (central_d_in < lateral_in) {
          if (now_time - status_start_time > central_d_in) {
            digitalWrite(central_dA, LOW);
            digitalWrite(central_dB, LOW);
            status += 2;
            status_start_time = now_time;
          }
          break;
        } else {
          if (now_time - status_start_time > central_d_in) {
            digitalWrite(lateral1A, LOW);
            digitalWrite(lateral1B, LOW);
            digitalWrite(lateral2A, LOW);
            digitalWrite(lateral2B, LOW);
            digitalWrite(central_dA, LOW);
            digitalWrite(central_dB, LOW);
            status += 3;
            status_start_time = now_time;
          }
          break;
        }
      }

    case 14:
      {  //#8 two lateral motors & central reward intrude (case: central_duration > lateral_duration)
        digitalWrite(lateral_valve1, LOW);
        digitalWrite(lateral_valve2, LOW);
        digitalWrite(central_reward_pin, LOW);
        digitalWrite(centralA, LOW);
        digitalWrite(centralB, HIGH);
        if (now_time - status_start_time > central_d_in - lateral_in) {
          digitalWrite(central_dA, LOW);
          digitalWrite(central_dB, LOW);
          status += 2;
          status_start_time = now_time;
        }
        break;
      }

    case 15:
      {  //#8 two lateral motors & central reward intrude (case: lateral_duration > central_duration)
        digitalWrite(lateral1A, LOW);
        digitalWrite(lateral1B, HIGH);
        digitalWrite(lateral2A, LOW);
        digitalWrite(lateral2B, HIGH);
        if (now_time - status_start_time > lateral_in - central_d_in) {
          digitalWrite(lateral1A, LOW);
          digitalWrite(lateral1B, LOW);
          digitalWrite(lateral2A, LOW);
          digitalWrite(lateral2B, LOW);
          status += 1;
          status_start_time = now_time;
        }
        break;
      }


    case 16:
      {  // #8-1,8-2. wait for ITI
        if (now_time - status_start_time > ITI) {
          status = 1;  // go back to begin of the trial
          if (decision != 0) {
            decision_prev = decision;
          }
          for (int i = 0; i < 6; i++) {
            central_prev_lick_started[i] = now_time;
            central_lick_cnt[i] = 0;
          }
          for (int i = 0; i < 2; i++) {
            lateral_prev_lick_started[i] = now_time;
            lateral_lick_cnt[i] = 0;
          }
          Serial.println(String("trial # " + String(trial_count) + "ended. L1: " + String(lateral1_count) + ", L2: " + String(lateral2_count)) + ", C: " + String(central_count) + 
          ", no decision: " + String(no_decision_count) + ", no sampling: " + String(no_sampling_count) + ", switching count: " + String(switch_count));
          trial_count += 1;
          status_start_time = now_time;
          trial_start_time = now_time;
          trial_printer(trial_count, now_time);
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

  delay(10);
}
