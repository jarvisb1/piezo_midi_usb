#include "MIDIUSB.h"
#include "PitchToNote.h"

//#define ENABLE_SERIAL_OUTPUT (1) //Uncomment this entire line to write debug info to serial
//#define DISABLE_MIDI_OUTPUT (1) //Uncomment this entire line to disable MIDI output (useful in debugging)

///////////////////
// CONFIGURATION //
///////////////////
//NOTE: This expects the sensors to be wired up like the knock sensor in https://www.arduino.cc/en/Tutorial/Knock
#define NUM_SENSORS (1)                           // Specify the number of sensors here. If you change this, you need to make sure the lists below have a value for every sensor (see examples in each comment)
const byte sensor_pins[NUM_SENSORS] = {A0};       // Specify a list of pins that the sensors are on, for example if you're using A0, A2, and A5: {A0, A2, A5};
const int sensor_thresholds[NUM_SENSORS] = {150}; // Specify a list of thresholds that each sensor will use to determine when it's been tapped, for example if you want 50 for the 2nd sensor but 20 for the others: {20, 50, 20};
const byte midi_channels[NUM_SENSORS] = {0};      // Specify a list of MIDI channel that each sensor will output to. For example if you want the first two to output to channel 0 and the 3rd to output to channel 1: {0, 0, 1};
const byte sensor_notes[NUM_SENSORS] = {pitchC3}; // Specify a list of pitches that each sensor should play. For example if you want the first sensor to play C3 and the 2nd and 3rd sensors to play G4: {pitchC4, pitchG4, pitchG4};
//Additionall pitch examples for convenience: pitchD3, pitchE3, pitchF3, pitchG3, pitchA3, pitchB3, pitchC4, pitchD4, pitchE4, pitchF4, pitchG4}; //C major scale

// To change the duration that each note is played, search for "To play notes for a different duration" in the code below and change the value in the comparison statement.

#define LOOP_SLEEP_MS (100) // Milliseconds to sleep/delay at the end of each loop iteration.

//Minimum and maximum values that could be passed into the note mapping function.
//Note that you may need to adjust these to match real-world values
#define VELOCITY_MIN (25) // The lowest possible velocity. Adjust this higher if light taps are not playing loud enough.
#define VELOCITY_MAX (100) // The highest possible velocity. You probably don't want to change this unless things are playing too strong.
//////////////////////////
// END OF CONFIGURATION //
//////////////////////////

#define ANALOG_IN_MIN (1)
#define ANALOG_IN_MAX (250) //The ADC can go up to 1024. But looking at the values, it never really goes higher than appx. 250.
#define DEFAULT_PITCH (pitchC3)
bool is_playing[NUM_SENSORS]; //This gets set to true while a note is playing
unsigned long note_on_time[NUM_SENSORS];
int curr_val[NUM_SENSORS];
int last_val[NUM_SENSORS];
byte pitches[NUM_SENSORS];
byte velocities[NUM_SENSORS];

void setup() {
#ifdef ENABLE_SERIAL_OUTPUT
  Serial.begin(115200);
#endif
  //Initialize the default values
  for (int sensor_num; sensor_num < NUM_SENSORS; sensor_num++) {
    pinMode(sensor_pins[sensor_num], INPUT);
    pitches[sensor_num] = DEFAULT_PITCH;
    curr_val[sensor_num] = -1;
    last_val[sensor_num] = -1;
    note_on_time[sensor_num] = 0;
    is_playing[sensor_num] = false;
  }

}

int read_value(int sensor_num) {
  return analogRead(sensor_pins[sensor_num]);
}

byte get_velocity_from_val(int val) {
  //Map the value to the range of velocities available
  return map(val, ANALOG_IN_MIN, ANALOG_IN_MAX, VELOCITY_MIN, VELOCITY_MAX);
}

void MIDINoteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
#ifndef DISABLE_MIDI_OUTPUT
  MidiUSB.sendMIDI(noteOn);
#endif
}

void MIDINoteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
#ifndef DISABLE_MIDI_OUTPUT
  MidiUSB.sendMIDI(noteOff);
#endif
}

void loop() {
  unsigned long curr_time = millis();
  for (int sensor_num = 0; sensor_num < NUM_SENSORS; sensor_num++) {
    //Read a new value and stash the previous one 
    last_val[sensor_num] = curr_val[sensor_num];
    curr_val[sensor_num] = read_value(sensor_num);

    //The curr and last values were initialized to -1, so don't do anything until these have been overwritten with real data
    if (last_val[sensor_num] == -1 || curr_val[sensor_num] == -1) {
      continue;
    }
#ifdef ENABLE_SERIAL_OUTPUT
    Serial.println("-------------------------------------------------------------------------------");
    Serial.print("["); Serial.print(curr_time); Serial.print("] Sensor "); Serial.println(sensor_num);
    Serial.print("---- Value: "); Serial.println(curr_val[sensor_num]);
    Serial.print("---- Currently playing?: "); Serial.println(is_playing[sensor_num]);
#endif
    if (is_playing[sensor_num]) {
      //If a note is playing currently, and there's still time left to play, do nothing for now
      //To play notes for a different duration, change the value on the right side of the "less than" below.
      //For some reason using a variable for that value was causing problems so that's why it's hard-coded like that. TODO: Figure out why that is.
      if ((curr_time - note_on_time[sensor_num]) < 200) {
#ifdef ENABLE_SERIAL_OUTPUT
        Serial.print("---- Curr time: "); Serial.println(curr_time);
        Serial.print("---- Note on time: "); Serial.println(note_on_time[sensor_num]);
#endif
        continue;
      }

      //Get here if a note is playing and its time has elapsed, so turn it off
      MIDINoteOff(midi_channels[sensor_num],
                  pitches[sensor_num],
                  velocities[sensor_num]);
      is_playing[sensor_num] = false;
#ifdef ENABLE_SERIAL_OUTPUT
    Serial.println("----Note on time elapsed. Note off.");
#endif
    } else {
#ifdef ENABLE_SERIAL_OUTPUT
      Serial.print("---- Value read: "); Serial.println(curr_val[sensor_num]);
#endif
      if (curr_val[sensor_num] > sensor_thresholds[sensor_num]) {
        pitches[sensor_num] = sensor_notes[sensor_num];
        velocities[sensor_num] = get_velocity_from_val(curr_val[sensor_num]);
#ifdef ENABLE_SERIAL_OUTPUT
        Serial.print("---- Threshold breached. Playing pitch: "); Serial.print(pitches[sensor_num]); Serial.print(", velocity: "); Serial.println(velocities[sensor_num]);
#endif
        MIDINoteOn(midi_channels[sensor_num],
                   pitches[sensor_num],
                   velocities[sensor_num]);
        note_on_time[sensor_num] = curr_time;
        is_playing[sensor_num] = true;
      }
    } 
  }
  delay(LOOP_SLEEP_MS);
}
