#include "MIDIUSB.h"
#include "PitchToNote.h"

#define NUM_SENSORS (1)
const byte sensor_pins[NUM_SENSORS] = {A1};
const int sensor_thresholds[NUM_SENSORS] = {20};
bool is_playing[NUM_SENSORS]; //This gets set to true while a note is playing
unsigned long note_on_time[NUM_SENSORS];
int note_duration_ms[NUM_SENSORS] = {200}; //When a note is triggered, how long should it play for (in ms)
int curr_val[NUM_SENSORS];
int last_val[NUM_SENSORS];

#define LOOP_SLEEP_MS (100) // Milliseconds to sleep/delay at the end of each loop iteration.

//Minimum and maximum values that could be passed into the note mapping function.
//Note that you may need to adjust these to match real-world values
#define ANALOG_IN_MIN (1)
#define ANALOG_IN_MAX (1024)
#define VELOCITY_MIN (1)
#define VELOCITY_MAX (100)

#define DEFAULT_PITCH (pitchC3)
byte pitches[NUM_SENSORS];
byte velocities[NUM_SENSORS];
const byte midi_channels[NUM_SENSORS] = {0}; //MIDI channel to output on per sensor

//The note each sensor will trigger
const byte sensor_notes[NUM_SENSORS] = {pitchC3};//, pitchD3, pitchE3, pitchF3, pitchG3, pitchA3, pitchB3, pitchC4, pitchD4, pitchE4, pitchF4, pitchG4}; //C major scale

void setup() {
  //Initialize the default values
  for (int sensor_num; sensor_num < NUM_SENSORS; sensor_num++) {
    pitches[sensor_num] = DEFAULT_PITCH;
    curr_val[sensor_num] = -1;
    last_val[sensor_num] = -1;
    note_on_time[sensor_num] = 0;
    is_playing[sensor_num] = false;
  }

}

int read_value(int sensor_num) {
  int val = 0;
  val = analogRead(sensor_pins[sensor_num]);
  return val;
}

byte get_velocity_from_val(int val) {
  //Map the value to the range of velocities available
  return map(val, ANALOG_IN_MIN, ANALOG_IN_MAX, VELOCITY_MIN, VELOCITY_MAX);
}

void MIDINoteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void MIDINoteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
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

    if (is_playing[sensor_num]) {
      //If a note is playing currently, and there's still time left to play, do nothing for now
      if ((curr_time - note_on_time[sensor_num]) < note_duration_ms[NUM_SENSORS]) {
        continue;
      }

      //Get here if a note is playing and its time has elapsed, so turn it off
      MIDINoteOff(midi_channels[sensor_num],
                  pitches[sensor_num],
                  velocities[sensor_num]);
      is_playing[sensor_num] = false;
    }
    //Get here if this sensor is not playing anything.

    //If the difference between two consecutive reads breaches the threshold, play a note.
    //There are different ways of determining whether to play something, so if this is behaving weirdly, look into using another approach.
    int value_diff = abs(last_val[sensor_num] - curr_val[sensor_num]);    
    if (value_diff > sensor_thresholds[sensor_num]) {
      pitches[sensor_num] = sensor_notes[sensor_num];
      velocities[sensor_num] = get_velocity_from_val(value_diff);
      MIDINoteOn(midi_channels[sensor_num],
                 pitches[sensor_num],
                 velocities[sensor_num]);
      is_playing[sensor_num] = true;
    } 
  }
  delay(LOOP_SLEEP_MS);
}
