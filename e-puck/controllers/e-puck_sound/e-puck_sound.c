//------------------------------------------------------------------------------------------
// File:         e-puck_sound.c
// Description:  Sound plugin demo using two e-puck robots.
//               One e-puck emits sound samples on its speaker, the other e-puck listens to
//               one of its 3 microphones and tries to detect these sound samples.
//               Note that the same controller code (this file) is used for both robots.
//               The particular "microphone" or "speaker" role is assigned according to what
//               is specified in the "controllerArgs" field of the DifferentialWheels robot.
// Author:       Yvan Bourquin, Swarm-intelligent Systems Group (SWIS), EPFL, Switzerland
// Created:      26-Oct-2007
//------------------------------------------------------------------------------------------

#include <webots/robot.h>
#include <webots/speaker.h>
#include <webots/microphone.h>
#include <webots/distance_sensor.h>
#include <webots/differential_wheels.h>
#include <string.h>
#include <stdio.h>

#define TIME_STEP 32
#define NUM_SENSORS 8
#define RANGE (1024 / 2)

enum { UNKNOWN, SPEAKER, MICROPHONE };

static const double EPUCK_MATRIX[8][2] = {
  { 150, -35 }, { 100, -15 }, {  80, -10 }, { -10, -10 },
  { -10, -10 }, { -10,  80 }, { -30, 100 }, { -20, 150 }
};

// sound sample to emit
static const short SAMPLE[] = {
  -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128,
  127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127
};

int main(int argc, const char *argv[]) {

  WbDeviceTag speaker, microphone;
  WbDeviceTag sensors[NUM_SENSORS];
  int role = UNKNOWN;
  int prev_audible = -1;
  int i, j; // for loops

  // initialize webots
  wb_robot_init();

  // determine role
  if (argc >= 2) {
    if (! strcmp(argv[1], "speaker"))
      role = SPEAKER;
    else if (! strcmp(argv[1], "microphone"))
      role = MICROPHONE;
  }

  // use speaker or microphone according to specified role
  if (role == SPEAKER)
    speaker = wb_robot_get_device("speaker");
  else if (role == MICROPHONE) {
    // we only use "mic0" in this demo
    microphone = wb_robot_get_device("mic0");
    wb_microphone_enable(microphone, TIME_STEP / 2);
  }

  // find distance sensors
  for (i = 0; i < NUM_SENSORS; i++) {
    char sensor_name[64];
    sprintf(sensor_name, "ps%d", i);
    sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }

  // main loop
  for (;;) {
    if (role == SPEAKER) {
       wb_speaker_emit_sample(speaker, SAMPLE, sizeof(SAMPLE));
    }
    else if (role == MICROPHONE) {
      const short *rec_buffer = (const short *)wb_microphone_get_sample_data(microphone);
      int numSamples =  wb_microphone_get_sample_size(microphone) / sizeof(SAMPLE[0]);
      if (rec_buffer) {
        int audible = 0;
        for (i = 0; i < numSamples; i++) {
          // warning this demo assumes no noise and therefore depends on the sound plugin configuration !
          if (rec_buffer[i] != 0)
            audible = 1;
        }

        if (audible != prev_audible) {
          printf(audible ? "I hear you now !\n" : "I can't hear you !\n");
          prev_audible = audible;
        }
      }
      else
        printf("received: nothing this time ...\n");
    }

    // read distance sensor values
    double sensor_values[NUM_SENSORS];
    for (i = 0; i < NUM_SENSORS; i++)
      sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);

    // compute braitenberg collision avoidance
    double speed[2];
    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
      for (j = 0; j < NUM_SENSORS; j++)
        speed[i] += EPUCK_MATRIX[j][i] * (1 - (sensor_values[j] / RANGE));
    }

    // set the motors speed
    wb_differential_wheels_set_speed(speed[0], speed[1]);

    // simulation step
    wb_robot_step(TIME_STEP);
  }

  return 0;
}
