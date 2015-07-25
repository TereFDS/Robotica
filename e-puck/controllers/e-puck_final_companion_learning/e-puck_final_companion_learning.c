#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/accelerometer.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP           64
#define TIME_STEP_MULTIPLIER 4
#define WHEEL_RADIUS        0.0205
#define AXLE_LENGTH         0.052
#define ENCODER_RESOLUTION  159.23
#define RANGE               (1024 / 2)

#define X_AXIS              0
#define Y_AXIS              2
#define Z_AXIS              1

#define FORWARD_SPEED       170
#define BACKWARD_SPEED      170


#define GRAVITY             9.8

#define ACC_THRESHOLD       1.4
#define ACC_THRESHOLD_LITLE 4

#define ACTIONS             5
#define STATES              5

#define GAMMA               0.9 
#define ALPHA               0.1

#define EPSILON_DELTA       0.001

#define MAX_EPS             1
#define TRAIN_ACTIVE        1


int epsilon_count = 2; //cantidad de epocas
float eps = MAX_EPS;


/*float Q[STATES][ACTIONS] = {
{-9011.671875, -13301.423828, -9372.247070}, 
{-2658.381104, 44.130100, 11997.417969}, 
{7.219242, 5321.901367, 931.141235} 
};*/

float Q[STATES][ACTIONS] = {
{0,0,0},
{0,0,0},
{0,0,0},
{0,0,0},
{0,0,0} 
};


double getAcceleration(WbDeviceTag accelerometer) {
    double * value =wb_accelerometer_get_values(accelerometer);   
    //printf("otra posible medida? %f\n",sin(value[X_AXIS]));
    //return asin(value[X_AXIS]);
    return (asin(value[Z_AXIS]/GRAVITY))*180/M_PI;
}

typedef enum {
    BALANCED, LITLE_LOW, LOW, LITLE_HIGH, HIGH
} State;

typedef enum {
    STAY, GO_FOWARD_LITLE, GO_FOWARD, GO_BACKWARD_LITLE, GO_BACKWARD
} Action;


int main(int argc, char *argv[]) {
    
    /* initialize Webots */
    wb_robot_init();
    
    
    // ENABLING WHEELS
    wb_differential_wheels_enable_encoders(TIME_STEP*TIME_STEP_MULTIPLIER);
    wb_robot_step(TIME_STEP*TIME_STEP_MULTIPLIER);
    
    for (;;) {
        wb_differential_wheels_set_speed(FORWARD_SPEED, FORWARD_SPEED);        
        wb_robot_step(TIME_STEP*TIME_STEP_MULTIPLIER);
        /*nextAction = chooseAction(prevState);
        executeAction(nextAction);
        
        currentState = getNewState(accelerometer);
        printf("estait: %d\n",currentState);
        if (TRAIN_ACTIVE == 1)
          updateQ(nextAction, prevState, currentState, reinforcement);
        
        updateEpsilon();
        print_matrix();
        prevState = currentState;
        */
    }
    
    return 0;
}