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

#define FORWARD_SPEED       100
#define BACKWARD_SPEED      100


#define GRAVITY             9.8

#define ACC_THRESHOLD       0.1

#define ACTIONS             3
#define STATES              3

#define GAMMA               0.001 // antes 0.9
#define ALPHA               0.005

#define EPSILON_DELTA       0.001

#define MAX_EPS             0
#define TRAIN_ACTIVE        0


int epsilon_count = 0;
float eps = MAX_EPS;


float Q[STATES][ACTIONS] = {
{-9011.671875, -13301.423828, -9372.247070}, 
{-2658.381104, 44.130100, 11997.417969}, 
{7.219242, 5321.901367, 931.141235} 
};


double getAcceleration(WbDeviceTag accelerometer) {
    double * value =wb_accelerometer_get_values(accelerometer);
    return (atan2(value[Y_AXIS],value[X_AXIS])-M_PI/2)/(2*M_PI)*360;
}

typedef enum {
    BALANCED, LOW, HIGH
} State;

typedef enum {
    STAY, GO_FOWARD, GO_BACKWARD
} Action;

State getNewState(WbDeviceTag tag);
Action chooseAction(const State state);
void executeAction(const Action nextAction);
void updateQ(Action action, State prevState, State currentState,float reinforcement);
Action getMaxRewardAction(State state);
float random_num();
int reinforcement_function(State actualState, State prevState);
void updateEpsilon();
void print_matrix();

int main(int argc, char *argv[]) {
    float yAcceleration;
    float reinforcement = 0;
    Action nextAction;
    State prevState, currentState;
    
    /* initialize Webots */
    wb_robot_init();
    
    /* ENABLE ACCELEROMETER */
    WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(accelerometer, TIME_STEP*TIME_STEP_MULTIPLIER);
    
    // ENABLING WHEELS
    wb_differential_wheels_enable_encoders(TIME_STEP*TIME_STEP_MULTIPLIER);
    wb_robot_step(TIME_STEP*TIME_STEP_MULTIPLIER);
    prevState = getNewState(accelerometer);
    
    for (;;) {
        nextAction = chooseAction(prevState);
        executeAction(nextAction);
        
        currentState = getNewState(accelerometer);
        
        if (TRAIN_ACTIVE == 1)
          updateQ(nextAction, prevState, currentState, reinforcement);
        
        updateEpsilon();
        
        prevState = currentState;
    }
    
    return 0;
}

void
executeAction(const Action nextAction) {
  
    if (nextAction == GO_FOWARD) {
        wb_differential_wheels_set_speed(FORWARD_SPEED, FORWARD_SPEED);
    } else if (nextAction == GO_BACKWARD) {
        wb_differential_wheels_set_speed(-BACKWARD_SPEED, -BACKWARD_SPEED);
    } else {
        wb_differential_wheels_set_speed(0, 0);
    }


  wb_robot_step(TIME_STEP*TIME_STEP_MULTIPLIER);

}

State
getNewState(WbDeviceTag tag) {
    
    float a = getAcceleration(tag);
    if (fabs(a) < ACC_THRESHOLD) {
        return BALANCED;
    } else if (a > 0) {
        return HIGH; 
    } else if (a < 0) {
        return LOW;
    }
}

Action
chooseAction(const State state) {
    float r = random_num();
    Action next_action;
    if (r >= eps) {
        // exploit
        next_action = getMaxRewardAction(state);        
    } else {
        //explore
        next_action= rand() % ACTIONS;
    }
        return next_action;
}

void
updateQ(Action action, State prevState, State currentState, float reinforcement) {
    
    int r = reinforcement_function(currentState,prevState);
    short bestAction = getMaxRewardAction(currentState);
    float max = GAMMA * Q[currentState][bestAction];
    
    Q[prevState][action] += ALPHA * (r + max - Q[prevState][action]);
}

//revisar
int
reinforcement_function(State actualState, State prevState ) {
  if (actualState == LOW && prevState == LOW) {
    return 0;
  } else if (actualState == HIGH && prevState == HIGH) {
    return -10;
  } else if (actualState == BALANCED && prevState != BALANCED) {
    return 100000;
  } else if (actualState != BALANCED && prevState == BALANCED) {
    return -100000;
  } else if (actualState == BALANCED && prevState == BALANCED) {
    return 1;
  } else {
    return 0;
  }
}

float
random_num() {
    return (float) rand() / (float) RAND_MAX;
}

Action
getMaxRewardAction(State state) {
  short i = state;
  int j = 0, max_index = 0;
  
  for (j = 0; j <= ACTIONS; j++){
    if(Q[i][max_index] <= Q[i][j]){
      max_index = j;
    }
  }
  return max_index;
}

void
updateEpsilon() {
    if (eps <= 0) {
      if (epsilon_count > 0) {
        eps = MAX_EPS;
        epsilon_count--;
      } else {
        eps = 0;
      }
    } else {
        eps -= EPSILON_DELTA;
    }
}

void print_matrix(){
  int states, actions;
  for (states = 0; states < STATES; states++) {
    for(actions = 0; actions < ACTIONS; actions++) {
      printf("%f ", Q[states][actions]);
    }
    printf("\n");
  }
}
