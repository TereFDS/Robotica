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

#define ACC_THRESHOLD       1.3
#define ACC_THRESHOLD_LITLE 3.8

#define ACTIONS             5
#define STATES              5

#define GAMMA               0.9 
#define ALPHA               0.1

#define EPSILON_DELTA       0.001

#define MAX_EPS             1
#define TRAIN_ACTIVE        1


int epsilon_count = 3; //cantidad de epocas
float eps = MAX_EPS;
float learning_reinforcement[(int)(MAX_EPS/EPSILON_DELTA)*4];
int learning_index= 0;


/*float Q[STATES][ACTIONS] = {
{408.846466, 398.468719, 409.099731, 385.317017, 409.766296}, 
{379.227448, 458.710358, 428.249939, 392.432892, 389.561829},
{437.941528, 536.843750, 549.125122, 429.928955, 441.923462},
{276.978668, 319.857056, 319.968262, 326.143463, 269.744568}, 
{293.818787, 316.082855, 253.805588, 638.329224, 489.348602}
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

State getNewState(WbDeviceTag tag);
Action chooseAction(const State state);
void executeAction(const Action nextAction);
void updateQ(Action action, State prevState, State currentState,float reinforcement);
Action getMaxRewardAction(State state);
float random_num();
int reinforcement_function(Action action, State actualState, State prevState);
void updateEpsilon();
void print_matrix();
void save_matrix();
void read_matrix();
void write_learning_curve();


int main(int argc, char *argv[]) {
    float yAcceleration;
    float reinforcement = 0;
    Action nextAction;
    State prevState, currentState;
    
    /* initialize Webots */
    wb_robot_init();
    //read_matrix();
    
    /* ENABLE ACCELEROMETER */
    WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(accelerometer, TIME_STEP*TIME_STEP_MULTIPLIER);
    
    // ENABLING WHEELS
    wb_differential_wheels_enable_encoders(TIME_STEP*TIME_STEP_MULTIPLIER);
    wb_robot_step(TIME_STEP*TIME_STEP_MULTIPLIER);
    prevState = getNewState(accelerometer);
    if(TRAIN_ACTIVE == 0)
      eps = 0;
    
    for (;;) {
        nextAction = chooseAction(prevState);
        executeAction(nextAction);      
        currentState = getNewState(accelerometer);
        printf("estait: %d\n",currentState);
        if (epsilon_count>=0){
          updateQ(nextAction, prevState, currentState, reinforcement);
          updateEpsilon();
        }
        print_matrix();
        prevState = currentState;
        
    }
    
    return 0;
}

void
executeAction(const Action nextAction) {
  
    if (nextAction == GO_FOWARD_LITLE) {
        wb_differential_wheels_set_speed(FORWARD_SPEED, FORWARD_SPEED);
    } 
    else if (nextAction == GO_FOWARD) {
        wb_differential_wheels_set_speed(2*FORWARD_SPEED, 2*FORWARD_SPEED);
    } else if (nextAction == GO_BACKWARD_LITLE) {
        wb_differential_wheels_set_speed(-BACKWARD_SPEED, -BACKWARD_SPEED);
    } else if (nextAction == GO_BACKWARD){ 
        wb_differential_wheels_set_speed(-2*BACKWARD_SPEED, -2*BACKWARD_SPEED);
    }
    else {
        wb_differential_wheels_set_speed(0, 0);
    }


  wb_robot_step(TIME_STEP*TIME_STEP_MULTIPLIER);

}

State
getNewState(WbDeviceTag tag) {
    
    float a = getAcceleration(tag);
    printf("aceleracion: %f\n",a);
    if (fabs(a) < ACC_THRESHOLD) {
        return BALANCED;
    } else if (a >= 0) { 
        return (fabs(a)<ACC_THRESHOLD_LITLE)?LITLE_LOW:LOW; 
    } else if (a < 0) {
        return (fabs(a)<ACC_THRESHOLD_LITLE)?LITLE_HIGH:HIGH;
    }

}

Action
chooseAction(const State state) {
    float r = random_num();
    Action next_action;
    if (r >= eps) {
        // exploit
        printf("exploit\n");
        next_action = getMaxRewardAction(state);        
    } else {
        //explore
        printf("explore\n");
        next_action= rand() % ACTIONS;
    }
        return next_action;
}

void
updateQ(Action action, State prevState, State currentState, float reinforcement) {
    
    int r = reinforcement_function(action, currentState,prevState);
    short bestAction = getMaxRewardAction(currentState);
    float max = GAMMA * Q[currentState][bestAction];
    float temporal_difference = ALPHA * (r + max - Q[prevState][action]);
    Q[prevState][action] += temporal_difference;
    if(learning_index< ((MAX_EPS/EPSILON_DELTA)*4)){
      learning_reinforcement[learning_index]=(learning_index==0)?0:learning_reinforcement[learning_index-1];
      learning_reinforcement[learning_index]=learning_reinforcement[learning_index]+temporal_difference;
      learning_index++;
    }

}

//revisar
int
reinforcement_function(Action action, State actualState, State prevState ) {
  /*if (actualState == LOW && prevState == LOW) {
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
  }*/
  if(prevState==actualState && actualState==BALANCED){
    return 200;
  }
  else if(prevState==actualState && (actualState==LITLE_LOW)){
    if(action == GO_FOWARD_LITLE || action == GO_FOWARD){
      return 20;
    }
    return -20;
  }
  else if(prevState==actualState && (actualState==LITLE_HIGH)){
    if(action == GO_BACKWARD_LITLE || action == GO_BACKWARD){
      return 20;
    }
    return -20;
  }
  else if(prevState==actualState && (actualState==LOW)){
    if(action == GO_FOWARD_LITLE || action == GO_FOWARD){
      return 80;
    }
    return -80;
  }
  else if(prevState==actualState && (actualState==HIGH)){
    if(action == GO_BACKWARD_LITLE || action == GO_BACKWARD){
      return 80;
    }
    return -80;
  }
  else if(prevState==BALANCED && (actualState==LITLE_LOW || actualState==LITLE_HIGH)){
    return -40;
  }
  else if(prevState==BALANCED && (actualState==LOW || actualState==HIGH)){
    return -160;
  }
  else if(actualState==BALANCED){
    return 100;
  }
  else if((prevState==LOW ||prevState==HIGH)&& (actualState==LITLE_LOW || actualState==LITLE_HIGH)){
    return 60;
  }
  else if((prevState==LITLE_LOW ||prevState==LITLE_HIGH)&& (actualState==LOW || actualState==HIGH)){
    return -60;
  }
  return 0;
}

float
random_num() {
    return (float) rand() / (float) RAND_MAX;
}

Action
getMaxRewardAction(State state) {
  short i = state;
  int j = 0, max_index = 0;
  
  for (j = 0; j < ACTIONS; j++){
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
        if(epsilon_count == 0){
          write_learning_curve();
          save_matrix();

        }
      } else {
        eps = 0;
        epsilon_count--;        
      }
      
    } else {
        eps -= EPSILON_DELTA;
    }
    printf("epsilon: %f \n",eps);
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

void save_matrix(){
  FILE* fp;
  fp=fopen("matrix.data","wb");
  if(fp!= NULL){
    fwrite(Q,sizeof(float),STATES*ACTIONS,fp);
  }
  fclose(fp);
}

void read_matrix(){
  FILE* fp;
  fp=fopen("matrix.data","rb");
  if(fp!= NULL){
    fread(Q,sizeof(float),STATES*ACTIONS,fp);
  }
  fclose(fp);
}

void write_learning_curve(){
  FILE* fp = fopen("learning_curve.csv","w");
  if(fp != NULL){
    int i=0;
    for(i=0;i<learning_index;i++){
      fprintf(fp,"%f\n",learning_reinforcement[i]);
    }

  }

}