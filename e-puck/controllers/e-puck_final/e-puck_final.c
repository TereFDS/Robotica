#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/accelerometer.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 64
#define TIME_STEP_MULTIPLIER 10
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)
#define X_AXIS 0
#define Y_AXIS 2
#define Z_AXIS 1
#define FORWARD_SPEED 170
#define BACKWARD_SPEED 170
#define GRAVITY 9.8
#define ACC_THRESHOLD 1.3
#define ACC_THRESHOLD_LITLE 3.8
#define ACTIONS 3
#define STATES 5
#define GAMMA 0.9
#define ALPHA 0.1
#define EPSILON_DELTA 0.001
#define MAX_EPS 1
#define TRAIN_ACTIVE 1
#define EPOQUES 4
#define ITERATIONS 1000

int epsilon_count = 4; //cantidad de epocas -1 (ahora tiene que valer 4 porque usamos 5 epocas)
float eps = MAX_EPS;
float learning_reinforcement[(int)(1000)*2];
int learning_index= 0;
/*float Q[STATES][ACTIONS] = {
{408.846466, 398.468719, 409.099731, 385.317017, 409.766296},
{379.227448, 458.710358, 428.249939, 392.432892, 389.561829},
{437.941528, 536.843750, 549.125122, 429.928955, 441.923462},
{276.978668, 319.857056, 319.968262, 326.143463, 269.744568},
{293.818787, 316.082855, 253.805588, 638.329224, 489.348602}
};*/

// nueva matriz (que parece funcionar)

/*float Q[STATES][ACTIONS] = {
{-1.322773, -3.350827, -2.919920}, 
{-2.665848, 0.191065, -9.550050}, 
{-25.448944, -12.997213, -33.240997},
{-9.330454, -6.218776, -0.099255},
{-21.824425, -17.649944, -5.335610}
};*/  

/*float Q[STATES][ACTIONS] = {
{8.608189, -3.487211, -1.776637}, 
{-6.350226, -0.974549, -8.272821}, 
{-16.809811, -13.805308, -16.765018},
{-8.367319, -7.912634, -2.446282}, 
{-11.970656, -14.247732, -8.355040},
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
    STAY, /*GO_FOWARD_LITLE, */GO_FOWARD, /*GO_BACKWARD_LITLE,*/ GO_BACKWARD
} Action;

State getNewState(WbDeviceTag tag);
Action chooseAction(const State state);
void executeAction(const Action nextAction);
void updateQ(Action action, State prevState, State currentState);
Action getMaxRewardAction(State state);
float random_num();
int reinforcement_function(Action action, State actualState, State prevState);
void updateEpsilon();
void print_matrix();
void save_matrix();
void read_matrix();
void write_learning_curve();
void write_epsilon();
void read_epsilon();

int main(int argc, char *argv[]) {
  int iterations=0;
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
  if(TRAIN_ACTIVE == 1)
    eps = -1;
  
  read_matrix();
  
  for (;;) {
    nextAction = chooseAction(prevState);
    executeAction(nextAction);
    currentState = getNewState(accelerometer);
    printf("state: %s\n",(currentState==0)?"balanced":((currentState==1)?"litle low":((currentState==2)?"low":((currentState==3)?"litle high":"high"))));
    
    if (epsilon_count>=0){
      updateQ(nextAction, prevState, currentState);
      updateEpsilon();
    }
    iterations++;
    print_matrix();
    prevState = currentState;
  }
  write_learning_curve();
  save_matrix();
  write_epsilon();
  return 0;
}

void
executeAction(const Action nextAction) {
  /*if (nextAction == GO_FOWARD_LITLE) {
  wb_differential_wheels_set_speed(FORWARD_SPEED, FORWARD_SPEED);
  }
  else */if (nextAction == GO_FOWARD) {
    wb_differential_wheels_set_speed(1*FORWARD_SPEED, 1*FORWARD_SPEED);
  } /*else if (nextAction == GO_BACKWARD_LITLE) {
  wb_differential_wheels_set_speed(-BACKWARD_SPEED, -BACKWARD_SPEED);
  }*/ else if (nextAction == GO_BACKWARD){
    wb_differential_wheels_set_speed(-1*BACKWARD_SPEED, -1*BACKWARD_SPEED);
  }
  else {
    wb_differential_wheels_set_speed(0, 0);
  }
  wb_robot_step(TIME_STEP*TIME_STEP_MULTIPLIER);
}

State
getNewState(WbDeviceTag tag) {
  float a = getAcceleration(tag);
  printf("inclination angle: %f\n",a);
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
updateQ(Action action, State prevState, State currentState) {
    
    int r = reinforcement_function(action, currentState,prevState);
    short bestAction = getMaxRewardAction(currentState);
    float max = GAMMA * Q[currentState][bestAction];
    float temporal_difference = ALPHA * (r + max - Q[prevState][action]);
    Q[prevState][action] += temporal_difference;
    if(learning_index< (1000*2)){
      learning_reinforcement[learning_index]=(learning_index==0)?0:learning_reinforcement[learning_index-1];
      learning_reinforcement[learning_index]=learning_reinforcement[learning_index]+r;
      learning_index++;
    }

}

//revisar
int
reinforcement_function(Action action, State actualState, State prevState ) {
  /*if(prevState==actualState && actualState==BALANCED){
    return 20;
  }
  else if(prevState==actualState && (actualState==LITLE_LOW)){
    return -2;
  }
  else if(prevState==actualState && (actualState==LITLE_HIGH)){
    return -2;
  }
  else if(prevState==actualState && (actualState==LOW)){
    return -8;
  }
  else if(prevState==actualState && (actualState==HIGH)){
    return -8;
  }
  else if(prevState==BALANCED && (actualState==LITLE_LOW || actualState==LITLE_HIGH)){
    return -4;
  }
  else if(prevState==BALANCED && (actualState==LOW || actualState==HIGH)){
    return -16;
  }
  else if(actualState==BALANCED){
    return 16;
  }
  else if((prevState==LOW ||prevState==HIGH)&& (actualState==LITLE_LOW || actualState==LITLE_HIGH)){
    return 6;
  }
  else if((prevState==LITLE_LOW ||prevState==LITLE_HIGH)&& (actualState==LOW || actualState==HIGH)){
    return -6;
  }*/
  if(actualState==BALANCED){
    return 2;
  }
  else if(actualState==LITLE_LOW){
    return -1;
  }
  else if(actualState==LITLE_HIGH){
    return -1;
  }
  else if(actualState==LOW){
    return -4;
  }
  else if(actualState==HIGH){
    return -4;
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
        
      } else {
        if(epsilon_count == 0){
          printf("guarda refuerzosssss\n");
          write_learning_curve();
          save_matrix();
        }
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

void write_epsilon(){
  FILE* fp = fopen("learning_epsilon.csv","w");
  if(fp != NULL){
    int i=0;
    for(i=0;i<learning_index;i++){
      fprintf(fp,"%f\n",eps);
    }
  }
}

void read_epsilon(){
  FILE* fp;
  fp=fopen("learning_epsilon.csv","r");
  if(fp!= NULL){
    fscanf(fp,"%f\n", &eps);
  }
  fclose(fp);
}
