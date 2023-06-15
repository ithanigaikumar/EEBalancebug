//Outputs to motors. Green for right, orange for left motor
#define left_dir_pin 27
#define left_step_pin 26
#define right_dir_pin 25
#define right_step_pin 33
#define steps_per_revolution 200

#define sgn(x) ((x) < 0 ? LOW : HIGH)  

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h> 

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

/*STATE OF ROVER*/ //global
bool is_vertical=false;
bool listen_command=false; //true: can accept rover commands,  false: can't listen to command as currently doing a command
/*Is not used to tell the website if the rover is available anymore. Is now a local state and website controls commands by https handshakes*/
//set to false first because rover will be open to commands ONLY AFTER CALIBRATION IS DONE


/*STATES*/ //global
struct sensor_outputs{ //buffer for sensor outputs to prevent data races
   float pitch; //pitch of the rover
   float pitch_rate; //pitch rate of the rover
   float x; //distance rover traveled
   float v; //velocity of rover
   float yaw; //previous v recorded in previous sense call. Made according to w value used during that time
   float yaw_rate;
};

sensor_outputs sensor_states = {0, 0, 0, 0, 0, 0};
sensor_outputs sensor_states_prev = {0, 0, 0, 0, 0, 0};







/*LAPTOP INPUTS*/ //THE DESIRED STATE INPUTS 
float yaw_desired=0; //the actual yaw we'll try to change to. Change during turns
float yaw_rate_desired=0;
float v_desired=0; //set by laptop and methods
float x_desired=0; //set by methods and only used for crash()
int vanish=0; //pixel distance between vanishing point and image center sent by laptop. Laptop will add disturbance to it and the controller will try to keep it zero by yaw rate control
bool mov=true; //should rover move while doing moveForward()?



/*MULTITHREADING FLAGS*/ //global
bool sensor_states_changing=false; //on while sense() is running so image broadcaster will wait until yaw and v are updated
bool listen_command_changing=false;
bool state_updated=false;
//threads read sensor state values from buffer, which is flagged, while state itself is flagged directly

/* Websocket server & UART setup*/
const char* ssid = "esp32aptestrle";
const char* password = "wasdqwerty32";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // create WebSocket instance
HardwareSerial SerialPort(2);
TaskHandle_t sendVideo;
TaskHandle_t controlTask;
TaskHandle_t senseTask;

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    // client connected
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if(type == WS_EVT_DISCONNECT){
    // client disconnected
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

/*
 * Function Name: mpuSetup() control
 * Input: None
 * Output: None
 * Function: Calibrates MPU based on desired range mode
 */
void mpu_setup() {

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu.setTemperatureStandby(true);

  mpu_calibrate();
  
}




/*Calibration only at start and zero rate compensation during movement*/
/*Make website see calibration taking place*/

//Will be called at start during mpu_setup while the rover is stationary and gets the mpu offsets needed to get stationary values zero and reduce drift
//Take too long to compute so only done on start on control
void mpu_calibrate(){ //ROVER NEEDS TO BE STATIONARY. WHEN ROVER DOESN'T MOVE RECORD G AND A VALUES FOR SOME TIME THEN SET THEM AS OFFSETS

  Serial.println("GET THE ROVER TO HORIZONTAL POSITION");
  delay(1500);
  Serial.print("Calibrating...");
  int calibration_cycles=150; //more cycles called mean more of the drift is compensated

  float gx_offset_found=0;
  float gy_offset_found=0;
  float gz_offset_found=0;
  float ax_offset_found=0;
  float ay_offset_found=0;
  float az_offset_found=0;

  for(int i=0; i<calibration_cycles; i++){
    

    mpu.getEvent(&a, &g, &temp);

    gx_offset_found+=g.gyro.x;
    gy_offset_found+=g.gyro.y;
    gz_offset_found+=g.gyro.z;
    ax_offset_found+=a.acceleration.x;
    ay_offset_found+=a.acceleration.y;
    az_offset_found+=a.acceleration.z;
    
  }

  Serial.println("Calibration complete. Set to vertical orientation before pressing start");

  gx_offset=gx_offset_found/calibration_cycles;
  gy_offset=gy_offset_found/calibration_cycles;
  gz_offset=gz_offset_found/calibration_cycles;
  ax_offset=ax_offset_found/calibration_cycles;
  ay_offset=ay_offset_found/calibration_cycles;
  az_offset=az_offset_found/calibration_cycles;

  /*
   * Set listen_command to zero (start at zero) to tell website to not start until calibration is done
   * Wait for 2 seconds until rover 
   * Take repeated measurements for some cycles to get g and a offsets
   * Get avg of those offsets, which would be used to get rid of the offset
   * Set listen_command to zero, which will allow website buttons to be pressed
   * 
   * Person will get rover to vertical position
   * Website presses begin button
   * FIRST COMMAND IS STOP() TO GET VELOCITY TO ZERO
   * SECOND COMAND IS MOVEFORWARD(), WHICH STARTS THE ROVER OPERATION
   * 
   * 
   * FOR MOVEFORWARD STATE REMAINS SAME NO NEED FOR TIMING CONSTRAINTS
   * 
   */

  
}





//updates the offset of the corresponsing yaw_rate giving gyroscope measurement IF NOT GIVING W_TURNS TO MOTORS FOR 20 SENSOR SAMPLES
void compensate_yaw(float gx, float gz){

  float zero_rate_value=is_vertical? gx:gz;
  bool is_input_yaw_rate_zero = abs(w_turn*(wheel_radius/rover_width))<=0.03; //do compensation if we're not giving YAW RATE INPUT, not if sensed yaw rate is zero. NOT ACCURATE UNTIL COMPENSATED

  if(doing_yaw_compensation){

    if(is_input_yaw_rate_zero) { //if doing compensation and yaw rate stays zero

      compensated_yaw_offset+=zero_rate_value;
      yaw_compensation_samples++;
      
    }

    else{ //if yaw rate goes nonzero while we're doing compensation. End of compensation

      doing_yaw_compensation=false;
      float* compansated_measurement=is_vertical ? &gx_offset : &gz_offset;

      //Take compensation as successfull and accurate if we were able to get 20 samples while yaw_rate was zero
      *compansated_measurement = yaw_compensation_samples>=20 ? compensated_yaw_offset/yaw_compensation_samples : *compansated_measurement; 
      yaw_compensation_samples=0;
      
    }
    
  }

  else{

    if(is_input_yaw_rate_zero){ //start compensation when zero yaw rate is detected

      doing_yaw_compensation=true;
      
    }
    
  }
  
}
  



//A GENERAL SENSE FUNCTION THAT MEASURES ALL STATES BY ONLY 2 SAMPLES. COULD CHANGE WITH NEW MPU LIBRARY AS IT CAN TAKE INDIVIDUAL READINGS
void sense(){ //do measurement only 2 times and use data to find pitch, yaw, and v all at the same time

  //Serial.println("sense called");

  sensor_outputs sensor_buffer;

  float dt_start=micros()*0.000001-sense_time; //one dt is that recorded in start, other is one recorded after one sampling time run
  float dt=0;
  float ax=0;
  float ay=0;
  float az=0;
  float gx=0;
  float gy=0;
  float gz=0;
  //velocity and distance are out of loop because v is directly given by w, which is not changed during sense period (ensured that)

  //need stable pitch to measure yaw rate, so measure desired gyros first

  //get accurate rate samples
  for(int i=0; i<total_samples; i++){  //in future do only pitch_rate if would only do the FPGA based measurement (DOWNSIDE IS IT WOULD TAKE LONGER TIME TO DO DUE TO SERIAL)

    mpu.getEvent(&a, &g, &temp); //will optimize in the future where only gyro or accelero is called at a time instead of measuring both at each call
    dt+=(micros()*0.000001-sense_time);

    ax+=a.acceleration.x-ax_offset;
    ay+=a.acceleration.y-ax_offset;
    az+=a.acceleration.z-ax_offset;

    gx+=g.gyro.x-ax_offset;
    gy+=g.gyro.y-ax_offset;
    gz+=g.gyro.z-ax_offset;
    
  }

  float rate = is_vertical ? gy : -gy;

  /*pitch rate*/
  sensor_buffer.pitch_rate=((abs(rate/total_samples)>=0.03) ? rate/total_samples : 0); //Added hysteresis to prevent drift //VALUE IN RAD, TRANSMISSION AND INPUT IN DEGREES
  sensor_states_prev.pitch_rate=sensor_buffer.pitch_rate;
  
  float hor= is_vertical ? ax : az;
  float ver= is_vertical ? az : ax;            //relative vertical of the triangle whose acute angle gives the pitch
  float acc_pitch = atan2(ver/total_samples, sqrt(pow(ay/total_samples,2) + pow(hor/total_samples,2)));
  //ADD OFFSET TO GET PITCH FOR CENTER OF MASS INSTEAD FOR ACTUAL PITCH OF SENSOR
  //SWITCHES X AND Z PLACES TO GET VERTICAL TO 0 AND HORIZONTAL TO 90
  //Serial.print("acc pitch: ");
  //Serial.println(acc_pitch*(360/(2*PI)));
  float gyro_pitch = sensor_states.pitch + 0.5*(sensor_states_prev.pitch_rate+sensor_buffer.pitch_rate) * dt/total_samples;    //IF GYRO IS USED ONLY, ONLY GOOD FOR DYNAMIC FORCE (ACCELERATION DIE OUT AND NOT INTERFERE WITH GRAVITY)
  //Serial.print("gyro pitch: ");
  //Serial.println(gyro_pitch*(360/(2*PI)));
  sensor_buffer.pitch = ((1-complementary_weight) * gyro_pitch + complementary_weight * acc_pitch) - pitch_offset*is_vertical;
  
  float turn=is_vertical? gx:gz;

  sensor_buffer.yaw_rate=((abs((-turn/cos(sensor_states.pitch))/total_samples)>=0.03) ? (-turn/cos(sensor_states.pitch))/total_samples : 0);
  sensor_states_prev.yaw_rate=sensor_buffer.yaw_rate;
  //subtract by roll*sin(pitch) if can find roll. gx and gz detect roll and yaw TOGETHER and CAN'T RECTIFY THAT. Could find roll from accelerometer then rollrate from dt???
  //roll interference and drift are the main protractors of accurate yaw measurement. MEASURE ROLL INTERFERENCE TOO SOMEHOW???
  //NEED TO DO PITCH SENSE BEFORE DOING YAW SENSE->TOGETHER. Also do velocity together as it's a quick measurement

  sensor_buffer.yaw=sense_yaw_sensor(dt/total_samples); //If can do beacon detection change it to detect yaw from beacon that comes from laptop. Save laptop data to a register and set flag high when value at buffer updated
  

  sensor_buffer.v = w * wheel_radius; //directly set by input so no need to stabilize
  sensor_buffer.x = sensor_states.x+0.5*(sensor_buffer.v+sensor_states_prev.v)*dt;
  sensor_states_prev.v = sensor_buffer.v;

  sensor_states_changing=true; //FLAG TO TELL DEAD RECKONING NOT TO READ THE BUFFER UNTIL READ IS COMPLETE
  change_sensor_states(sensor_buffer); //PUT COPYING INTO A METHOD SO THAT METHOD COULD BE PUT TO A PRIORITY LIST
  sensor_states_changing=false;
  state_updated=true;
  
}



/*CONTROLLERS*/

//NEED TO CONTINUOUSLY CALL UPDATES() FOR CONTROLLER TO UPDATE RESPONSE. FOR BOTH PID AND LQR. Is there a way to continuously run them in the background???

//NOT FINISHED YET. IRON OUT IMPLEMENTATION HICCUPS

void set_velocity(double vd){

  v_desired=vd;

  //SET_VELOCITY IS A COMMAND SEND BY LAPTOP

}

//Have basic constants so start testing. More overshoot a state does increase its cost


void update_velocity(){ //vd=v, x can change

  //DO THE CONTROLLER STUFF THERE
  /*
   * get feedback from sensors
   * scale them with optimal gain coefficients to get actual input to subtract from. 
   * apply new input to system by setting new w_desired (w_rate_desired for turns
   * run the motor updater code (the code only does steps if time range set by w inputs is exceeded)
   * 
   */

   //x v pitch and pitchrate are updated


   //FIND W UPDATE THEN CALLUPDATE WHEEL SPEEDS TO GIVE RESPONSE. WOULD ALSO UPDATE TURN AND BALANCING IF NECESSARY
   float timeout=micros()*0.000001;
   while(sensor_states_changing && micros()*0.000001-timeout_time<min_sense_period*3){} //wait if buffer is changing
   
   float w_feed = is_vertical ? -(k[0]*(v_desired-sensor_states.v)-k[1]*sensor_states.pitch-k[2]*sensor_states.pitch_rate) : v_desired/wheel_radius;

   set_wheel_speeds(w_feed, w_turn);
   
  
}

void set_distance(float x_d){

   x_desired=x_d;

   //SET DISTANCE IS A COMMAND SEND BY LAPTOP
  
}



void update_distance() { //yaw_d=yaw, yaw_rate can change

  float timeout=micros()*0.000001;
  while(sensor_states_changing && micros()*0.000001-timeout_time<min_sense_period*3){} //wait if buffer is changing

  float error=x_desired-sensor_states.x;

  float new_v=kp_x*error; //PID OUTPUT
  set_velocity(new_v);
  update_velocity();
  
}



void set_yaw_rate(float rate_d){ //yaw_rate_d=yaw_rate, desired_yaw not used

  yaw_rate_desired=rate_d;
  //implement velocity controller here, will set w and yaw rate desired at the end

}

void update_yaw_rate() { //yaw_d=yaw, desired_yaw_rate can change

  float dt=micros()*0.000001-yaw_rate_pid_time;
  
  float error=yaw_rate_desired-sensor_states.yaw_rate;
  yaw_rate_cum_error+=error*dt; //integration of error

  //Not use states so no need to wait for sensor buffer

  float w_feed= is_vertical ? ki*yaw_rate_cum_error : -yaw_rate_desired * (rover_width/wheel_radius); //PID OUTPUT  //minus sign put to account for direction differences of the motors

  set_wheel_speeds(w, w_feed);

  yaw_rate_pid_time=micros()*0.000001;

  //Used an I controller because closed loop response was GR/(1+GR) where G was a constant. It gave response (G*ki)/(s+G*ki), so ki can directly control bandwidth.

}


void set_yaw(float yaw_d){ //Assume given yaw is in rev 0. Offset the given yaw input to the rev that makes the difference between yaw and it the smallest

  float timeout=micros()*0.000001;
  while(sensor_states_changing && micros()*0.000001-timeout_time<min_sense_period*3){} //wait if buffer is changing 

  int rev=sensor_states.yaw/(2*PI) + !sgn(sensor_states.yaw)*(-1); //get the revolution yaw value is at. Pass 360 you get one rotation pass 0 you lose one rotation    -360 : -1 -1 | 0 : 359  0 | 360 : 719 1

  //chose rev rev-1 or rev+1 based on closest 

  int offset=0;
  
  float mag_yaw_difference=abs((yaw_d+2*PI*rev)-sensor_states.yaw);
  
  if(abs((yaw_d+2*PI*(rev+1))-sensor_states.yaw)>=mag_yaw_difference){

    mag_yaw_difference=abs(yaw_d*(rev+1)-sensor_states.yaw);
    offset=1;
    
  }

  if(abs((yaw_d+2*PI*(rev-1))-sensor_states.yaw)>=mag_yaw_difference){

    mag_yaw_difference=abs(yaw_d*(rev-1)-sensor_states.yaw);
    offset=-1;
    
  } //TWIST TURNS DIRECTIONS FOR HORIZONTAL MODE
  

   yaw_desired=yaw_d+2*PI*(rev+offset); //input is scaled by the amount of full turns the rover already did from the start.

   //IS A COMMAND SENT BY LAPTOP, BUT WOULD GET DEGREES DATA THAT HASN'T BEEN UNWRAPPED. CONVERT TO RADS FIRST THEN GIVE TO SET_YAW TO BE UNWRAPPED OPTIMALLY
  
}



void update_yaw(){ //yaw_d=yaw, desired_yaw_rate can change

  float timeout=micros()*0.000001;
  while(sensor_states_changing && micros()*0.000001-timeout_time<min_sense_period*3){} //wait if buffer is changing

  float error=yaw_desired-sensor_states.yaw;

  float new_yaw_rate=kp_yaw*error; //PID OUTPUT
  set_yaw_rate(new_yaw_rate);
  update_yaw_rate();
  
}

//do straightening during motion if mov is true, if false do when stationary
void straighten(int vanish, bool mov){ //max yaw displacement is 75 degrees. Increase for sharper turns but not 90 or would get stuck on large corridors

  float straighten_turn=(vanish*(max_turn_angle)/(image_width/2)); //negative vanish offset lead to negative yaw rate
  set_velocity(mov*v_normal);
  set_yaw_rate(straighten_turn); //can both run in parallel??? IF NOT TURN IT INTO AN OPERATION. DO IT WHENEVER YOU RECEIVE NEW VALUES
  
}


void cap_w(float w_in, float w_turn_in, float* lp, float* rp){ //determine w cap first, then cap w_turn based on w

  //Serial.println("at capper");

  /*Serial.println("w input");
  Serial.println(w_in);*/

  float w_mag = abs(w_in);
  float w_turn_mag=abs(w_turn_in);

  float w_chosen;
  float w_turn_chosen;

  float deadzone=0.5; //wheel speed not become zero

  if(w_mag>=w_max){
    w_chosen = (-1+2*sgn(w_in))*w_max;
  }
  else if (w_mag<=w_min){ //lim was 0.5
    
    w_chosen =  (w_mag<w_min*deadzone) ? 0 : (w_min*(-1+2*sgn(w_in))); //for the section lower than w_min, set right of section to w_min and left to zero to minimize error
    
  }
  else{
    w_chosen=w_in;

  }


  //if w+w_turn hit max, limit w_turn to until max

//FLAG


  float w_chosen_mag=abs(w_chosen); 

  if(w_chosen_mag<=0.005){

    if(w_turn_mag>=w_max){
      
      w_turn_chosen = (-1+2*sgn(w_turn_in))*w_max;
      
    }
    
    else if (w_mag<=w_min){ //lim was 0.5
    
      w_turn_chosen =  (w_turn_mag<w_min*deadzone) ? 0 : (w_min*(-1+2*sgn(w_turn_in))); //for the section lower than w_min, set right of section to w_min and left to zero to minimize error
    
    }
    
    else{
      
      w_turn_chosen=w_turn_in;

    }
    
  }

  else if (w_chosen_mag>=w_min || w_chosen_mag<=w_max){

    float turn_max= w_chosen_mag<0.5*(w_max+w_min) ? w_chosen_mag-w_min : w_max-w_chosen_mag; //maximum range of turn wheel speed that's determined by the chosen linear wheel speed;

    w_turn_chosen= (-1+2*sgn(w_turn_in)) * (w_turn_mag>=turn_max ? turn_max :  w_turn_mag);

    
  }

  else{

    w_turn_chosen=0;
    
  }
/*
  Serial.print("w_turn_chosen: ");
  Serial.println(w_turn_chosen);  */


  *lp=w_chosen+w_turn_chosen;
  *rp=w_chosen-w_turn_chosen;

  /*
  Serial.print("w_left: ");
  Serial.println(*lp);
  Serial.print("w_right: ");
  Serial.println(*rp); */
  
  
}


//4*5118*10^-6=0.020472 s.  pi/20=0.157 s     ->0.0887758 s->88.776 ms        




//Set the input change of motors. Set to the period of the motor which is the longest (eg. if have speeds r=2 and l=4, have period as 1/2 (choose r))



//will give out the next step period and set the new w
void one_step(float* w, float w_d, float* stp, float stp_d){


  float stp_value=*stp;
  bool is_goal_zero=abs(w_d)<w_min/2.2;
  bool is_current_zero=abs(*w)<w_min/2.2;

  float period_change=0;

  if(is_current_zero){ //current w is zero and either moving to + or - range, or staying at zero

    *w = is_goal_zero ? 0 : (-1+2*sgn(w_d))*w_min;
    *stp=max_period; 
    
  }

  else if(is_goal_zero || sgn(*w)!=sgn(w_d)){ //if goal is zero or a different sign that current. Move towards 15000 w=0. w=w_min case done by other path

    float diff=max_period-stp_value;

    if(abs(diff)>max_step_period_change){

      stp_value=stp_value+max_step_period_change; //stp_value+max_step_period_change*(-1+2*sgn(diff));
      *stp=stp_value;
      *w=((2*PI)/(steps_per_revolution*stp_value))*(-1+2*sgn(*w));
      
    }
    
    else{ //lower than 900 so round to max_period (w become zero

      stp_value=max_period;
      *stp=stp_value;
      *w=0;
      
    }

    /*Serial.println("updated w");
    Serial.println(*w);*/
    
  }

  else{ //none of the w's is zero and they have the same direction

    float diff=stp_d-stp_value;

    if(abs(diff)>=max_step_period_change){

      stp_value=stp_value+max_step_period_change*(-1+2*sgn(diff)); //+-period step based on difference sign
      *stp=stp_value;
      *w=((2*PI)/(steps_per_revolution*stp_value))*(-1+2*sgn(*w));
      
    }
    
    else{ //lower than 900 so round to max_period (w become zero

      stp_value=stp_d;
      *stp=stp_value;
      *w=w_d;
      
    }
    
  }
  
}


//Sets the desired w inputs and turns them into implementable (capped) right and left motor wheel speed goals
void set_wheel_speeds(float wd, float w_turnd){  //w_turn offset should remain the same for right and left motors  CHANGE TO SET WHEEL SPEEDS

  //Serial.println("updating wheel speeds");

  //MIN IS 4.2,MAX IS 13.4
  //Need to limit w s there.

  //ASSUME WITHIN GOOD OPERATION LIMITS
  //need to set left and right seperately

//add when moving in + direction, left mag needs to be higher for clockwise turn
//for - direction turning, to get clockwise rotation, left mag needs to be lower. ----->add w and w_turn with their signs then abs()
  w_desired=wd; //cap w_desired variables too??
  w_turn_desired=w_turnd; //w turn is offset to w that allows rotation at w_turn to occur in parallel with the linear velocity response
  
  /*Serial.println("computed max");
  Serial.println(computed_max_period*1000000);*/
  
   
   //MAX INCORRECT


  //CHECK CAPPING
  cap_w(w_desired,w_turn_desired,&w_left_desired, &w_right_desired);  //Sets the DESIRED velocities of right and left motor based on the existing input limits
  
  //ALSO CAP W_DESIRED AND W_DESIRED BASED ON CAPPED LEFT AND RIGHT???
  left_step_period_desired = (abs(w_left_desired)<w_min/2.2) ? max_period : (2*PI)/(steps_per_revolution*abs(w_left_desired));  //if w set was zero, period would be infinite, which would block the code. WHEN W IS ZERO CAN CHANGE W ANY TIME 
  right_step_period_desired = (abs(w_right_desired)<w_min/2.2) ? max_period: (2*PI)/(steps_per_revolution*abs(w_right_desired)); //didn't remove sign as while changing the period with step need sign to tell direction and also allow neg to pos step movement

//Make zero's period same as period max to allow similar clock step in turn_motor. RIGHT NOW ASSUME W_MAX TO 0 ACCELERATION CAN BE DONE WITHOUT JITTER. Just make sure no step is done during zero
//Zero is special case, always keep toggle on to ensure changes can come but not call stepper. Call turn motor but only call turnmotor to change period. If desired is zero then keep toggle period zero.

//increase in steps of step_period_change then toggle until clock of desired + 5 clocks have passed (change clock)

  //choose based on largest period step not by largest period. if close can do in one step as steprates are similar so acceleration is low

/*
  Serial.println("w chosen");
  Serial.println(w_right_desired);
  Serial.println("w");
  Serial.println(w_right);
  Serial.println("period desired");
  Serial.println(right_step_period_desired*1000000);
  Serial.println("current period");
  Serial.println(right_step_period*1000000);*/

  int cycles_right;

//if one of the w's is zero, and if none are, have the same sign
  if( (abs(w_right_desired)<w_min/2.2) || (abs(w_right)<w_min/2.2) || sgn(w_right)==sgn(w_right_desired) ){

    cycles_right=(int)(abs(right_step_period_desired-right_step_period)/max_step_period_change) + steady_cycles; // for 500 to 700 operation periods are 500 700*steady (500 done at start to compute +900 or <900 step or to zero step) 
  //CORRECT  FOR POSITIVE AND ZERO INPUT                                                                                                   // for 500 to 1500 periods are 500 1400 1500*steady

//0 to 1 rad->get to 15000 and decrease below to 5000
//1 to 0 rad->go near 15000 from 5000 then switch to zero

//increment if slowing down, decrement if speeding up->decrement when period difference is negative (fast) increment when positive (slow)
//decide on direction in turn_motor
  
  }

//14800->15000 (for same sign) or 14800->0 (for to zero or differing sign). If passing to zero, next shift is zero when higher than max-step

//if non of the w's are zero and have different signs
  else{

  //add up changes from current to zero and zero to desired //ceil adds the cycle of 15000 that arise from 0->15000 switch
    cycles_right=(int)(abs(max_period-right_step_period)/max_step_period_change) + 1 + ceil(abs(right_step_period_desired-max_period)/max_step_period_change) + steady_cycles; //for 13900 to -13900) 13900 14800 0 | 15000 14100 13900*steady
                                                                                                                                                                                                    // ceil(difference/step) + 1 (for zero) + ceil(difference from max/step) + desired*steady                    
  } //CORRECT FOR CURRENT TO ZERO


  int cycles_left; //same logic as half_cycles_right


  if( (abs(w_left_desired)<w_min/2.2) || (abs(w_left)<w_min/2.2) || sgn(w_left)==sgn(w_left_desired) ){

    cycles_left=(int)(abs(left_step_period_desired-left_step_period)/max_step_period_change) + steady_cycles; 

  }

  else{

    cycles_left=(int)(abs(max_period-left_step_period)/max_step_period_change) + 1 + ceil(abs(left_step_period_desired-max_period)/max_step_period_change) + steady_cycles; 

  }

  right_largest=cycles_right>cycles_left;
  cycles = (right_largest ? cycles_right : cycles_left);


  //zero gives correct number of cycles but stepping wrong

  //step periods once to get motors on first period

  one_step(&w_left, w_left_desired, &left_step_period, left_step_period_desired);
  one_step(&w_right, w_right_desired, &right_step_period, right_step_period_desired);

  w=(w_left+w_right)*0.5;
  w_turn=w_left-w;

  //toggle done based on completion of largest step. sensor done for synced faster signal->automatically the nonzero one. NOT DO SENSOR UPDATE BASED ON LARGEST_RIGHT AS IT GIVES THE MOTOR WITH LARGEST STEP. DO ACCORDING TO THE SHORTEST CURRENT PERIOD 
  
} //After very low speeds the continuous assumption of motor fails. THUS WE NEEDS TO HAVE DEADZONE FOR TOO LOW FREQUENCY AND NOT USE LOCAL PERIOD TO DEFINE LOCAL CONTINUOUS SPEED.
  //As a result, distortion caused by the section where w is set to zero has no effect


//accelerates w's to desired w's while also doing the stepping of the motor. Difference from previous motor updater is that periods can be changed based on desired input
void turn_motor(){ //offset does the turn. positive for clockwise turns  //change dir based on sign. Fowward for positive  Would bypass w=0 because we can't implement it due to min_period limits (0 means infinite delay) 

  //Serial.println("turning motors");

  //CHANGE PERIOD ONLY ON ZERO STATE

  float current_time=micros()*0.000001; //1) make state change based on duty, 2) give direction turn switching with zero bypass 3) sensor would use the smallest one 4)implement a turn_motor_off: set w to zero and period inf when off. When on set w to w_min and period no finite number
  bool is_left_zero=abs(w_left)<w_min/2.2;
  bool is_right_zero=abs(w_right)<w_min/2.2;

  //toggle state if the current duty of the left step signal passes. If left is zero, duty is fixed to one, resulting in toggle period being the complete period of zero
  if( (current_time-left_step_time) >= ( left_step_period * ( 1 - ( duty_cycle+right_state*(1-2*duty_cycle) ) *!is_left_zero ) ) ){ //( (current_time-left_step_time) >= ( left_step_period * ( 1-duty_cycle+left_state*(-1+2*duty_cycle) ) * !is_left_zero ) ) { //CAN SKIP STEPS. NOT SEND UPDATES TOO FAST, MAKE W STEPS TOO HIGH

    digitalWrite(left_dir_pin, sgn(w_left)); //HIGH IS FORWARD //PROBLEM FOR LOW SPEEDS; WHEN W IS ZERO SAME SGN RESPONSE GIVEN BUT WE WANT SIGNS TO BE DIFFERENT
    
    left_state = is_left_zero ? HIGH : !left_state; //Not do step if w is zero
    digitalWrite(left_step_pin, left_state); //make 50% duty cycle if duuty seems too small. TOO SMALL AND PULSES NOT HAVE ENOUGH TIME TO RISE TO HIGH STATE. TURN TO 50% DUTY CYCLE
    //Either minimum cycle method or slow rise method (how slow and accurate???)

    //DONE BEFORE W IS UPDATED AND AFTER STEP IS DONE: REDUCE THE CYCLE FROM THE PREVIOUS W
    cycles=cycles-(is_left_zero ? 1 : left_state)*!right_largest;    //remove one if the signal is the one half_cycles uses, subtract another one if previous w was zero (account for zero having no negative part)
    //Didn't removed the period of zeros to have a complete picture readable in code 

    

    //change period if had 2 half cycles passed (went to high or on high) and not reached desired yet
    if(abs(w_left-w_left_desired)>=75*0.000001 && left_state){

      one_step(&w_left, w_left_desired, &left_step_period, left_step_period_desired); //do one step towards the desired period. one_step will determine correct direction & amount & set w accordingly.
    
    }
    
    left_step_time=micros()*0.000001; 
    
  }

  //ZERO LOGIC INCORRECT. Run the same step period as w_min for zero, but close the enable of motors

  if( (current_time-right_step_time) >= ( right_step_period * ( 1 - ( duty_cycle+right_state*(1-2*duty_cycle) ) * !is_right_zero) ) ) {  //duty for positive part, 1-duty for negative part. Still enter conditional for zero to reduce cycle
    
    digitalWrite(right_dir_pin, !sgn(w_right)); //LOW IS FORWARD, that's why sign is flipped

    //disable motors if speed is zero

    right_state = is_right_zero ? HIGH : !right_state; //switched pulse to high when 
    digitalWrite(right_step_pin, right_state);

    cycles=cycles-(is_right_zero ? 1 : right_state)*right_largest;

    //(right_step_period_desired-right_step_period)>=75*0.000001 && right_state); //change period is period not within 75 us of desired period

    if(abs(w_right-w_right_desired)>=75*0.000001 && right_state){ //NEED TO ENSURE WRAP AROUND BEFORE IT'S OK (TO CHANGE DIRECTION GO TOWARDS 15000 FIRST THEN WRAP TO 0 THEN -15000 AND CONTINUE INCREASING)

      one_step(&w_right, w_right_desired, &right_step_period, right_step_period_desired);
      //Serial.println(w_right);  //stepping through the zero point is accurate
     
    }

    //Serial.println(w_right); //won't call when period is close so that works

    right_step_time=micros()*0.000001;
    
  }
  
}
 
//The commands rover will send is done, busy, and need_help. Busy when its performing task or doing critical task like crash fixing. Become available after task is done, and will send need_help after moving away from wall to get laptop to help it refind a path

void setup() {
  Serial.begin(115200);

  pinMode(left_step_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(right_step_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu_setup();
  listen_command=true; //allow website to press start and send commands only after calibration

  digitalWrite(left_step_pin, HIGH);
  digitalWrite(right_step_pin, HIGH);

  
  right_step_time=micros()*0.000001;
  left_step_time=micros()*0.000001;
  pitch_time=micros()*0.000001;
  yaw_time=micros()*0.000001;
  velocity_time=micros()*0.000001;
  sense_time=micros()*0.000001;
  yaw_rate_pid_time=micros()*0.000001;

  
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
  
  xTaskCreatePinnedToCore(
        sendVideoCode, /* Function to implement the task */
        "sendVideo", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &sendVideo,  /* Task handle. */
        0); /* Core where the task should run */
 xTaskCreatePinnedToCore(
        senseTaskCode, /* Function to implement the task */
        "senseTask", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &senseTask,  /* Task handle. */
        0); /* Core where the task should run */
 xTaskCreatePinnedToCore(
        controlTaskCode, /* Function to implement the task */
        "controlTask", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &controlTask,  /* Task handle. */
        1); /* Core where the task should run */


}

void senseTaskCode( void * parameter){
  for(;;){
      if( current_half_period-(current_time-current_step_time)>=sense_runtime*0.9  && current_time-sense_time>=max_sense_period*0.9){ //ensure sensed values are accurately tracked while not adding significant delay to motor times
 
      sense(); //too much sensor calls could distort the slow wave if w_turn exists. Need to limit sensor calls so slow period has less chance of getting distorted PHASE SHIFT???
      //Called to keep pitches and yaws up to date until desired w is updated. Rover is blind during w acceleration.
    
      sense_time=micros()*0.000001;
    
    }
    delay(1);
  }

}

void sendVideoCode( void * parameter) {
  String hexBuffer = "";
  float timeout_time; 
  for(;;) {
    while (SerialPort.available() < 2) { 
      SerialPort.write(0xEE); //flag to indicate to nios that esp is ready to receive frame
      delay(1);
    }
    uint16_t header = SerialPort.read(); // Read the first byte and shift it to the upper byte of the header
    header |= SerialPort.read() << 8; // Read the second byte and combine it with the lower byte of the header
    uint32_t header4x = header * 4;
    //Serial.println(header4x);


    // Read the frame based on the header
    while (SerialPort.available() < (header4x)) { delay(1); }
    uint8_t buffer[header4x];
    SerialPort.readBytes(buffer, header4x);

    // Convert the buffer to a hexadecimal string
    for (int i = header4x-1; i >= 0; i--) {
      char hex[3];
      sprintf(hex, "%02X", buffer[i]);
      hexBuffer += String(hex);
    }
    
    timeout_time = micros()*0.000001;
    while(sensor_states_changing && micros()*0.000001-timeout_time<min_sense_period*3){} //infinite loop to wait until sense_on becomes zero or timeout occurs


    if(state_updated){

      //USE BUFFER VALUES FOR DEAD RECKONING CALCULATION
      /*DEAD RECKONING*/  //ACCURATE ONLY WHEN YAW IS ACCURATE, ROVER PATH ONLY DOES ONE CIRCULAR CURVE, AND SAMPLING TIME IS FAST ENOUGH
      float yaw_diff=(sensor_states.yaw-sensor_states_prev.yaw)%(2*PI); //formulas work with wrapped yaws so need to wrap difference
      int loops=(int)(sensor_states.yaw-sensor_states_prev.yaw/(2*PI)); //do loops in order to get rid of any x difference that's resultant from the rover circling around itself in order to increase accuracy
      float x_diff=sensor_states.x-sensor_states_prev.x;
      float arc_triangle_angle=abs(yaw_diff)>=PI ? abs(yaw_diff) : 2*PI-abs(yaw_diff);
      float arc_radius= abs(yaw_diff)<=0.05 ? x_diff : x_diff/yaw_diff; //if yaw didn't change we don't have an arc but a straight line
      
      float dead_yaw=(sensor_states_prev.yaw+0.5*abs(sensor_states.yaw-sensor_states_prev.yaw)*(-1+2*sgn(sensor_states.yaw-sensor_states_prev.yaw)))%(2*PI) + 2*PI*!sgn(dead_yaw);
      float dead_x_diff= abs(yaw_diff)<=0.05 ? arc_radius : (2*(x_diff-loops*2*PI*arc_radius)*sin(0.5*arc_triangle_angle))/(arc_triangle_angle); 

      int dead_x_diff_int=(int)round(dead_x_diff*10000);



      sensor_states_prev.x=sensor_states.x; //do that after transmission
      sensor_states_prev.yaw=sensor_states.yaw;
      state_updated = false;
      //Is it neccassary to convert dead_yaw from radians to degrees? No, numpy needs radians anyways for sin,cos
      dead_yaw = dead_yaw * 180.0 / PI;
 
      // Convert it to an integer
      int dead_yaw_int = (int)round(dead_yaw);

      // Create a char array to store the formatted string
      char dead_yaw_str[4];

      // Format the string to be 3 characters long, padding with zeros if necessary
      sprintf(dead_yaw_str, "%03d", dead_yaw_int);

      // Append the formatted string to hexBuffer
      hexBuffer += dead_yaw_str;
      
    }
  
    ws.textAll(hexBuffer);
    hexBuffer = "";


  }
}


void toggle_state(){

  listen_command=!listen_command;
  
}


void controlTaskCode( void * parameter) {
  /*SENSOR OFFSETS*/ 
  float gx_offset=0;
  float gy_offset=0;
  float gz_offset=0;
  float ax_offset=0;
  float ay_offset=0;
  float az_offset=0;

  /*TIME FLAGS*/
  float pitch_time; //float due to needing to be in seconds, not milliseconds
  float velocity_time;
  float yaw_time; //DELETE THEM AFTER VERIFYING SENSE()
  float sense_time;
  float right_step_time;
  float left_step_time;
  float yaw_rate_pid_time;
  //float toggle_time;

  /*CONTROL CONSTANTS*/ //control
  float wheel_radius=3.25*0.01; //3.25 cm wheel radius
  float rover_width=7.4*2*0.01; //WIDTH OF ROVER
  int image_width=640; //????????
  float max_turn_angle=75*((2*PI)/360);
  float complementary_weight=0.85;
  float pitch_offset=9.46*((2*PI)/360);
  float k[3]={-8.9443,   55.4896,   15.6862}; //LQR GAINS FOR VELOCITY CONTROLLER. WILL UPDATE THEM TO CALIBRATE THE RESPONSE. 
  //ALSO COEFFICIENTS WERE FOR CONTINUOUS TIME SO WILL DO K VALUES AGAIN FOR DISCRETE TIME. AND MAKE SURE K'S OBEY THE CHOSEN MOTOR BANDWIDTH.
  float v_bandwidth=20; //direction flip of the max w speed is going from max to min peak of the test sine wave. Max motor delay corresponds to half of the bandwidth frequency's period.  //include effects of website to that too!!!
  //kept bandwidth lower than in mind to account for website effects. INCREASE AND DECREASE ACCORDINGLY
  float x_bandwidth=v_bandwidth/3; //lower bandwidth to allow tracking
  float kp_x=x_bandwidth;
  float yaw_rate_bandwidth=v_bandwidth; //Bandwidth of yaw rate controller. CHANGE BASED ON MOTOR BANDWIDTH
  float ki=yaw_rate_bandwidth/(wheel_radius/rover_width);
  float yaw_bandwidth=yaw_rate_bandwidth/3; //lower bandwidth to allow tracking
  float kp_yaw=yaw_bandwidth;
  float yaw_rate_cum_error=0;

  /*LATENCIES*/ //control
  float max_w_delay=(2*PI)/v_bandwidth; //maximum allowed delay between w changes
  int total_samples=2; //2.5ms delay per getEvent call, will lead to around 2.5*3=7.5ms delay for 3 samples each, decrease if too much. ABSOLUTE MIN IT GIVES IS 15.4ms for 3 sample case
  float duty_cycle=0.15; //a min duty cycle needed to ensure pulse has time to rise. 0.01 is good enough
  float max_sense_period=50*0.001; //Max delay latency sensors could handle. Increase or decrease based on quality of state tracking    STEP SIZE LIMITED BY SENSOR LATENCY IF DO PER ONE CLOCK CYCLE
  float sense_runtime=(total_samples*2.5+0.4)*0.001; //shouldn't do sense at start if period too low (fast frequency). will cap the min step period and limit max speed   ADAFRUIT IS BAD AS IT TAKES ALL SAMPLES INSTEAD OF ONE BY ONE THUS USE ELECTRONIC CAT VERSION (MIGHT STILL HAVE DELAY)
  //min sense period+max network delay caps the negative region of the step signal
  float max_wifi_read_delay=1.3*0.000001; //usually in several microseconds. UPDATE AFTER TESTS
  //sensing and command checking are done when we're away from an edge of the 

  //SENSOR LATENCIES COULD BE CHANGED BASED ON MICROSTEPPING, MOVING SENSOR TO CORE 0, ETC.
  int resolution=2;
  float steps_per_revolution = 200 * resolution; //will determine the lowest and fastest frequencies we get. But absolute max and min have no direct relationship with it. Find with experiment
  float max_step_period_change=(resolution>=2) ? 3250*0.000001 : 1000*0.000001; //do steps of 1800 micresecond while going from one w to another. CAN I DO LARGER STEPS WHEN AT LOWER SPEEDS??? TEST (W ACCELERATION DEPENDS ON MAX TORQUE GIVEN BY MOTOR AND LOAD WEIGHT)
  float absolute_max_period= (resolution>=2) ? 14000*0.000001 : 13000*0.000001;
  float absolute_min_period= (resolution>=2) ? 800*0.000001 : 1800*0.000001; //min delay incorrect
  float min_period=( absolute_min_period*(1-duty_cycle) >= (sense_runtime+max_wifi_read_delay) ) ? absolute_min_period : (sense_runtime+max_wifi_read_delay)*(1/(1-duty_cycle))*1.01; //either the lowest frequency found from tests or the one limited by delays. *1.05 added to give slack

  //from aritmethic sum inequality. Aritmethic sum from min to max in 1800 steps needs to be below half of maximum allowed delay
  float computed_max_period=16.2*0.001;//(max_step_period_change/2)*(-1+sqrt(1+(4/max_step_period_change)*(PI/v_bandwidth + min_period/max_step_period_change))); 
  // Current give w_min as 2.415 and w_max as 4.987


  float max_period=min(absolute_max_period, computed_max_period); //choose based on maximum desired delay determined by v bandwidth //put in max setter method if needed to change bandwidth much
  //Divide 157 by 2 to get upper limit of the aritmethic sum from 2338 to max. Aritmethic sum is S=n*(avg of min+step*n and min). n=(max-min)/step       floor
  float w_max=(2*PI)/(steps_per_revolution*min_period); //PERIOD CAN'T BE LOWER THAN 
  float w_min=(2*PI)/(steps_per_revolution*max_period);

  /*FIXED MOTOR SPEED VALUES*/ //The speeds I predetermined to be used in operations 
  float v_normal=(w_min+w_max)*0.5*wheel_radius; //average of v range. CAN DECREASE IF TOO FAST FOR IMAGE PROCESSOR OR MAKE TOO MUCH DRIFT
  float v_slow=((w_min+w_max)*0.5+w_min)*0.5*wheel_radius; //lower if still too high
  float yaw_rate_moving_normal=(v_slow/wheel_radius-w_min)*0.5*(wheel_radius/rover_width); //???
  float yaw_rate_moving_slow=(v_slow/wheel_radius-w_min)*0.5*(wheel_radius/rover_width); //??? Not too sure about accuracy 
  float yaw_rate_stat_normal=(w_min+w_max)*0.5*(wheel_radius/rover_width); //???
  float yaw_rate_stat_slow=(w_min)*(wheel_radius/rover_width);

  /*MOTOR SPEED VALUES*/ //They're inputs to motors 
  float w_left=0;
  float w_left_desired=0;
  float w_right=0; //w and w_turn are desired inputs, w_right and w_left are clamped w's given to left and right motors based on operation constraints
  float w_right_desired=0;
  float w=0; //after getting w_left and w_right. Get w_linear from average of left and right. Its sign also shows the direction of movement
  float w_desired=0;
  float w_turn=0;
  float w_turn_desired=0;


  /*MOTOR SPEED CONTROL VARIABLES*/
  float left_step_period=max_period;
  float left_step_period_desired=0;
  float right_step_period=max_period;
  float right_step_period_desired=0;
  //float toggle_period=steps_per_revolution*max_period; //not toggle until all motors got to steady state
  int cycles=0; //min half pulses needs to be done before w switch to keep operation smooth
  bool right_largest=true; //Tells which motor signal undergoes the longest acceleration. Half_cycles set and changed based on that signal. Left for false right for true.
  int steady_cycles=5; //number of cycles you want motor to stay on steady state for. INCREASE IF JITTER
  bool left_state=true; //Value of the corresponding step pin. Start from true so positive edge occurs after desired period passes, preventing phase shifts
  bool right_state=true; 

  bool doing_yaw_compensation=false;
  float compensated_yaw_offset=0;
  int yaw_compensation_samples=0;

  float input;
  float* output;
  float past_output;
  bool control_v=true;
  bool control_yaw_rate=true;
  int operations_left=0;


  struct operation{
    int id;
    float input;
  };

  bool start_operation=false;

  operation[4] operations; //HAVE FIXED SIZE OF MAX OPERATIONS. 3 IS THE MAXIMUM AMOUNT OF OPERATIONS WE HAVE
  int i=0; //save commands as structs of ids and inputs to an operation array

  
  for(;;) {

    else{ //IF RECEIVED A COMMAND and currently executing its operations. SHOULD ONLY ENTER WHEN DOING A COMMAND


      if(start_operation){ //if need to load a new operation

        if(i==4){

          listen_command_changing=true;
          toggle_state();
          listen_command_changing=false;
          
        }
          

        else{ //coperation executor
          
          switch(operations[i].id){

            case 0: //END

              listen_command_changing=true;
              toggle_state();
              listen_command_changing=false;
          
            break;
            case 1: //set_v

              control_v=true;
              control_yaw=true;
              output=&v;
              past_output=*output;
              set_velocity(operations[i].input);
          
            break;
            case 2: //set_x DON'T UPDATE X ATM, CHANGE BASED ON FINAL OPERATION

              control_v=false;
              control_yaw=true;
              output=&x;
              past_output=*output;
              set_distance(operations[i].input);
            
            break;
            case 3: //set_yaw_rate

              control_v=true;
              control_yaw=false;
              output=&yaw_rate;
              past_output=*output;
              set_yaw_rate(operations[i].input);
            
            break;
            case 4: //set_yaw

              control_v=true;
              control_yaw=true;
              output=&yaw;
              past_output=*output;
              set_yaw(operations[i].input);
          
            break;
            default: //if command not valid do nothing and wait for new command
              listen_command_changing=true;
              toggle_state();
              listen_command_changing=false;
              //TRANSMIT INVALID COMMAND ERROR TO LAPTOP?
          
            break;

          
          } //switch end

        }
        start_operation=false;
        
      } //start command end

      else{

        //Send complete and change state when close to input and stable (output close to steady state value and doesn't vary much)
      //Also switch to the use of v and yaw rate controllers since these controllers would be the ones used during listening mode (while doing straighten) (Might switch to yaw instead of yaw rate controller in final design?)
        if(abs(*output-operations[i].input)<0.1 && (*output-past_output)<0.3){  //WILL CHANGE MARGINS BSED ON MPU TEST ACCURACY

          i++;
          start_operation=true;
        
          control_v=true;
          control_yaw=true;
          //Laptop will see that command is done when it receives the rover's changed state along with the image data
        
        }

      //Change state and keep updating the controller variable.
      past_output=*output;

      //The else block didn't listen to commands, so the busy state is completely deaf to laptop commands
        
      }

      

    } 

    /*END OF COMMAND LISTENER AND EXECUTIONER*/

    //update corresponding controlled variable (free mode only update velocity and yaw, busy mode update input parameters)

    /*START OF SENSOR*/ //the if statement to ensure start won't disturb an edge. Not perfect but does the job. MIGHT NOT NEEDED WHEN MULTICORE
    float current_time=micros()*0.000001;
    float current_right_half_period=right_step_period*(1-!(abs(w_right)<w_min/2.2)*(duty_cycle-(2*duty_cycle-1)*right_state));    //right_step_period*(1-duty_cycle+right_state*(-1+2*duty_cycle))*!(abs(w_right)<w_min/2.2);
    float current_left_half_period=left_step_period*(1-!(abs(w_left)<w_min/2.2)*(duty_cycle-(2*duty_cycle-1)*left_state));  //left_step_period*(1-duty_cycle+left_state*(-1+2*duty_cycle))*!(abs(w_left)<w_min/2.2);
    bool fastest_right=abs(w_right)>=abs(w_left);
    float current_half_period = fastest_right ? current_right_half_period : current_left_half_period;
    float current_step_time= fastest_right ? right_step_time : left_step_time;

    
    //DO A SENSE IF NOT NEAR EDGES AND SENSE_PERIOD HAS PASSES. REDUCED SLACK TO ALLOW FAST VELOCITIES TO BE SENSED. ELSE THERE'S NO SENSE PERIOD AVAILABLE TO THEM. ALLOWED SLIGHT DISTORTION
    if( current_half_period-(current_time-current_step_time)>=sense_runtime*0.9  && current_time-sense_time>=max_sense_period*0.9){ //ensure sensed values are accurately tracked while not adding significant delay to motor times
 
      sense(); //too much sensor calls could distort the slow wave if w_turn exists. Need to limit sensor calls so slow period has less chance of getting distorted PHASE SHIFT???
      //Called to keep pitches and yaws up to date until desired w is updated. Rover is blind during w acceleration.
    
      sense_time=micros()*0.000001;
    
    }

    /*END OF SENSOR*/


    /*START OF MOTOR*/ //will make use of the global variable cycles to keep track of toggling. ASSIGN MOTOR 
    if(cycles<=0){ //this is where the desired controllers would be run when we're done with w acceleration

    //Serial.println("updating velocity");

    digitalWrite(left_step_pin, LOW);
    digitalWrite(right_step_pin, LOW); //reset both signals to 0 phase to ensure any phase shifts done on the slow signal due to delays isn't carried over.

    control_v ? update_velocity() : update_distance();
    control_yaw ? update_yaw() : update_yaw_rate(); //run the control loop and get the new motor w's, cap them, then set the new step periods and toggle accordingly

    //If delay too high, we can't have steps with short periods->can cause jitter and speed reduction

    //Serial.println("velocity updated");
    //toggle_time=micros()*0.000001;
    
    }


    turn_motor();

    /*END OF MOTOR*/
    
    delay(0);
 
  }
}
