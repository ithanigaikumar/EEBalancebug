#define left_dir_pin 27
#define left_step_pin 26
#define right_dir_pin 25
#define right_step_pin 33
#define steps_per_revolution 200 //will determine the lowest and fastest frequencies we get. But absolute max and min have no direct relationship with it. Find with experiment

#define sgn(val) ((val) >= 0 ? HIGH : LOW)

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h> //Library automatically setups the sensor pins due to SPI based communication. USES power pins and D3 for SCL and D4 for SDA (use these pins because they're SPI pins?)
//#include "MPU6050_6Axis_MotionApps20.h     //COPY PASTE ENTIRE CODE
#include <Wire.h>

//FPGA and Wireless stuff

const char* ssid = "esp32aptestrle";
const char* password = "wasdqwerty32";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // create WebSocket instance

HardwareSerial SerialPort(2);

TaskHandle_t sendVideo;
TaskHandle_t senseTask;
TaskHandle_t controlTask;

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    // client connected
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if(type == WS_EVT_DISCONNECT){
    // client disconnected
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

//Control stuff

//delay is 2.5m per sample call, FPGA only delay of yaw is 0.14ms after reduction to 16 bits of data (not float but an int). Max motor delay is 67.7 ms for 2500 to 7500 gradual transition (5 clock in steady state, increase if problem)
//70.1ms for 10000 max period, 140.6ms for 15000 max period (5 clocks in steady state)


Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp; //INCREASE THE W RANGES TO REACT TO LARGE AND SLOW DISTURBANCES BETTER 

/*TIME FLAGS*/
float pitch_time; //float due to needing to be in seconds, not milliseconds
float velocity_time;
float yaw_time; //DELETE THEM AFTER VERIFYING SENSE()
float sense_time;
float right_step_time;
float left_step_time;
float yaw_rate_pid_time;
//float toggle_time;

//do sensing on low part of the cycle, change high period based on rise time

//straighten will set yaw rates then motors would be updated accordingly. NO STRAIGHTEN OR ANY W UPDATE CALLS UNTIL TRANSIENT FOR THAT MOTOR IS OVER

//absolute min is 2338 and absolute min is 15000

/*CONTROL CONSTANTS*/
float wheel_radius=3.25*0.01; //3.25 cm wheel radius
float rover_width=7.4*2*0.01; //WIDTH OF ROVER
float complementary_weight=0.85;
float pitch_offset=11.35*((2*PI)/360);
float k[4]={-1800,  -67,  814.6, 30.2}; //LQR GAINS FOR VELOCITY CONTROLLER. WILL UPDATE THEM TO CALIBRATE THE RESPONSE. 
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


//website delays would likely to limit the max speed of the rover if shown to be too much. TEST WEBSITE LISTENING AND COMMAND DECODING SPEED AND ADJUST DELAY AS SENSOR DELAY+WEBSITE DELAY AND CHANGE MAXIMUM SPEED ACCORDINGLY

//for 20 bandwidth, max time for wmax to -wmax switch is 157 ms. For 2.338m based min_period, get max_period as 11.338ms. For 5.4m based min period, get max_period as 11.7ms
//approximate as cumulative sum from min to max periods with 900 step, then multiply by 2, max delay for 900 step for 2500 to 15000 min and max is 273 ms

/*LATENCIES*/
float max_w_delay=(2*PI)/v_bandwidth; //maximum allowed delay between w changes
int total_samples=2; //2.5ms delay per getEvent call, will lead to around 2.5*3=7.5ms delay for 3 samples each, decrease if too much. ABSOLUTE MIN IT GIVES IS 15.4ms for 3 sample case
float duty_cycle=0.01; //a min duty cycle needed to ensure pulse has time to rise. 0.01 is good enough
float max_sense_period=100*0.001; //Max delay latency sensors could handle. Increase or decrease based on quality of state tracking    STEP SIZE LIMITED BY SENSOR LATENCY IF DO PER ONE CLOCK CYCLE
float sense_runtime=(total_samples*2.5+0.4)*0.001; //shouldn't do sense at start if period too low (fast frequency). will cap the min step period and limit max speed
//min sense period+max network delay caps the negative region of the step signal
float max_wifi_read_delay=1.3*0.000001; //usually in several microseconds. UPDATE AFTER TESTS
//sensing and command checking are done when we're away from an edge of the 

//SENSOR LATENCIES COULD BE CHANGED IF YAW IS FOUND WITH FPGA AND SUCH
float max_step_period_change=1800*0.000001; //do steps of 1800 micresecond while going from one w to another. CAN I DO LARGER STEPS WHEN AT LOWER SPEEDS??? TEST (W ACCELERATION DEPENDS ON MAX TORQUE GIVEN BY MOTOR AND LOAD WEIGHT)
float absolute_max_period=15000*0.000001;
float absolute_min_period=2338*0.000001; //min delay incorrect
float min_period=( (2338*0.000001)*(1-duty_cycle) >= (sense_runtime+max_wifi_read_delay) ) ? 2338*0.000001 : (sense_runtime+max_wifi_read_delay)*(1/(1-duty_cycle))*1.01; //either the lowest frequency found from tests or the one limited by delays. *1.05 added to give slack

//from aritmethic sum inequality. Aritmethic sum from min to max in 1800 steps needs to be below half of maximum allowed delay
float computed_max_period=16.2*0.001;//(max_step_period_change/2)*(-1+sqrt(1+(4/max_step_period_change)*(PI/v_bandwidth + min_period/max_step_period_change))); //CALC NOT COMPLETE. DO IT!!!! //floor(sqrt(pow(min_period,2)+max_step_period_change*max_w_delay));
//CALC NOT COMPLETE OR CORRECT. UPDATE!!!   Current give w_min as 2.415 and w_max as 4.987


float max_period=min(absolute_max_period, computed_max_period); //choose based on maximum desired delay determined by v bandwidth //put in max setter method if needed to change bandwidth much
//Divide 157 by 2 to get upper limit of the aritmethic sum from 2338 to max. Aritmethic sum is S=n*(avg of min+step*n and min). n=(max-min)/step       floor
float w_max=(2*PI)/(steps_per_revolution*min_period); //PERIOD CAN'T BE LOWER THAN 
float w_min=(2*PI)/(steps_per_revolution*max_period); 

/*STATES*/
float pitch=0; //pitch of the rover
float pitch_rate=0; //pitch rate of the rover
float pitch_rate_prev=0;
float v=0; //velocity of rover
float v_prev=0;
float x=0; //distance rover traveled
float yaw=0; //previous v recorded in previous sense call. Made according to w value used during that time
float yaw_rate=0;
float yaw_rate_prev=0;

//INPUTS TO METHODS //angular velocity of the wheels. OUTPUTS OF THE ESP32  //SETUP THEM TO THE MINIMUM SPEED BECAUSE W=0 IS INF PERIOD WHICH CAN SOFTLOCK THE SYSTEM. IF NEEDED, HAVE METHOD TO GET ALL METHODS TO ZERO

/*MOTOR SPEED VALUES*/
float w_left=w_min;
float w_left_desired=w_min;
float w_right=w_min; //w and w_turn are desired inputs, w_right and w_left are clamped w's given to left and right motors based on operation constraints
float w_right_desired=w_min;
float w=w_min; //after getting w_left and w_right. Get w_linear from average of left and right. Its sign also shows the direction of movement
float w_desired=w_min;
float w_turn=0;
float w_turn_desired=0;

/*MOTOR SPEED CONTROL VARIABLES*/
float left_step_period=steps_per_revolution*max_period;
float left_step_period_desired=steps_per_revolution*max_period;
float right_step_period=steps_per_revolution*max_period;
float right_step_period_desired=steps_per_revolution*max_period;
//float toggle_period=steps_per_revolution*max_period; //not toggle until all motors got to steady state
int cycles=0; //min half pulses needs to be done before w switch to keep operation smooth
bool right_largest=true; //Tells which motor signal undergoes the longest acceleration. Half_cycles set and changed based on that signal. Left for false right for true.
int steady_cycles=4; //number of cycles you want motor to stay on steady state for.
bool left_state=true; //Value of the corresponding step pin. Start from true so positive edge occurs after desired period passes, preventing phase shifts
bool right_state=true; 

//sensor only updated when we would update desired w (set that based on sensed current states). DO SENSING AFTER TIME HAS PASSED BUT IF TOOK LONGER THAN 100ms DO SENSING TO KEEP GOOD TRACK
//But not do tracking if one of motor periods too low? or it would cause stalling

//LAPTOP INPUTS
float yaw_desired=0; //the actual yaw we'll try to change to. Change during turns
float yaw_rate_desired=0;
float v_desired=0; //set by laptop and methods
float x_desired=0; //set by methods and only used for crash()
int vanish=0; //pixel distance between vanishing point and image center sent by laptop. Laptop will add disturbance to it and the controller will try to keep it zero by yaw rate control

//MULTITHREADING FLAGS
static SemaphoreHandle_t sense_mutex;//indicate sense() is running so image broadcaster will wait until yaw and v are updated
bool state_changing=false;


/*TASK HANDLE*/
TaskHandle_t control_handle; //handle for control task





void mpu_setup() {

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu.setTemperatureStandby(true);
  
}




void sense_pitch_pitchrate(){ //senses velocity, pitch, and yaw
  //YAW SIGNIFICANTLY DRIFTS DUE TO NONZERO NOISE, FIX BY REJECTING LESS THAN 0.05 YAW_RATES (NOISES)

  //gyro hpf, accelerometer lpf
  

  //2 ways to get q. Pick whichever has leff drift and noise. (can fix direct measurement noise by moving filter on all)

  float new_pitch_rate=0; //pitch drifts a lot, figure out why?
  float gyro_pitch=0;
  float acc_pitch=0; //5 plot points until drop for 150ms delay

  float pitch_time=0;
  float dt=0;

  for(int i=0; i<total_samples; i++){ //put pitch measurement in loop too if shown to be too noisy for concise use. CAN REMOVE LOOP IF MPU TEST SHOWS PITCH RATE TO NOT BE NOISY
    pitch_time=micros()*0.000001; //NOT CORRECT
    mpu.getEvent(&a, &g, &temp);
    dt+=micros()*0.000001-pitch_time;
    new_pitch_rate += g.gyro.y;
    acc_pitch += atan2(a.acceleration.z, sqrt(pow(a.acceleration.y,2) + pow(a.acceleration.x,2)));   //IF ACC IS USED ONLY. ONLY GOOD FOR STATIC FORCE
  }

  pitch_rate=((abs(new_pitch_rate/total_samples)>=0.08) ? new_pitch_rate/total_samples : 0); //Added hysteresis to prevent drift //VALUE IN RAD, TRANSMISSION AND INPUT IN DEGREES
  gyro_pitch = pitch + pitch_rate * dt/total_samples;    //IF GYRO IS USED ONLY, ONLY GOOD FOR DYNAMIC FORCE (ACCELERATION DIE OUT AND NOT INTERFERE WITH GRAVITY)
  //Serial.print("gyro pitch: ");
  //Serial.println(gyro_pitch*(360/(2*PI)));
  acc_pitch/=total_samples;
   //ADD OFFSET TO GET PITCH FOR CENTER OF MASS INSTEAD FOR ACTUAL PITCH OF SENSOR
  //SWITCHES X AND Z PLACES TO GET VERTICAL TO 0 AND HORIZONTAL TO 90
  //Serial.print("acc pitch: ");
  //Serial.println(acc_pitch*(360/(2*PI)));
  
  //FORMULA IS THE RESULT OF COMBINING THE LPF GYRO AND HPF ACC FILTERS. 
  pitch = (1-complementary_weight) * gyro_pitch + complementary_weight * acc_pitch;  //Not sure if 100% correct. Basically how internet shows complementary filter is done
 
  
  /*
   * Kalman filters are too computation heavy for Arduino, so should stick with either using complementary filter and averaging
   * For rates, our only method is direct measurement of gyroscope, which we can reduce noise through moving average
   */

  //q=(new_pitch-pitch)/dt;    other method if averaging method gives too much delay

  //ACCELEROMETER INTEGRATION GIVES DRIFT TOO, KALMAN ONLY ON PITCH. 
  //KALMAN USED BECAUSE GYRO CAN'T TELL GRAVITY FROM ACCELERATION. LPF FILTERS OUT EFFECT OF ACCELERATION. ACCELERATION CAN TELL YOU PITCH FOR LOW BANDWIDTH (geometry of force tell pitch)
  
}




void sense_yaw_yawrate(){ //CHANGE YAW_RATE MEASUREMENT BASED ON SYSTEM DEFINED

  

  //gyro hpf, accelerometer lpf


  float new_yaw_rate=0;
  float yaw_time=0; //change to micros if deemed not fine enough
  float dt=0;

  for(int i=0; i<total_samples; i++){  //in future do only pitch_rate if would only do the FPGA based measurement (DOWNSIDE IS IT WOULD TAKE LONGER TIME TO DO DUE TO SERIAL)

    yaw_time=micros()*0.000001;
    mpu.getEvent(&a, &g, &temp); //will optimize in the future where only gyro or accelero is called at a time instead of measuring both at each call
    Serial.print("gx yaw rate: ");
    Serial.println(-g.gyro.x/cos(pitch)); //Used angular velocity vectors.
    Serial.print("gz yaw rate: ");
    Serial.println(-g.gyro.z/cos(90-pitch));
    dt+=micros()*0.000001-yaw_time;
    new_yaw_rate+=-g.gyro.x/cos(pitch);  //subtract by roll*sin(pitch) if can find roll. gx and gz detect roll and yaw TOGETHER and CAN'T RECTIFY THAT. Could find roll from accelerometer then rollrate from dt???
    //roll interference and drift are the main protractors of accurate yaw measurement
    //NEED TO DO PITCH SENSE BEFORE DOING YAW SENSE->TOGETHER. Also do velocity together as it's a quick measurement
    
  }

  yaw_rate=((abs(new_yaw_rate/total_samples)>=0.15) ? -new_yaw_rate/total_samples : 0); //invert sign if positive is counterclockwise (DO TESTS). REMOVE LOOP IF ONE MEASUREMENT IS NOT NOISY

  yaw=sense_yaw_sensor(dt/total_samples);

  yaw_rate_prev=yaw_rate;
  
}

//on FPGA we will get 2 bytes as an int, then divide by 3 to get float. Best decimal quality is 3 decimal places

//chose 0-360 range to have universal direction no matter yaw. Laptop can set exact yaw instead of getting confused




float sense_yaw_sensor(float dt){

  return (yaw + 0.5*(yaw_rate+yaw_rate_prev) * dt); //Removed %360 to stop wrapping. The SENDER NEEDS TO WRAP IN TURN AND WE NEED TO SCALE THE INPUT

  //Same logic done as x dead reckoning
  
}




void sense_velocity_distance(){ //make dt finding better as delays could increase dt much and make dead reckoning inaccurate

  //assume you went from last x to new x in velocity you measured now. Can also record previous velocity and use their average to make v more accurate? 

  float dt=micros()*0.000001-velocity_time;

  //Can't use accelerometer for velocity due to drift, so used wheel counting instead. COMPLEMENTARY FILTER WITH IT IF SHOWN TO BE INACCURATE????
  
  v= w * wheel_radius; //ONLY IF NO SLIP.     Haven't decided if I need to implement an upper limit to velocity
  x=x+0.5*(v+v_prev)*dt; //0.5*(v-v_prev)*dt made a trapezoid that minimized the area error of v graph. give dx, which is then added.
  //CAN'T IMPROVE WITH QUADRATIC FIT AS DON'T KNOW HOW V EVOLVED. more concave and dt->more error
  //Can also do middle riemann sum?
  v_prev=v;
  velocity_time=micros()*0.000001;
  
}

void senseTaskCode( void * parameter){
  for(;;){
      if( current_half_period-(current_time-current_step_time)>=sense_runtime*0.9  && current_time-sense_time>=max_sense_period*0.9){ //ensure sensed values are accurately tracked while not adding significant delay to motor times
 
         sense(); //too much sensor calls could distort the slow wave if w_turn exists. Need to limit sensor calls so slow period has less chance of getting distorted PHASE SHIFT???
      //Called to keep pitches and yaws up to date until desired w is updated. Rover is blind during w acceleration.
    
      sense_time=micros()*0.000001;
      vTaskDelay(50 / portTICK_PERIOD_MS);
         
    
    }
    
  }

}





void sense(){ //do measurement only 2 times and use data to find pitch, yaw, and v all at the same time

  xSemaphoreTake(sense_mutex, 2);

  float dt_start=micros()*0.000001-sense_time; //one dt is that recorded in start, other is one recorded after one sampling time run
  float dt=0;
  float ax=0;
  float ay=0;
  float az=0;
  float gx=0;
  float gy=0;
  //velocity and distance are out of loop because v is directly given by w, which is not changed during sense period (ensured that)

  //need stable pitch to measure yaw rate, so measure desired gyros first

  //get accurate rate samples
  for(int i=0; i<total_samples; i++){  //in future do only pitch_rate if would only do the FPGA based measurement (DOWNSIDE IS IT WOULD TAKE LONGER TIME TO DO DUE TO SERIAL)

    mpu.getEvent(&a, &g, &temp); //will optimize in the future where only gyro or accelero is called at a time instead of measuring both at each call
    dt+=(micros()*0.000001-sense_time);

    ax+=a.acceleration.x;
    ay+=a.acceleration.y;
    az+=a.acceleration.z;

    gx+=g.gyro.x;
    gy+=g.gyro.y;
    
  }

  pitch_rate=((abs(gy/total_samples)>=0.08) ? gy/total_samples : 0); //Added hysteresis to prevent drift //VALUE IN RAD, TRANSMISSION AND INPUT IN DEGREES
  
  float acc_pitch = atan2(az/total_samples, sqrt(pow(ay/total_samples,2) + pow(ax/total_samples,2)));
  //ADD OFFSET TO GET PITCH FOR CENTER OF MASS INSTEAD FOR ACTUAL PITCH OF SENSOR
  //SWITCHES X AND Z PLACES TO GET VERTICAL TO 0 AND HORIZONTAL TO 90
  //Serial.print("acc pitch: ");
  //Serial.println(acc_pitch*(360/(2*PI)));
  float gyro_pitch = pitch + 0.5*(pitch_rate_prev+pitch_rate) * dt/total_samples;    //IF GYRO IS USED ONLY, ONLY GOOD FOR DYNAMIC FORCE (ACCELERATION DIE OUT AND NOT INTERFERE WITH GRAVITY)
  pitch_rate_prev=pitch_rate;
  
  //Serial.print("gyro pitch: ");
  //Serial.println(gyro_pitch*(360/(2*PI)));
  pitch = ((1-complementary_weight) * gyro_pitch + complementary_weight * acc_pitch)-pitch_offset;
  

  yaw_rate=((abs((-gx/cos(pitch))/total_samples)>=0.10) ? (-gx/cos(pitch))/total_samples : 0);
  //subtract by roll*sin(pitch) if can find roll. gx and gz detect roll and yaw TOGETHER and CAN'T RECTIFY THAT. Could find roll from accelerometer then rollrate from dt???
  //roll interference and drift are the main protractors of accurate yaw measurement. MEASURE ROLL INTERFERENCE TOO SOMEHOW???
  //NEED TO DO PITCH SENSE BEFORE DOING YAW SENSE->TOGETHER. Also do velocity together as it's a quick measurement

  yaw=sense_yaw_sensor(dt/total_samples);
  yaw_rate_prev=yaw_rate;

  v = w * wheel_radius; //directly set by input so no need to stabilize
  x=x+0.5*(v+v_prev)*dt;
  v_prev=v;
  
  xSemaphoreGive(sense_mutex);
}


//SENSORS ARE FIXED AND UPGRADED FOR MORE ACCURACY
//DO THAT, MULTITHREADING WITH SHREYA, 
//WRITE STRAIGHTEN (currently await data from laptop, might switch to hybrid system if shown to have too much delay), CRASH, FPGA (float to int in yaw, get 16 bit for vanish (-320 to 320 is range, need 10 bits (int)) , AND WIFI STUFF WITH SHREYA






float gain=7.8/28; //will adjust to get accurate q to torque transfer that accomodates input voltage effects and load variances (speed torque graph only for max torque so can't directly use it)

//GONNA USE IT IN MATLAB CODE ONLY, WILL USE W DIRECTLY AS THE INPUT OF THE MOTORS
//Maybe use it to determine period_step needed based on acceleration needed?
float w_to_torque(float w){

return ((w<=(2*PI*67)/60.0) ? (0.162) : (0.172-((60.0*w)/(2*PI))))*gain; //accurate up to 400 rpm

}

//NEED TO CONTINUOUSLY CALL UPDATES() FOR CONTROLLER TO UPDATE RESPONSE. FOR BOTH PID AND LQR. 




void set_velocity(float vd){

  v_desired=vd;

  //SET_VELOCITY IS A COMMAND SEND BY LAPTOP

}

//Have basic constants so start testing. More overshoot a state does increase its cost

//PARAMETERS HEAVILY PRONE TO CHANGE




void update_velocity(){ //vd=v, x can change

  //DO THE CONTROLLER STUFF THERE
  /*
   * get feedback from sensors
   * scale them with optimal gain coefficients to get actual input to subtract from. 
   * apply new input to system by setting new w_desired for turns
   * 
   */

   //FIND W UPDATE THEN CALLUPDATE WHEEL SPEEDS TO GIVE RESPONSE. WOULD ALSO UPDATE TURN AND BALANCING IF NECESSARY

   //Serial.println("at velocity controller");
    
   float w_feed=k[0]*x+k[1]*(v_desired-v)+k[2]*pitch+k[3]*pitch_rate; //v_desired/wheel_radius;       //created new variable for readibility only

   //Serial.print("w_feed: ");
   //Serial.println(w_feed); 

   set_wheel_speeds(w_feed, w_turn); //w_turn didn't change for velocity control as we're not turning.
   
  
}



//USED THE SAME CONTROLLER AS YAW_RATE DUE TO BEING THE SAME PROBLEM

//LQR CONTROLLER ALSO CONTROLS X STATE, SO NOT SURE IF SHOULD NEED SEPERATE CONTROLLER FOR X, MIGHT IMPLEMENT THE LQR CONTROLLER FOR X AGAIN

void set_distance(float x_d){

   x_desired=x_d;

   //SET DISTANCE IS A COMMAND SEND BY LAPTOP
  
}






void update_distance(){ //yaw_d=yaw, yaw_rate can change

//https://www.teachmemicro.com/arduino-pid-control-tutorial/ 

  float error=x_desired-x;

  float new_v=kp_x*error; //PID OUTPUT
  set_velocity(new_v);
  update_velocity();

  //Did a P controller because found closed loop response as R/(s+R), where inner loop was seen as unity and G=1/s. Could control bandwidth easily with a P controller
  
}





void set_yaw_rate(float rate_d){ //yaw_rate_d=yaw_rate, desired_yaw not used

  yaw_rate_desired=rate_d;
  //implement velocity controller here, will set w and yaw rate desired at the end

}

//Made sure yaw_rate stayed at actual value (rather than giving w_turn input directly and hoping for the best) through a PID controller.
//G was only the constant radius/width, thus just did an integrator with gain u=crossover/G to control yaw_rate






void update_yaw_rate(){ //yaw_d=yaw, desired_yaw_rate can change

  float dt=millis()*0.001-yaw_rate_pid_time;
  
  float error=yaw_rate_desired-yaw_rate;
  yaw_rate_cum_error+=error*dt; //integration of error

  float w_feed=ki*yaw_rate_cum_error; //PID OUTPUT

  set_wheel_speeds(w, w_feed);

  yaw_rate_pid_time=millis()*0.001;

  //Used an I controller because closed loop response was GR/(1+GR) where G was a constant. It gave response (G*ki)/(s+G*ki), so ki can directly control bandwidth.

}
//YAW SETTING IS STILL WORK IN PROGRESS

//sensor set according to set bandwidth




  
void set_yaw(float yaw_d){ //Assume given yaw is in rev 0. Offset the given yaw input to the rev that makes the difference between yaw and it the smallest


   int rev=yaw/(2*PI) + !sgn(yaw)*-1; //get the revolution yaw value is at. Pass 360 you get one rotation pass 0 you lose one rotation    -360 : -1 -1 | 0 : 359  0 | 360 : 719 1

  //chose rev rev-1 or rev+1 based on closest 

  int offset=0;
  
  float mag_yaw_difference=abs((yaw_d+2*PI*rev)-yaw);
  
  if(abs((yaw_d+2*PI*(rev+1))-yaw)>=mag_yaw_difference){

    mag_yaw_difference=abs(yaw_d*(rev+1)-yaw);
    offset=1;
    
  }

  if(abs((yaw_d+2*PI*(rev-1))-yaw)>=mag_yaw_difference){

    mag_yaw_difference=abs(yaw_d*(rev-1)-yaw);
    offset=-1;
    
  }
  

   yaw_desired=yaw_d+2*PI*(rev+offset); //input is scaled by the amount of full turns the rover already did from the start.

   //IS A COMMAND SENT BY LAPTOP, BUT WOULD GET DEGREES DATA THAT HASN'T BEEN UNWRAPPED. CONVERT TO RADS FIRST THEN GIVE TO SET_YAW TO BE UNWRAPPED OPTIMALLY
  
}



//AS MENTIONED ABOVE, HAS A HUGE PROBLEM I'LL SOLVE
void update_yaw(){ //yaw_d=yaw, desired_yaw_rate can change

//https://www.teachmemicro.com/arduino-pid-control-tutorial/ tutorial on Arduino PID controllers

  sense_yaw_yawrate();  

  float error=yaw_desired-yaw;

  float new_yaw_rate=kp_yaw*error; //PID OUTPUT
  set_yaw_rate(new_yaw_rate);
  update_yaw_rate();

  //Did a P controller because found closed loop response as R/(s+R), where inner loop was seen as unity and G=1/s. Could control bandwidth easily with a P controller
  
}

//high current more jitter but faster, low current slower but less jitter (jitter is from high acceleration)
//CURRENT CAN'T CHANGE STEPSPERREV AS IT'S A MECHANICAL LIMIT
//microstepping to smooth movement? DRIVER HAS MS PINS THAT

//update done based on slowest motor.






void cap_w(float w_in, float w_turn_in, float* lp, float* rp){ //determine w cap first, then cap w_turn based on w

  //Serial.println("at capper");

  float w_mag = abs(w_in);
  float w_turn_mag=abs(w_turn_in);

  float w_chosen;
  float w_turn_chosen;

  if(w_mag>=w_max){
    w_chosen = (-1+2*sgn(w_in))*w_max;
  }
  else if (w_mag<=w_min){ //lim was 0.5
    
    w_chosen =  (w_mag<w_min*0.5) ? 0 : (w_min*(-1+2*sgn(w_in))); //for the section lower than w_min, set right of section to w_min and left to zero to minimize error
    
  }
  else{
    w_chosen=w_in;

  }


  //if w+w_turn hit max, limit w_turn to until max

    //Serial.print("w_chosen: ");
    //Serial.println(w_chosen); 

  float w_chosen_mag=abs(w_chosen);

  if(w_chosen_mag<=0.005){

    if(w_turn_mag>=w_max){
      
      w_turn_chosen = (-1+2*sgn(w_turn_in))*w_max;
      
    }
    
    else if (w_mag<=w_min){ //lim was 0.5
    
      w_turn_chosen =  (w_turn_mag<w_min*0.5) ? 0 : (w_min*(-1+2*sgn(w_turn_in))); //for the section lower than w_min, set right of section to w_min and left to zero to minimize error
    
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

  /*
 * EASIER TO FIND HALF CYCLES AND COMPARE
 * 
 * if sign of first and last the same, just increment period accodingly
 * if one is zero, decrement from 15000 and add one FULL cycle to get to 15000
 * if signs differ, (15000-current)+(15000-goal)+2 (for large to zero and zero to large shift)
 * 
 * If next step leads to larger than max_period next w set is fixed to zero and period set to one step larger than max
 * If going from zero, next w is set to max period times whichever direction we're going to
 * 
 * ASSUME WE ALREADY SET PERIOD TO CURRENT+STEP
 * 
 */

  float stp_value=*stp;
  bool is_goal_zero=abs(w_d)<w_min/2.2;
  bool is_current_zero=abs(*w)<w_min/2.2;

  float period_change=0;

  if(is_current_zero){ //current w is zero and either moving to + or - range, or staying at zero

    *w = is_goal_zero ? 0 : (-1+2*sgn(w_d))*w_min;
    *stp=max_period;
    
  }

  else if(is_goal_zero || sgn(*w)!=sgn(w_d)){ //if goal is zero or a different sign that current. Move towards 15000 w=0

    float diff=max_period-stp_value;

    if(abs(diff)>max_step_period_change){

      stp_value=stp_value+max_step_period_change*(-1+2*sgn(diff));
      *stp=stp_value;
      *w=((2*PI)/(steps_per_revolution*stp_value))*(-1+2*sgn(*w));
      
    }
    
    else{ //lower than 900 so round to max_period (w become zero

      stp_value=max_period;
      *stp=stp_value;
      *w=0;
      
    }
    
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
  //CORRECT
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

  //step periods once to get motors on first period

  one_step(&w_left, w_left_desired, &left_step_period, left_step_period_desired);
  one_step(&w_right, w_right_desired, &right_step_period, right_step_period_desired);

  w=(w_left+w_right)*0.5;
  w_turn=w_left-w;

  //toggle done based on completion of largest step. sensor done for synced faster signal->automatically the nonzero one. NOT DO SENSOR UPDATE BASED ON LARGEST_RIGHT AS IT GIVES THE MOTOR WITH LARGEST STEP. DO ACCORDING TO THE SHORTEST CURRENT PERIOD 
  
  } 


//accelerates w's to desired w's while also doing the stepping of the motor. Difference from previous motor updater is that periods can be changed based on desired input
void turn_motor(){ //offset does the turn. positive for clockwise turns  //change dir based on sign. Fowward for positive  Would bypass w=0 because we can't implement it due to min_period limits (0 means infinite delay) 

  //Serial.println("turning motors");

  //CHANGE PERIOD ONLY ON ZERO STATE

  float current_time=micros()*0.000001; //1) make state change based on duty, 2) give direction turn switching with zero bypass 3) sensor would use the smallest one 4)implement a turn_motor_off: set w to zero and period inf when off. When on set w to w_min and period no finite number
  bool is_left_zero=abs(w_left)<w_min/2.2;
  bool is_right_zero=abs(w_right)<w_min/2.2;
  

  //toggle state if the current duty of the left step signal passes. If left is zero, duty is fixed to one, resulting in toggle period being the complete period of zero
  if( (current_time-left_step_time) >= ( left_step_period * ( 1-duty_cycle+left_state*(-1+2*duty_cycle) ) * !is_left_zero ) ) { //CAN SKIP STEPS. NOT SEND UPDATES TOO FAST, MAKE W STEPS TOO HIGH

    digitalWrite(left_dir_pin, sgn(w_left)); //HIGH IS FORWARD //PROBLEM FOR LOW SPEEDS; WHEN W IS ZERO SAME SGN RESPONSE GIVEN BUT WE WANT SIGNS TO BE DIFFERENT
    
    left_state = is_left_zero ? HIGH : !left_state; //Not do step and keep giving high (high to not make phase shift) if w is zero
    digitalWrite(left_step_pin, left_state); //make 50% duty cycle if duuty seems too small. TOO SMALL AND PULSES NOT HAVE ENOUGH TIME TO RISE TO HIGH STATE. TURN TO 50% DUTY CYCLE
    //Either minimum cycle method or slow rise method (how slow and accurate???)
    
    cycles=cycles-(is_left_zero ? 1 : left_state)*!right_largest;    //remove one if the signal is the one half_cycles uses, subtract another one if previous w was zero (account for zero having no negative part)
    //Didn't removed the period of zeros to have a complete picture readable in code 

    

    //change period if had 2 half cycles passed (went to high or on high) and not reached desired yet
    if(abs(left_step_period_desired-left_step_period)>=75*0.000001 && left_state){
      
      one_step(&w_left, w_left_desired, &left_step_period, left_step_period_desired); //do one step towards the desired period. one_step will determine correct direction & amount & set w accordingly.
    
    }
    
    left_step_time=micros()*0.000001; 
    
  }

  if( (current_time-right_step_time) >= ( right_step_period * ( 1-duty_cycle+right_state*(-1+2*duty_cycle) ) * !is_right_zero ) ) { 
    
    digitalWrite(right_dir_pin, !sgn(w_right)); //LOW IS FORWARD, that's why sign is flipped

    right_state=is_right_zero ? HIGH : !right_state;
    digitalWrite(right_step_pin, right_state);

    cycles=cycles-(is_right_zero ? 1 : right_state)*right_largest;

    Serial.println(abs(right_step_period_desired-right_step_period)>=75*0.000001 && right_state); //change period is period not within 75 us of desired period

    if(abs(right_step_period_desired-right_step_period)>=75*0.000001 && right_state){ //NEED TO ENSURE WRAP AROUND BEFORE IT'S OK (TO CHANGE DIRECTION GO TOWARDS 15000 FIRST THEN WRAP TO 0 THEN -15000 AND CONTINUE INCREASING)

      one_step(&w_right, w_right_desired, &right_step_period, right_step_period_desired);
     
    }

    right_step_time=micros()*0.000001;
    
  }
  
}


/*
 * IF JITTER AND INACCURACY OCCUR IN CHANGE TO ZERO. UTILIZE THE EN PIN TO SET MOTORS TO ZERO
 * IF W_MIN TOO HIGH, *IF DUE TO DELAY LIMIT>INCREASE BANDWIDTH OR MAKE CONTROLLER MORE DELAY RESISTANT OR FIND WAY TO INCREASE PERIOD CHANGE AMOUNT *IF DUE TO ABSOLUTE MINIMUM->INCREASE STEP PER REV FOR LOWER SPEEDS BY MICROSTEPPING  
 * IF W_MAX TOO LOW, FIND WAYS TO REDUCE SENSOR AND LISTENING DELAYS OR MOVE THESE COMPONENTS TO THE OTHER CORE
 * 
 */



void setup() {
  Serial.begin(115200);


//Control setup

  pinMode(left_step_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(right_step_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);

  //calibration point not important since STATE USES PITCH ACCELERATION?
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu_setup();

  digitalWrite(left_step_pin, HIGH);
  digitalWrite(right_step_pin, HIGH);

  
  right_step_time=micros()*0.000001;
  left_step_time=micros()*0.000001;
  pitch_time=micros()*0.000001;
  yaw_time=micros()*0.000001;
  velocity_time=micros()*0.000001;
  sense_time=micros()*0.000001;
  yaw_rate_pid_time=micros()*0.000001;
  
  set_velocity(0); //in real operation, would call the set method when see a laptop command while update is done whenever wheel acceleration is complete to get actual w to control input

//FPGA and wireless setup

  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  //   Serial.println("Connecting to WiFi...");
  // }
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();

//multithread initialization
  sense_mutex = xSemaphoreCreateMutex();

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


  vTaskDelete(NULL); //delete setup task

}

void sendVideoCode( void * parameter) {
  String hexBuffer = "";
  // float dx;
  // float timeout_time; micros()*0.000001;
  for(;;) {
    while (SerialPort.available() < 1) { 
      SerialPort.write(0xEE); //flag to indicate to nios that esp is ready to receive frame
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    uint8_t header = SerialPort.read();
    uint16_t header4x = header * 4;
    //Serial.println(header4x);


    // Read the frame based on the header
    while (SerialPort.available() < (header4x)) { 
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    uint8_t buffer[header4x];
    SerialPort.readBytes(buffer, header4x);

    //TODO: attach state variables here?
    xSemaphoreTake(sense_mutex, 10);

    xSemaphoreGive(sense_mutex);

    // Convert the buffer to a hexadecimal string
    for (int i = header4x-1; i >= 0; i--) {
      char hex[3];
      sprintf(hex, "%02X", buffer[i]);
      hexBuffer += String(hex);
    }
    //Serial.println(hexBuffer);

    // Send the frame over WebSocket


    // timeout_time = micros()*0.000001;
    // while(sense_on && (micros()*0.000001-timeout_time) < (min_sense_period*3)){} //5.4ms maximum delay that can be added to image transmission
    // dx = x - x_prev;

    ws.textAll(hexBuffer);
    // x_prev = x;
    hexBuffer = "";
  }
}

//Control loop
//get w from controller, get step differences, determine the number of cycles needed and not toggle until they're passed. at the end of one toggle period change the motor period
void ControlTask( void * parameter){//TEST DELAY REACTIVITY TOMORROW WITH POWER SOURCE AND DETERMINE DELAY CAUSED MAXIMUM VELOCITY (DYNAMICALLY UPDATE IT BASED ON DELAY AVAILABLE IN SYSTEM?). 
  //DO CONTROLLER TEST WITHOUT SERIALS AND CHANGE DUTY CYCLE AMOUNT TO ALLOW SMOOTHNESS. sensor done, basic control and calibration left. Another pole placement controller to control bandwidth of lqr stabilized controller?, or use pole placement to stabilize then lqr to thrive?
  // put your main code here, to run repeatedly:

  //Sensing done at start, can't sense during speeds with period lower than min_sensing period and become blind, so speed limited by min sensing period, should be larger than it to allow sensing to be after motor state toggling

  //micros()*0.000001-sense_time>( (toggle_period<=max_sense_period) ? toggle_period : 100*0.001)

  //Set to the length of the current state of the square wave. Set to the total period length if w is zero
  float current_time=micros()*0.000001;
  float current_right_half_period=right_step_period*(1-duty_cycle+right_state*(-1+2*duty_cycle))*!(abs(w_right)<w_min/2.2);
  float current_left_half_period=left_step_period*(1-duty_cycle+left_state*(-1+2*duty_cycle))*!(abs(w_right)<w_min/2.2);
  bool fastest_right=abs(w_right)>=abs(w_left);
  
  /*
   * set current half periods to max_period if currently at zero
   */

  //if there's no incoming edges on fastest step before the min sense period and sense_runtime passed, sense as pleased. Else not and do sense at start when step changes. Did *1.1 to give slack for small frequencies. Can wait until start and preserve shortest
  if( ( !(fastest_right && current_right_half_period-(current_time-right_step_time)<sense_runtime*1.1) || !(!fastest_right && current_left_half_period-(current_time-left_step_time)<sense_runtime*1.1) ) && current_time-sense_time>=max_sense_period*0.9){ //ensure sensed values are accurately tracked while not adding significant delay to motor times

    //sense_on=true; Replaced by takemutex in sense()
 
    sense(); //too much sensor calls could distort the slow wave if w_turn exists. Need to limit sensor calls so slow period has less chance of getting distorted PHASE SHIFT???
    //Called to keep pitches and yaws up to date until desired w is updated. Rover is blind during w acceleration.

    //sense_on=false; Replaced by givemutex in sense()
    
    sense_time=micros()*0.000001;
    
  }

  //Serial.print("dt: ");
  //Serial.println(micros()*0.000001-toggle_time);

  //update w only when went through the desired number of cycles to get smooth transition
  if(cycles<=0){ //this is where the desired controllers would be run when we're done with w acceleration

    //Serial.println("updating velocity");

    digitalWrite(left_step_pin, HIGH);
    digitalWrite(right_step_pin, HIGH); //reset both signals to 0 phase to ensure any phase shifts done on the slow signal due to delays isn't carried over.

    update_velocity(); //run the control loop and get the new motor w's, cap them, then set the new step periods and toggle accordingly

    //If delay too high, we can't have steps with short periods->can cause jitter and speed reduction

    //Serial.println("velocity updated");
    //toggle_time=micros()*0.000001;
    
  }


  turn_motor(); //continuouly check if it's time to step then step and also update period if going towards desire
  

/*
  Serial.println("INPUTS:");
  Serial.print("w: ");
  Serial.println(w);
  Serial.print("w_turn: ");
  Serial.println(w_turn);
*/
  vTaskDelay(1 / portTICK_PERIOD_MS);//1ms delay to allow other tasks to run. Increase it to see how much delay system can take (maybe undone by the larger low speed delays)
  //Increase it to see how much delay system can take (maybe undone by the larger low speed delays)

  /*
  update_velocity();

  Serial.println("STATES");
  Serial.print("x: ");
  Serial.println(x);
  Serial.print("v: ");
  Serial.println(v);
  Serial.print("pitch: ");
  Serial.println(pitch*(360/(2*PI)));
  Serial.print("pitch_rate: ");
  Serial.println(pitch_rate*(360/(2*PI)));

  */

}


void loop() { //by default this loop runs in core 1 so the controller code would go here while the image broadcasting is done on core 0
                                                    //better setup another task to run the controller code and leave this loop blank
//blank loop to allow other tasks to run

}
