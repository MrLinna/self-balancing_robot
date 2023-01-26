
// Sensor 
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// remote control 
#include <WiFi101.h>
#include <BlynkSimpleMKR1000.h>
char auth[] = "Blynk authentication";
char ssid[] = "network's SSID";
char pass[] = "network's password";

// initialization of variables
double Kp, Ki, Kd; // Coefficients to adjust different parts of the PID sum
double error, last_error;
double pid_p, pid_i, pid_d, last_pid_d, pid;
double target_angle;
double kierroslkm=30; // how many times the derivative is used
int n;
// motor pins
int a = 3;   
int b = 6;  
// motor directions
int a0 = 4;  
int a1 = 5;  
int b0 = 8;  
int b1 = 7; 

void setup(){
  Serial.begin(9600); 
  // Initialization of motor pins
  pinMode(a, OUTPUT);  
  pinMode(b, OUTPUT);
  pinMode(a0, OUTPUT);
  pinMode(a1, OUTPUT);
  pinMode(b0, OUTPUT);
  pinMode(b1, OUTPUT);
  // sensor initialization
  if(!bno.begin()){          
    Serial.println(" bno error");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  // Remote control initialization
  Blynk.begin(auth, ssid, pass);
  Blynk.run();
}

void loop(){
    // reading values ​​from the remote control
    Blynk.run();
    // reading sensor values
    sensors_event_t event;
    bno.getEvent(&event);
    double sensor_angle = event.orientation.z;
    // the error of the previous round is saved and the current one is calculated.
    last_error = error; 
    error = target_angle-sensor_angle;

    // directly proportional part
    pid_p = constrain(Kp*error/2,-255,255);

    // integral
    if((Ki==0)){
      pid_i = 0;
    }
    else{
      pid_i = pid_i+Ki*error*0.01;
      pid_i = constrain(pid_i,-255,255);
    }
    
    // derivative
    pid_d =(error - last_error) * Kd;
    constrain(pid_d,-255,255);

    // enhancing the derivative effect
    n = n+1;
    if ((n/kierroslkm==1)||(last_pid_d==0)){
      last_pid_d = pid_d;
      n=0;
    }
    else{
      pid_d = last_pid_d;
    }
    // the value for the motors is obtained by summing the calculated terms.
    pid = pid_p + pid_i + pid_d;
    pid = constrain(pid,-255,255);
    shutoff();
    
    // Turn off the motors when there is no more hope.
    if (error < -60 || error > 60) pid = 0; 
    // Liikkeen suunnan valitseminen
    if (error>0){
        bkw(abs(pid));
    }
    else{
        fwd(abs(pid));
      }
}
// functions for adjusting motors
void fwd(int velocity){ 
  digitalWrite(a0, 0);
  digitalWrite(a1, 1);
  digitalWrite(b0, 0);
  digitalWrite(b1, 1);
  analogWrite(a, velocity); 
  analogWrite(b, velocity); 
}

void bkw(int velocity){ 
  digitalWrite(a0, 1);
  digitalWrite(a1, 0);
  digitalWrite(b0, 1);
  digitalWrite(b1, 0);
  analogWrite(a, velocity); 
  analogWrite(b, velocity); 
}

void shutoff(){ 
  digitalWrite(a0, 0);
  digitalWrite(a1, 0);
  digitalWrite(b0, 0);
  digitalWrite(b1, 0);
}
// Values from the remote control
BLYNK_WRITE(V0){
  float kulma = param[0].asFloat();
  target_angle = kulma/10 -5;
}
BLYNK_WRITE(V1){
  Kp = param[0].asFloat();
}
BLYNK_WRITE(V2){
  Ki = param[0].asFloat();
}
BLYNK_WRITE(V3){
  Kd = param[0].asFloat();
}
BLYNK_WRITE(V4) {
  x = param[0].asFloat();
}
