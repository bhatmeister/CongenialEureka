
#include<Wire.h>
#include <NewPing.h>

#define SLAVE_ADDRESS 0x04
#define SPEED 255
#define TURN_SPEED 255
#define SONAR_NUM     5 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

int command = -1;
int output[11];
int autoPilotMode = 0;
uint8_t outputUnmapped[24];
unsigned long currentTime;
unsigned long delayTime;
boolean obstacleFlag = false;
boolean obstacleArray[SONAR_NUM];
boolean obstacle;
boolean sideUS;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(11, 6, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(12, 7, MAX_DISTANCE),
  NewPing(13, 8, MAX_DISTANCE),
  NewPing(0, 9, MAX_DISTANCE),
  NewPing(1, 10, MAX_DISTANCE)
};




void setup() {
  //Motors
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
 
  //LDR
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  //Ultrasonic
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  //Serial Communications
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(sendData); 
  Wire.onReceive(receiveData);
 
}

void loop() {
  
  output[0] = analogRead(A0);
  output[1] = analogRead(A1);
  output[2] = analogRead(A2);
  output[3] = analogRead(A3);
  output[4] = analogRead(A6);
  output[5] = analogRead(A7);
  fetchUltrasonicValues();
  mapValues(output);
  checkForObstacles();
  
  if(autoPilotMode)
  {
    if(obstacleFlag)
    {
      if(sideUS){
        delayTime = currentTime + 100;
      }
      else
      {
        delayTime = currentTime + 1000;
      }
      if(millis() >= delayTime)
      {
        obstacleFlag = false;
        move(1);
      }
    }
    else{
      if(obstacle)
      {
         obstacleFlag = true;
        //Stop the bot and move back a few steps
        if(obstacleArray[1] == true){
          turn(2,1);
        }

        if(obstacleArray[3] == true){
          turn(1,1);
        }

        if(obstacleArray[0] == true){
          turn(1,2);
        }

        if(obstacleArray[4] == true){
          turn(2,2);
        }
        
        if(obstacleArray[2] == true){
          long direction = random(0,2);
        
          switch(direction)
            {
              case 0: turn(1,2);
                      break;
              case 1: turn(2,2);
                      break;
            }
        }

        

        currentTime = millis();
      }

      else
      {
        move(1);
      }
  
    }
  }

}

void checkForObstacles(){

  obstacle = false;
  sideUS = false; 
  for(int q = 0; q < SONAR_NUM; ++q )
  {
    if(q == 0 || q == 2 || q == 4)
    {
      if(cm[q] > 2 && cm[q] < 10)
      {
        obstacleArray[q] = true;
        obstacle = true;
      }
      else
      {
        obstacleArray[q] = false;
      }
    }
    else
    {
      if(cm[q] > 2 && cm[q] < 5)
      {
        obstacleArray[q] = true;
        obstacle = true;
        sideUS = true;
      }
      else
      {
        obstacleArray[q] = false;
      }
    }
  }
  
}

void mapValues(int mapped[11]){

  int i = 0;
  int checksum = 0;
  for(int j=2; j<24; j=j+2)
  {
    
    outputUnmapped[j] = mapped[i] >> 8;
    checksum += outputUnmapped[j];
    outputUnmapped[j+1] = mapped[i] & 0xff;
    checksum += outputUnmapped[j+1];
    ++i;
   }
   outputUnmapped[0] = checksum >> 8;
   outputUnmapped[1] = checksum & 0xff;
 }
 
void fetchUltrasonicValues(){
 // Serial.println("in function");
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
   // Serial.println("in loop");
    if (millis() >= pingTimer[i]) {               // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  int k = 6;
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    output[k] = cm[i];
    ++k;
    //Serial.print(cm[i]);
    //Serial.print("  ");
  }
  //Serial.println();
}


void drive(int command)
{
  if(command == 1){
    move(1);

  }

  else if(command == 10){
    stop();
  }

  else if(command == 2)
  {
    turn(1,1);
  }

  else if(command == 3)
  {
    turn(2,1);
  }

  else if(command == 4){
     move(2);
  }
  
}

void move(int direction) {
 
  int inPin1 = LOW;
  int inPin2 = SPEED;

  if (direction == 1) {
    inPin1 = SPEED;
    inPin2 = LOW;
    }

    analogWrite(3, inPin1);
    digitalWrite(2,inPin2);
    analogWrite(5, inPin1);
    digitalWrite(4,inPin2);
  
}

void turn(int motor,int direction){

  int inPin1 = LOW;
  int inPin2 = TURN_SPEED;

  if (direction == 1) {
    inPin1 = TURN_SPEED;
    inPin2 = LOW;
    }
 
    
  if (motor == 1) {
    analogWrite(3, inPin1);
    digitalWrite(2, inPin2);
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
   
  }

  else{
    digitalWrite(3, LOW);
    digitalWrite(2, LOW);
    analogWrite(5, inPin1);
    digitalWrite(4, inPin2);
}
}

void stop() {
  Serial.println("stop");
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
}


void runMotors(int command)
{
   if (command == 5){
    autoPilotMode = !autoPilotMode;
    stop();
  }

 drive(command);

}

void sendData(){
  Wire.write(outputUnmapped,24);
}

void receiveData(int x) {
  while(Wire.available())
  {
    command = Wire.read();
    runMotors(command);
   
    Serial.println(command);
  }
}
