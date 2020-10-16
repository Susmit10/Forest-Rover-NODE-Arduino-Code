
// FINAL PROJECT


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h>
#include <Servo.h>          //Servo motor library.
#include <NewPing.h>        //Ultrasonic sensor function library.

//our L298N control pins
const int LeftMotorForward = 32;     //7
const int LeftMotorBackward = 34;    //6
const int RightMotorForward = 36;    //4
const int RightMotorBackward = 38;   //5
const int EnableA = 4;
const int EnableB = 5;


//sensor pins
#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2

#define maximum_distance 200
boolean goesForward = false;
int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name

#define led 12
#define DHTPIN 44
#define DHTTYPE DHT22
#define gas A0
#define flame A3
#define water 25
#define pir 30
#define Relay1 40
#define Relay2 42
#define resetMega 11

DHT dht(DHTPIN, DHTTYPE);

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte temperature;
  byte humidity;
  byte gasValue;
  byte flameValue;
  byte mode;
  byte waterValue;
  byte rel1;
  byte rel2;
  byte pirValue;
//  byte tSwitch2;
//  byte button1;
//  byte button2;
//  byte button3;
//  byte button4;
};

Data_Package data; //Create a variable with the above structure

struct Data_PackageRec {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
//  byte tSwitch1;
//  byte tSwitch2;
  byte relay1;
  byte relay2;
  byte resetMEGA;
  byte Mode;
  byte buttonState;
  byte dir;
  byte leftMotor;
  byte rightMotor;
};

Data_PackageRec dataRec; //Create a variable with the above structure


void setup() {
  pinMode(led, OUTPUT);
  pinMode(gas, INPUT);
  pinMode(flame, INPUT);
  pinMode(water, INPUT);
  pinMode(pir, INPUT);
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  Serial.begin(9600);
  dht.begin();
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  //radio.setPALevel(RF24_PA_MIN);

  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  pinMode(EnableA, OUTPUT);
  pinMode(EnableB, OUTPUT);

  digitalWrite(EnableA, LOW);
  digitalWrite(EnableB, LOW);
  
  servo_motor.attach(10); //our servo pin

  servo_motor.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);

  data.temperature = 0;
  data.humidity = 0;
  data.gasValue = 0;
  data.flameValue = 0;
  data.mode = 0;
  data.waterValue = 0;
  data.rel1 = 0;
  data.rel2 = 0;
  data.pirValue = 0;
//  data.tSwitch2 = 1;
//  data.button1 = 1;
//  data.button2 = 1;
//  data.button3 = 1;
//  data.button4 = 1;

  resetData();
  
}

void loop() {

  data.gasValue = analogRead(gas);
  data.humidity = dht.readHumidity();
  data.temperature = dht.readTemperature();
  data.flameValue = analogRead(flame);
  data.waterValue = digitalRead(water);
  distance = readPing();
  data.pirValue = digitalRead(pir);
  
  Serial.println(distance);
  
  delay(5);

  radio.stopListening();
  //int potValue = analogRead(A0);
  //int angleValue = map(potValue, 0, 1023, 0, 180);
  //radio.write(&angleValue, sizeof(angleValue));
  radio.write(&data, sizeof(Data_Package));

  delay(5);
  radio.startListening();

  if (radio.available()) {
    radio.read(&dataRec, sizeof(Data_PackageRec)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 10000 ) { // If current time is more then 10 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior.
  }
  // Print the data in the Serial Monitor
  Serial.print("j1PotX: ");
  Serial.print(dataRec.j1PotX);
  Serial.print("; j1PotY: ");
  Serial.print(dataRec.j1PotY);
  Serial.print("; button1: ");
  Serial.print(dataRec.j1Button);
  Serial.print("; j2PotX: ");
  Serial.println(dataRec.j2PotX);

  int m = dataRec.Mode;
  if(m == 1)
  {
  runObstacleAvoid();                      // OBSTACLE AVOIDING RANGER MAIN FUNCTION
  Serial.print("Running ObstacleAvoid");
  }
  else if(m == 2)
  {
  runJoystickControl();
  Serial.print("Running JoystickControl");
  }
  else 
  {
  runStop();
  Serial.print("At Stop MODE");
  }
  
  int z = dataRec.buttonState;
  if(z == 1)
  {
  digitalWrite(led, HIGH);
  Serial.println("Reading Success");
  }
  else
  {
  digitalWrite(led, LOW);
  //Serial.println("Reading Faliure");
  }
  int v = dataRec.relay1;
  if(v == 1)
  digitalWrite(Relay1, HIGH);
  else
  digitalWrite(Relay1, LOW);

  int r = dataRec.resetMEGA;
  digitalWrite(resetMega, r);

}



void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  dataRec.j1PotX = 127;
  dataRec.j1PotY = 127;
  dataRec.j2PotX = 127;
  dataRec.j2PotY = 127;
  dataRec.j1Button = 1;
  dataRec.j2Button = 1;
  dataRec.pot1 = 1;
  dataRec.pot2 = 1;
//  dataRec.tSwitch1 = 1;
//  dataRec.tSwitch2 = 1;
  dataRec.relay1 = 0;
  dataRec.relay2 = 0;
  //dataRec.button3 = 1;
  dataRec.Mode = 0;
  dataRec.buttonState = 0;
  dataRec.dir = 0;
  dataRec.leftMotor = 0;
  dataRec.rightMotor = 0;
  dataRec.resetMEGA = 1;
}



// OBSTACLE AVOIDING RANGER MAIN FUNCTION

void runObstacleAvoid(){

  digitalWrite(EnableA, HIGH);
  digitalWrite(EnableB, HIGH);

  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 20){
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft){
      turnRight();
      moveStop();
    }
    else{
      turnLeft();
      moveStop();
    }
  }
  else{
    moveForward(); 
  }
    distance = readPing();
}



// OBSTACLE AVOIDING RANGER FUNCTIONS

int lookRight(){  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void moveStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward(){

  if(!goesForward){

    goesForward=true;
    
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  }
}

void moveBackward(){

  goesForward=false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
}

void turnRight(){

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
  delay(500);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
 
  
  
}

void turnLeft(){

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(500);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}





void runJoystickControl()
{
  Serial.println("Running JoystickControl"); 


  if (dataRec.dir == 1)
      {
    // Motors are backwards
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorForward, LOW);
    digitalWrite(RightMotorBackward, HIGH);
    }
    else if(dataRec.dir == 2)
    {
    // Motors are forwards
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(RightMotorBackward, LOW);
     }
     else
     {
      digitalWrite(LeftMotorForward, LOW);
      digitalWrite(LeftMotorBackward, LOW);
      digitalWrite(RightMotorForward, LOW);
      digitalWrite(RightMotorBackward, LOW);
     }
 
      
      // Drive Motors
      analogWrite(EnableA, dataRec.leftMotor);
      analogWrite(EnableB, dataRec.rightMotor);
  
}





void runStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}
