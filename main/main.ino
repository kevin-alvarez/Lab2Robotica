#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, zpitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 169.5;//Busqueda de Referencia angulo 0 (173) (172.120)
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 40;   // entre 30 y 100 - 40
double Ki = 180; // entre 0 y 200 - 180
double Kd = 2; // entre 0 y 2 - 2

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.5;
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

//movement instructions
double movementFactor = 2; //grados de cambio del 0 del giroscopio
int speedLeft = 100;
int speedRight = 100;
bool turningLeft = false;
bool turningRight = false;
int RR = 0;
void moveForward(){
  if (setpoint != originalSetpoint + movementFactor) setpoint += movementFactor;
}

void moveBackward(){
  if (setpoint != originalSetpoint - movementFactor) setpoint -= movementFactor;
}

void moveLeft(){
  turningLeft = true;
}

void moveRight(){
  turningRight = true;
}

void idleMovement(){
  if (setpoint != originalSetpoint) setpoint = originalSetpoint;
  turningLeft = false;
  turningRight = false;
}

//bluetooth
int bTx = 11;
int bRx = 12;
char cmdChar;
char state; // can be P | I | D
SoftwareSerial bluetooth(bTx, bRx);
String stackPID = "";
bool reading = false;


// Bluetooth config
void bluetoothSetup(){
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void bluetoothLoop(){
  if(bluetooth.available() > 0){
    cmdChar = bluetooth.read();
    //Serial.println("Entered...");
    if(reading){ //En caso de fallar, mover dentro de condicional de bluetooth
    switch(cmdChar){
      case 'o':
        reading = false;
        switch(state){
          case 'P':
            Kp = stackPID.toDouble();
            break;
          case 'I':
            Ki = stackPID.toDouble();
            break;
          case 'D':
            Kd = stackPID.toDouble();
            break;
          default:
            break;
        }
        Serial.println(stackPID);
        pid.SetTunings(Kp, Ki, Kd);
        stackPID = "";
        break;
      default:
        stackPID += cmdChar;
        break;
      }
    }
    switch(cmdChar){
      case 'w':
        moveForward();
        Serial.println(cmdChar);
        break;
        
      case 'a':
        moveLeft();
        Serial.println(cmdChar);
        break;
        
      case 's':
        moveBackward();
        Serial.println(cmdChar);
        break;
    
      case 'd':
        moveRight();
        Serial.println(cmdChar);
        break;
        
      case 'P':
        reading = true;
        state = 'P';
        break;
        
      case 'I':
        reading = true;
        state = 'I';
        break;
        
      case 'D':
        reading = true;
        state = 'D';
        break;
        
      default:
        idleMovement();
        Serial.println(cmdChar);
        break;
    }
  }
  
}

//guardado de rutas
char currentInst;
int loopCounter;
struct waypoints{
  char instruction[50];
  int loopCount[50];
  int lastItem = 0;
};
struct waypoints wp;
int loopRoute(){ //guardar estado de ruta por cada loop
  
  currentInst = cmdChar; //guarda instruccion actual  
}

void runRoute(){ //correr ruta guardada
  
}

void setup()
{
  // bluetooth setup
  bluetoothSetup();
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
	#endif

	mpu.initialize();

	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();

		//setup PID
		pid.SetMode(AUTOMATIC);
		pid.SetSampleTime(25); // sample original: 10
		pid.SetOutputLimits(-255, 255); 
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}


void loop()
{
  // bluetooth loop
  bluetoothLoop();
  
	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		//no mpu data - performing PID calculations and output to motors 
		pid.Compute();
    RR = RR+1 % 3;
    if (turningLeft && RR == 0){
      motorController.move(-200,200, MIN_ABS_SPEED);
    }else if (turningRight && RR == 0){
      motorController.move(200,-200, MIN_ABS_SPEED);
    }else{
      motorController.move(output, MIN_ABS_SPEED);
    }
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		input = ypr[1] * 180/M_PI + 180;
	}
}
