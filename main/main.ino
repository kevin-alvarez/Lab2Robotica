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
  //if (setpoint != originalSetpoint + movementFactor) setpoint += movementFactor;
  turningLeft = true;
}

void moveRight(){
  //if (setpoint != originalSetpoint + movementFactor) setpoint += movementFactor;
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
bool readingPID = false;
char stateRoute; // Rutas: 'r' - recording ; 'p' - playing ; '0' - none

// Bluetooth config
void bluetoothSetup(){
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void bluetoothLoop(){
  if(bluetooth.available() > 0){
    cmdChar = bluetooth.read();
    //Serial.println("Entered...");
    if(readingPID){ //En caso de fallar, mover dentro de condicional de bluetooth
      switch(cmdChar){
        case 'o':
          readingPID = false;
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
        saveInst();
        break;
        
      case 'a':
        moveLeft();
        saveInst();
        break;
        
      case 's':
        moveBackward();
        saveInst();
        break;
    
      case 'd':
        moveRight();
        saveInst();
        break;
        
      case 'r':
        stateRoute = 'r'; // recording route
        break;
        
      case 'p':
        stateRoute = 'p'; // playing route
        break;
        
      case 'P':
        readingPID = true;
        state = 'P';
        break;
        
      case 'I':
        readingPID = true;
        state = 'I';
        break;
        
      case 'D':
        readingPID = true;
        state = 'D';
        break;
        
      default:
        idleMovement();
        if (stateRoute == 'r') savePoint();
        break;
    }
  }
  
}

//guardado de rutas
char currentInst;
int loopCounter;
int currentIndexRunning; // var to run routes
struct waypoints{
  char instruction[50];
  int loopCount[50];
  int lastItem = 0;
};
struct waypoints wp;

void routeSetup(){
  stateRoute = '0';
  currentInst = '0';
  loopCounter = 0;
  currentIndexRunning = 0;
}

void saveInst(){
  currentInst = cmdChar;
}

void savePoint(){
  wp.instruction[wp.lastItem] = currentInst; // save current instrucion on list
  wp.loopCount[wp.lastItem] = loopCounter; // save loop counter on list
  wp.lastItem++; // update list size
  loopCounter = 0; // restart loop counter
}

void loopRoute(){
  loopCounter++;
}

void runRoute(){
  switch(wp.instruction[currentIndexRunning]){ // ineficiente - muchas lecturas y escrituras
    case 'w':
      moveForward();
      break;

    case 'a':
      moveLeft();
      break;

    case 's':
      moveBackward();
      break;

    case 'd':
      moveRight();
      break;
  }
  wp.loopCount[currentIndexRunning]--;
  if (wp.loopCount[currentIndexRunning] == 0) currentIndexRunning++; // if not loops remaining, then go to next instruction
  if (currentIndexRunning == wp.lastItem) stateRoute = '0'; // if all waypoints were passed, then finish route playing
}

void setup()
{
  // bluetooth setup
  bluetoothSetup();
  // Route setup
  routeSetup();
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
  if (stateRoute == 'r') loopRoute();
  if (stateRoute == 'p') runRoute();
  
	// if programming failed, don't try to do anything
	if (!dmpReady) return;
	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		//no mpu data - performing PID calculations and output to motors 
		pid.Compute();
    if (turningLeft && output < 10){
      motorController.move(0, output, MIN_ABS_SPEED); // giro izquiera (-w_i = w_d)
    }else if (turningRight && output < 10){
      motorController.move(output, 0, MIN_ABS_SPEED); // giro derecha (w_i = -w_d)
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
