#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #define FASTLED_ALL_PINS_HARDWARE_SPI
// #define FASTLED_ESP32_SPI_BUS HSPI
#include "FastLED.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "PRINT_SERIAL_DEBUG" if you want to see the debug output
// for the OUTPUT_READABLE_YAWPITCHROLL values in the terminal.
// Note that printing serial messages slows down the execution speed!
//#define PRINT_SERIAL_DEBUG


#define INTERRUPT_PIN 5
#define NUM_LEDS 4 // Count of LEDs in the stripe connected to the microcontroller LED_PIN
#define LED_PIN 18 // Microcontroller pin where the LED stripe is connected
#define LED_BRIGHTNESS 50 // Brightness of the LED stripe

CRGB leds[NUM_LEDS];
uint8_t paletteIndex = 0;
DEFINE_GRADIENT_PALETTE (heatmap_gp) {
  0, 255, 0, 0,
  108, 0, 0, 0,
  188, 0, 0, 0,
  255, 255, 0, 0
};

CRGBPalette16 greeblu = heatmap_gp;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion quatern;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float calculatedDegrees = 0.0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    doMPU6050Orientation();

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    // FastLED.addLeds<WS2812, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    resetStrip();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    if (mpuInterrupt) {
      doMPU6050Orientation();
      mpuInterrupt = false;
    }

    // If the maxMap-Value gets adjusted, remember to also change the values in the ledStuff()-method.
    ledStuff(mapDegreesToMilliseconds(calculatedDegrees, 0, 90, 0, 5000));   
}

float mapDegreesToMilliseconds(float degrees, int minDeg, int maxDeg, int minMap, int maxMap) {
  return ( (degrees - minDeg) * ( (maxMap - minMap) / (maxDeg - minDeg) ) + minMap);
}

void ledStuff(float delay) {
  delay=(delay / 100);
  #ifdef PRINT_SERIAL_DEBUG
    Serial.print("EVERY_N set to:\t");
    Serial.println(50 - delay); // 50 = mapDegreesToMilliseconds.maxMap Value divided by 100
  #endif
  fill_palette(leds, NUM_LEDS, paletteIndex, 200 / NUM_LEDS, greeblu, LED_BRIGHTNESS, LINEARBLEND);
  EVERY_N_MILLISECONDS_I(timingObj, 1) {
    timingObj.setPeriod(50 - delay); // 50 = mapDegreesToMilliseconds.maxMap Value divided by 100
    paletteIndex++;
  }
  FastLED.show();
}

void resetStrip() {

}

void doMPU6050Orientation() {
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&quatern, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &quatern);
            mpu.dmpGetYawPitchRoll(ypr, &quatern, &gravity);
            float yawl = ypr[0] * 180/M_PI;
            float pitchl = ypr[1] * 180/M_PI;
            float rolll = ypr[2] * 180/M_PI;
            calculatedDegrees = calculateDegrees(pitchl, rolll);
            #ifdef PRINT_SERIAL_DEBUG
              Serial.print("yaw pitch roll\t");
              Serial.print(yawl);
              Serial.print("\t");
              Serial.print(pitchl);
              Serial.print("\t");
              Serial.println(rolll);
              Serial.print("Overall degrees\t");
              Serial.println(calculatedDegrees);
            #endif
        #endif
    }
}

float calculateDegrees(float pitch, float roll) {
  float kipp_winkel = atan2(sqrt(pow(tan(pitch * PI / 180), 2) + pow(tan(roll * PI / 180), 2)), 1) * 180 / PI;
  return kipp_winkel;
}
