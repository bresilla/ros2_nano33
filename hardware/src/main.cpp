#include <AHRS.h>
#include <Arduino_BMI270_BMM150.h>

AHRS ahrs;
SensorData data;

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 10;
unsigned long previousMillis = 0;

void setup() {
    //  Initialise the AHRS
    //  Use default fusion algo and parameters
    ahrs.begin();
    
    ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
    ahrs.setDeclination(12.717);                      //  Sydney, Australia

    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    Serial.print("Detected Board - ");
    Serial.println(ahrs.getBoardTypeString());

    if (IMU.begin() && ahrs.getBoardType() == BoardType::NANO33BLE_SENSE_R2) {
        Serial.println("BMI270 & BMM150 IMUs Connected."); 
        Serial.print("Gyroscope sample rate = ");
        Serial.print(IMU.gyroscopeSampleRate());
        Serial.println(" Hz");
        Serial.println();
        Serial.println("Gyroscope in degrees/second");
        Serial.print("Accelerometer sample rate = ");
        Serial.print(IMU.accelerationSampleRate());
        Serial.println(" Hz");
        Serial.println();
        Serial.println("Acceleration in G's");
        Serial.print("Magnetic field sample rate = ");
        Serial.print(IMU.magneticFieldSampleRate());
        Serial.println(" Hz");
        Serial.println();
        Serial.println("Magnetic Field in uT");
    } 
    else {
        Serial.println("BMI270 & BMM150 IMUs Not Detected.");
        while(1);
    }
}

void loop() {
    if (IMU.gyroscopeAvailable()) {  IMU.readGyroscope(data.gx, data.gy, data.gz);  }
    if (IMU.accelerationAvailable()) {  IMU.readAcceleration(data.ax, data.ay, data.az);  }
    if (IMU.magneticFieldAvailable()) {  IMU.readMagneticField(data.mx, data.my, data.mz);  }

    ahrs.setData(data);
    ahrs.update();

    Serial.print("Acceleration: ");
    Serial.print(data.ax);
    Serial.print(" ");
    Serial.print(data.ay);
    Serial.print(" ");
    Serial.println(data.az);

    Serial.print("Gyroscope: ");
    Serial.print(data.gx);
    Serial.print(" ");
    Serial.print(data.gy);
    Serial.print(" ");
    Serial.println(data.gz);


    Serial.print("Magnetometer: ");
    Serial.print(data.mx);
    Serial.print(" ");
    Serial.print(data.my);
    Serial.print(" ");
    Serial.println(data.mz);


    auto roll = ahrs.angles.roll;
    auto pitch = ahrs.angles.pitch;
    auto heading = ahrs.angles.yaw;

    Serial.print("Orientation: ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(heading);
}