#include <Wire.h>


#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);


long timer = 0, prevTimer = 0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                             // integration interval for both filter schemes

float a[3] = {0.0, 0.0, 0.0},
      g[3] = {0.0, 0.0, 0.0}, 
      coord[3] = {0.0, 0.0, 0.0}, 
      velocity[3] = {0.0, 0.0, 0.0}, 
      prevAcce[3] = {0.0, 0.0, 0.0}, 
      offSets[3] = {0.0, 0.0, 0.0},
      startVelocity[3] = {0.0, 0.0, 0.0},
      startCoord[3] = {0.0, 0.0, 0.0},
      timerHis[3][5] = {{0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}}, 
      acceHis[3][5] = {{0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}}; // record acceleration, coordinate, velocity, previous acceleration, offSets
int offSetCount[3] = {0, 0, 0},
    countCounter[3] = {0, 0, 0};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  if(millis() - timer > 1000){
    
    timer = millis()/1000;
    for (int i = 0; i < 3; i++) {
        coord[i] += (0.5*a[i]*(timer - prevTimer) + velocity[i])*(timer - prevTimer);
        velocity[i] += a[i]*(timer - prevTimer);
    }
    a[0] = mpu6050.getAccX(), a[1] = mpu6050.getAccY(), a[2] = mpu6050.getAccZ();
    g[0] = mpu6050.getGyroX(), g[1] = mpu6050.getGyroY(), g[2] = mpu6050.getGyroZ();
    // MadgwickQuaternionUpdate(a[0], a[1], a[2], g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f);

    for (int i = 0; i < 3; i++) {
        if (offSetJudge(abs(prevAcce[i]), abs(a[i]))) {
            if (offSetCount[i] == 0) {
                startCoord[i] = coord[i];
                startVelocity[i] = velocity[i]; 
            }
            acceHis[i][offSetCount[i]] = a[i];
            timerHis[i][offSetCount[i]] = (timer - prevTimer);
            prevAcce[i] = (prevAcce[i]*offSetCount[i] + a[i])/(offSetCount[i] + 1);
            offSetCount[i]++;
        } else {
            countCounter[i]++;
            if (countCounter[i] >= 3) {
                countCounter[i] = 0, offSetCount[i] = 0;
            }
        }
        if (offSetCount[i] > 5) {
            offSets[i] = -prevAcce[i];
            prevAcce[i] = 0;
            offSetCount[i] = 0;
            velocity[0] = 0.0, velocity[1] = 0.0, velocity[2] = 0.0;
            float correctDisplacement = (startVelocity[i] + 0.5*acceHis[i][0]*timerHis[i][0])*timerHis[i][0], tempVelocity = startVelocity[i] + acceHis[i][0]*timerHis[i][0];
            for (int j = 1; j < 5; j++) {
                correctDisplacement += (tempVelocity + 0.5*acceHis[i][j]*timerHis[i][j])*timerHis[i][j];
                tempVelocity += acceHis[i][j]*timerHis[i][j];
            }
            startVelocity[i] = 0.0, startCoord[i] = 0.0, startVelocity[i] = 0.0;
            for (int j = 1; j < 5; j++) {
                timerHis[i][j] = 0.0, acceHis[i][j] = 0.0;
            }
            coord[i] = correctDisplacement;
        }
    }
    
    for (int i = 0; i < 3; i++) {
        a[i] = a[i] + offSets[i];
    }
    
    Serial.println("=======================================================");
    Serial.print("accX : ");Serial.print(a[0]);
    Serial.print("        accY : ");Serial.print(a[1]);
    Serial.print("        accZ : ");Serial.println(a[2]);
    
    Serial.print("prevAccX : ");Serial.print(prevAcce[0]);
    Serial.print("        prevAccY : ");Serial.print(prevAcce[1]);
    Serial.print("        prevAccZ : ");Serial.println(prevAcce[2]);
    
    Serial.print("offSetsX : ");Serial.print(offSets[0]);
    Serial.print("        offSetsY : ");Serial.print(offSets[1]);
    Serial.print("        offSetsZ : ");Serial.println(offSets[2]);
    
    Serial.print("offSetCountX : ");Serial.print(offSetCount[0]);
    Serial.print("        offSetCountY : ");Serial.print(offSetCount[1]);
    Serial.print("        offSetCountZ : ");Serial.println(offSetCount[2]);
    
    Serial.print("direction: ");Serial.print(velocity[0]);Serial.print(" ");Serial.print(velocity[1]);Serial.print(" ");Serial.println(velocity[2]);
    Serial.print("Coordinates: ");Serial.print(coord[0]);Serial.print(" ");Serial.print(coord[1]);Serial.print(" ");Serial.println(coord[2]);
    Serial.print("time: ");Serial.println(timer);
    Serial.println("=======================================================\n");
    prevTimer = timer;
  }

}

bool offSetJudge(float mean, float input) {
    if (mean - input <= 0.3 && input > 1) {
        return true;
    } else if (mean - input <= 0.1 && input < 1 && input > 0.5) {
        return true;
    } else if (mean - input <= 0.05 && input < 0.5 && input > 0.2) {
        return true;
    } else if (mean - input <= 0.02 && input < 0.2 && input > 0.05) {
        return true;
    } else {
        return false;
    }
}
