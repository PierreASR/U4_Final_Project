#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 64

int Ua = 0;  //velocidad actual
int Ua2 = 0;
int Up = 0;  //velocidad anterior
int Up2 = 0;
double kp = 5.7; //ganancia proporcional
double kp2 = 6;
double ki = 0.0; //ganancia integral
double ki2 = 0.0;
double Ea = 0; //error acumulado
double Ea2 = 0;
double Ep = 0; //error
double Ep2 = 0;
double posAct = 0.0; //pos actual
double posAct2 = 0.0;
double posRef = -1.5709; //pos de referencia
double posRef2 = -1.5709;
double pos_value;  //lectura del sensor
double velocity1 = 0; //variable velocidad
double velocity2 = 0;

double PIcontroll2 (double posRef, double posAct) {
  Ea2 = fabs(posRef - posAct);
  Ua2 = Up2 + (Ea2 * (kp2 + (ki2 * (TIME_STEP/1000)))) - (kp2 * Ep2);
  Up2 = Ua2;
  Ep2 = Ea2;
  printf("error%lf\t error acumulado%lf\t \n", Ep, Ea);
  if (posRef < 0){
    Ua2 = Ua2 *(-1);
  }
  return Ua2;
}

double PIcontroll (double posRef, double posAct) {
  Ea = fabs(posRef - posAct);
  Ua = Up + (Ea * (kp + (ki * (TIME_STEP/1000)))) - (kp * Ep);
  Up = Ua;
  Ep = Ea;
  printf("error%lf\t error acumulado%lf\t \n", Ep, Ea);
  if (posRef < 0){
    Ua = Ua *(-1);
  }
  return Ua;
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag torso = wb_robot_get_device("motor1");
  WbDeviceTag hombro = wb_robot_get_device("motor2");
  WbDeviceTag codo = wb_robot_get_device("motor3");
  WbDeviceTag wrist = wb_robot_get_device("motor4");

  WbDeviceTag posicion1 = wb_robot_get_device("encoder1");
  WbDeviceTag posicion2 = wb_robot_get_device("encoder2");

  wb_position_sensor_enable(posicion1, TIME_STEP);
  wb_motor_set_position(torso, INFINITY);
  wb_position_sensor_enable(posicion2, TIME_STEP);
  wb_motor_set_position(hombro, INFINITY);

  while (wb_robot_step(TIME_STEP) != -1) {

    posAct = wb_position_sensor_get_value(posicion1);

    if (velocity1 > 0) {
      ki = 5.7;
    }

    velocity1 = PIcontroll(posRef, posAct);
    if (velocity1 > wb_motor_get_max_velocity(torso)){
        velocity1 = wb_motor_get_max_velocity(torso);
    }

    ///////////////////////////////////////////
    if (velocity2 > 0) {
      ki2 = 5.9;
    }

    velocity2 = PIcontroll2(posRef2, posAct2);
    if (velocity2 > wb_motor_get_max_velocity(hombro)){
      velocity2 = wb_motor_get_max_velocity(hombro);
    }

    //velocity2 = Motor_direc(velocity2, wb_position_sensor_get_value(posicion2), a);

    posAct2 = wb_position_sensor_get_value(posicion2);
    printf("posicion %lf\t posicion2 %lf\n", posAct, posAct2);
    printf("velocity1 %lf\t velocity2 %lf\t\n", velocity1, velocity2);
    wb_motor_set_velocity(torso, velocity1);
    wb_motor_set_velocity(hombro, velocity2);
    wb_motor_set_position(codo, 0);
    wb_motor_set_position(wrist, 0);
  };

  wb_robot_cleanup();

  return 0;
}
