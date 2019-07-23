int Ua = 0;  
int Up = 0;
int kp = 0; //ganancia proporcional 
int ki = 0; //ganancia integral
double Ea = 0; //error actual
int E = 0; /error 
int velAct = 0; //velocidad actual
int velRef = 0; //velocidad de referencia

double PIcontroll (velAct, velRef) {
  
  E = fabs(velRef - velAct);
  Ua = Up + (Ea * kp + (ki * TIME_STEP))) - (kp * E);
  Up = Ua;
  Ea = E;
  return Ua;
  }