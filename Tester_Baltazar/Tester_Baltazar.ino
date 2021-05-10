//pines de control de velocidad y direccion motores

#define V_MRIGTH 5
#define D_MRIGTH 4
#define V_MLEFT 6
#define D_MLEFT 7

//pines de Lectura sensores
//sensores ultrasonicos

#define TRIGGER_F 11
#define ECHO_F 10
#define TRIGGER_B 9
#define ECHO_B 8
//sensores de contacto
#define CONTACT_R 13
#define CONTACT_L 12

//Constantes Cinematica del robot

#define PULSOSPORGIRO 300
#define PI 3.14159265358979323
#define RADIOROBOT 4.5   //medida en cm
#define RADIORUEDA 2.1   //medida en cm
#define TIEMPO_MUESTREO 50 //tiempo de muestreo en millis
#define ADVANCEOBST 0.05   //el robot avanza x mts

//CONSTANTES DE DIRECCIONES EN LOS SENSORES DE LUZ
#define LIGHT_FRONT 1
#define LIGHT_RIGHT 2
#define LIGHT_LEFT 3
#define LIGHT_BACK 4

//PINES DE INTERRUPCION PARA EL CANAL DEL ENCODER
int RH_ENCODER_A = 3;
int LH_ENCODER_A = 2;

//VARIABLES PARA LA LECTURA DEL SENSADO Y CALCULO DE POSICIONES Y VELOCIDADES
//contadores de pulsos en los encoders del motor.
//Seran de utilidad para determinar los movimientos del robot
volatile unsigned long left_count_ticks = 0;
volatile unsigned long right_count_ticks = 0;

//variables para el registro de posiciones de las ruedas del robot
//su utilidad sera para llevar a cabo el calculo de velocidades del robot
volatile unsigned long left_last_position = 0;
volatile unsigned long left_position = 0;
volatile unsigned long left_delta_position = 0;
volatile unsigned long right_position = 0;
volatile unsigned long right_last_position = 0;
volatile unsigned long right_delta_position = 0;

//variables que garantizan el tiempo de muestreo entre interrupciones
volatile unsigned _last_sample_time = 0;
volatile unsigned _sample_time = 0;
volatile unsigned _delta_time = 0;

//VARIABLES PARA CONTROLES DE LA SEÑAL PWM

//TODO: testear el uso de control PID y perfiles de velocidad para los giros y avances

//TEMP: variables de velocidad para probar los giros
int M_FRONT = 128;
int M_BACK = 128;
int M_STOP = 0;

//Variables y constantes para los valores de PWM
const int MAX_PWM_VAL = 255; //Limites para los valores del PWM 
const int MIN_PWM_VAL = 0;
int PWM_MOTOR_R = 60; //valores variables a partir del Control PID
int PWM_MOTOR_L = 60;

//VARIABLES PARA EL CALCULO DE LOS CONTROLES PID Y COMPARADORES DE VELOCIDADES

//variables para el Comparador de velocidades
double _offset = 0;                //valor de offset en la difereicia de velocidades
double _difVel = 0;                //salida integral de la diferecial de velocidades
const double _KIcomp = 0.0175;     //Constante de integracion para el comparador
double _errDifVel[2] = {0.0, 0.0}; //registro de errores en el comparador

//variables para el control PID de velocidad
double _kpv = 0.275;  //contante de proporcionalidad
double _kiv = 0.5; //constante de integracion
double _kdv = 0.275;  //contante de derivacion

const double VEL_DES = 60.0;

double _Rd[2] = {0.0, 0.0};           //Rn de la llanta derecha
double errRight[3] = {0.0, 0.0, 0.0}; //error en la velocidad de la llanta derecha

int _Ri[2] = {0.0, 0.0};             //Rn de la llanta izquierda
double errLeft[3] = {0.0, 0.0, 0.0}; //error de la velocidad en la llanta izquierda

//cadena de almacenaje y procesamiento de los comandos recibidos
String command;

void setup()
{
  Serial.begin(9600);
  //configuracion de los pines de entrada y salida
  pinMode(D_MRIGTH, OUTPUT);
  pinMode(D_MLEFT, OUTPUT);
  pinMode(CONTACT_R, INPUT);
  pinMode(CONTACT_L, INPUT);
  pinMode(TRIGGER_F, OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(TRIGGER_B, OUTPUT);
  pinMode(ECHO_B, INPUT);
  //configuracion de los pines de interrucion para los encoders
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, FALLING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, FALLING);

  analogWrite(V_MLEFT, 0);
  analogWrite(V_MRIGTH, 0);
}

//SENSADO DE DISTANCIAS

//Distancia del sensor ultrasonico
/**
 * @name: sensor_Sonar
 * @params: int: puerto Trigger, int: puerto Echo
 * @return: double: distance
 * @description: se reciben los pines del los puertos Trigger y Echo para ubicar al sensor ultrasonico
 * frontal o trasero para activarlo ,leer datos y calcular la distancia, la cual se retorna
*/
double sensor_Sonar(int Trigger, int Echo)
{
  //front
  double distance;
  long tiempo;

  //sound Sensors
  digitalWrite(Trigger, LOW); //signal to active the sensor
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH); //signal to active the sensor
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  tiempo = (pulseIn(Echo, HIGH));
  distance = float((tiempo / 2) / 29);

  return distance;
}

//Distancias de los sensores Intfrarrojos
/**
 * @name: read_IR_R y read_IR_L
 * @params: int: muestreo o taza de muestreo
 * @return: double: Distancia_cm
 * @description: Se recibe un valor entero, el cual representa una taza de muestreo con la cual
 * se utilizara para implementar un filtro de media y reducir el ruido en la lectura del sensor
 * infrarrojo. El valor generado por ese filtro se utilizara para calcular las distancias en cm
 * y se retorna
*/
double read_IR_R(int muestreo)
{
  int suma = 0;
  for (int i = 0; i < muestreo; i++)
  {
    suma = suma + analogRead(A0);
  }
  double adc = (suma / muestreo) * 0.0048828125;
  double Distancia_cm = 13 * pow(adc, -1);
  return Distancia_cm;
}

long read_IR_L(int muestreo)
{
  int suma = 0;
  for (int i = 0; i < muestreo; i++)
  {
    suma = suma + analogRead(A1);
  }
  float adc = (suma / muestreo) * 0.0048828125;
  float Distancia_cm = 13 * pow(adc, -1);
  return Distancia_cm;
}

/**
 * @function: g_seguidor
 * @param: void
 * @return: int
 * @description: Se encarga del sensado de la intensidad de luz en
 * alguna de las 4 direcciones definidas. Al encontrar la direccion
 * con mayor intesidad de luz retorna un numero que representa esa direccion
*/
int g_seguidor()
{
  //sensores
  double ldr_Front = 0.0;
  double ldr_Back = 0.0;
  double ldr_Left = 0.0;
  double ldr_Rigth = 0.0;

  ldr_Front = analogRead(A2); //blue
  ldr_Rigth = analogRead(A3); //yellow
  ldr_Back = analogRead(A4);  //orange
  ldr_Left = analogRead(A5);  //green

  if ((ldr_Front > ldr_Rigth) && (ldr_Front > ldr_Left) && (ldr_Front > ldr_Back))
    return LIGHT_FRONT;
  else if ((ldr_Rigth > ldr_Front) && (ldr_Rigth > ldr_Left) && (ldr_Rigth > ldr_Back))
    return LIGHT_RIGHT;
  else if ((ldr_Left > ldr_Front) && (ldr_Left > ldr_Rigth) && (ldr_Left > ldr_Back))
    return LIGHT_LEFT;
  else if ((ldr_Back > ldr_Front) && (ldr_Back > ldr_Rigth) && (ldr_Back > ldr_Left))
    return LIGHT_BACK;
  else
    return -1; //Error en la lectura;
}

/**
 * @function: findGoal
 * @param: int
 * @return: void
 * @description: recibe un numero entero que representa una direccion definida,
 * a partir de ella, decide a que direccion debe de girar el robot segun el caso
*/
void findGoal(int direccion)
{
  switch (direccion)
  {
  case LIGHT_FRONT:
    turn_X_degrees_R(0); //no gira, esta frente a la meta
    stop_Robot();
    break;
  case LIGHT_RIGHT:
    turn_X_degrees_R(getTicks(90)); //la meta esta a la derecha
    stop_Robot();
    break;
  case LIGHT_LEFT:
    turn_X_degrees_L(getTicks(90)); //la meta esta a la izquierda
    stop_Robot();
    break;
  case LIGHT_BACK:
    turn_X_degrees_R(getTicks(180)); //la meta esta atras
    stop_Robot();
    break;
  default:
    stop_Robot();
    break;
  }
}
//FUNCIONES DE LOS ENCODERS PARA EL SENSADO DE POSICIONES Y VELOCIDADES

// funciones que unicamente realizan el conteo de pulsos que generan los encoders en los motores
void leftEncoderEvent()
{
  left_count_ticks++; //acumulador de pulsos para calcular las distancias recorridas (reseteable)
  left_position++; //acumulador de pulsos para calcular las velocidades en cada rueda (no reseteable)
}

void rightEncoderEvent()
{
  right_count_ticks++;
  right_position++;
}

/**
 * @function: PID_Right
 * @params: vdes [velocidad deseada para el robot], vmed [velocidad medida por los encoders] 
 * @description: toma de referencia la velocidad que se desea alcanzar en cm/s, y tambien recibe
 * la velodcidad medida en tiempo real; a partir de esos datos, se implementa un control de velodad PID,
 * utilizando las ecuaciones propuestas y los Coeficientes definidos en el encabezado del programa
*/
int PID_Right(double vdes, double vmed)
{
  //corrimiento de los registros de error para el motor derecho;
  errRight[2] = errRight[1];
  errRight[1] = errRight[0];
  errRight[0] = vdes - vmed;

  //calculo de las componentes del control PID
  double proporcional = _kpv * (errRight[0] - errRight[1]);
  double integral = _kiv * ((errRight[0] + errRight[1]) / 2);
  double diferencial = _kpv * (errRight[0] - (2 * errRight[1]) + errRight[2]);

  //corrimiento de los registros de valores calculados para el control PID
  _Rd[1] = _Rd[0];
  _Rd[0] = _Rd[1] + (proporcional + integral + diferencial);

  /**
     * Se aplica un tope al pwm para mantenerlo en un rango y no meter
     * datos indeseables
    */
  if (_Rd[0] > MAX_PWM_VAL)
    _Rd[0] = MAX_PWM_VAL;

  if (_Rd[0] < MIN_PWM_VAL)
    _Rd[0] = MIN_PWM_VAL;

  return _Rd[0];
}
/**
 * @function: PID_Left
 * @params: vdes [velocidad deseada para el robot], vmed [velocidad medida por los encoders] 
 * @description: toma de referencia la velocidad que se desea alcanzar en cm/s, y tambien recibe
 * la velodcidad medida en tiempo real; a partir de esos datos, se implementa un control de velodad PID,
 * utilizando las ecuaciones propuestas y los Coeficientes definidos en el encabezado del programa
*/
int PID_Left(double vdes, double vmed)
{
  //corrimiento de los registros de error para el motor derecho;
  errLeft[2] = errLeft[1];
  errLeft[1] = errLeft[0];
  errLeft[0] = vdes - vmed;

  //calculo de las componentes del control PID
  double proporcional = _kpv * (errLeft[0] - errLeft[1]);
  double integral = _kiv * ((errLeft[0] + errLeft[1]) / 2);
  double diferencial = _kpv * (errLeft[0] - (2 * errLeft[1]) + errLeft[2]);

  //corrimiento de los registros de valores calculados para el control PID
  _Ri[1] = _Ri[0];
  _Ri[0] = _Ri[1] + (proporcional + integral + diferencial);

  /**
     * Se aplica un tope al pwm para mantenerlo en un rango y no meter
     * datos indeseables
    */
  if (_Ri[0] > MAX_PWM_VAL)
    _Ri[0] = MAX_PWM_VAL;

  if (_Ri[0] < MIN_PWM_VAL)
    _Ri[0] = MIN_PWM_VAL;

  return _Ri[0]; //retorna respuesta actual
}

/**
 * @function: get_speed_R
 * @params: none 
 * @return: double -> la velocidad medida en cm/s
 * @description: A partir de los valores sensados y guardados en los registros de posicion
 * y el tiempo de muestreo, Se calcula la velocidad de giro de las ruedas del motor medida en cm/s
*/
double get_speed_R()
{

  //calcular el diferencial de posicion
  right_delta_position = right_position - right_last_position;
  right_last_position = right_position;                                       //guardamos el registro de las posiciones de la rueda
  double frecuencia = (1000) / TIEMPO_MUESTREO;                               //muestreos en un segundo
  double wr = ((2 * PI * right_delta_position) / PULSOSPORGIRO) * frecuencia; //velocidad angular de la rueda r/s
  return RADIORUEDA * wr;                                                     //velociad lineal de la rueda en cm/s
}

/**
 * @function: get_speed_L
 * @params: none 
 * @return: double -> la velocidad medida en cm/s
 * @description: A partir de los valores sensados y guardados en los registros de posicion
 * y el tiempo de muestreo, Se calcula la velocidad de giro de las ruedas del motor medida en cm/s
*/
double get_speed_L()
{

  //calcular el diferencial de posicion
  left_delta_position = left_position - left_last_position;
  left_last_position = left_position;                                        //guardamos el registro de las posiciones de la rueda
  double frecuencia = (1000) / TIEMPO_MUESTREO;                              //muestreos en un segundo
  double wr = ((2 * PI * left_delta_position) / PULSOSPORGIRO) * frecuencia; //velocidad angular de la rueda r/s
  return RADIORUEDA * wr;                                                    //velociad lineal de la rueda en cm/s
}

/**
 * Function: rotation_left_45
 * params: None
 * return: None
*/
void rotation_left_45()
{
  left_count_ticks = 0;
  right_count_ticks = 0;
  const int tope = 30;

  while (right_count_ticks <= tope)
  {
    analogWrite(V_MRIGTH, M_FRONT);
    analogWrite(V_MLEFT, M_FRONT);
    digitalWrite(D_MRIGTH, LOW);
    digitalWrite(D_MLEFT, LOW);
  }
}

void rotation_right_45()
{
  left_count_ticks = 0;
  right_count_ticks = 0;
  const int tope = 30;

  while (left_count_ticks <= tope)
  {
    analogWrite(V_MLEFT, M_FRONT);
    analogWrite(V_MRIGTH, M_FRONT);
    digitalWrite(D_MLEFT, HIGH);
    digitalWrite(D_MRIGTH, HIGH);
  }
}

void advance_cm()
{
  left_count_ticks = 0;
  right_count_ticks = 0;
  const int tope = 23;

  while (left_count_ticks < tope)
  {
    analogWrite(V_MLEFT, M_FRONT);
    analogWrite(V_MRIGTH, M_FRONT);
    digitalWrite(D_MRIGTH, LOW);
    digitalWrite(D_MLEFT, HIGH);
  }
}

/**
 * @function: getTicksShift
 * @param: float meters: la cantidad en metros que debe de avanzar el robot
 * @return: null
 * @description: Recibe la distancia en metros que debe de avanzar el robot, y
 * genera el numero de pulsos que debe de contar el encoder para avanzar la distancia deseada.
*/
int getTicksShift(float meters)
{

  int pulsos = 0;

  if (meters == 0)
    return 0;

  pulsos = (int)((PULSOSPORGIRO * meters) / (2 * PI * (RADIORUEDA)/100));
  //Serial.println(pulsos);
  return pulsos;
}

/**
 * @function: advance_x_cm
 * @param: int ticks -> El numero de pulsos que debe de sensar en el encoder
 * @return: null
 * @description: Hace un desplazamiento del robot hacia adelante, segun los centimetros indicados
 * por el numero de pulsos que recibe. Ademas implementa el control PID de velocidad
*/
void advance_x_cm(int ticks)
{
  left_count_ticks = 0;
  right_count_ticks = 0;

  while (left_count_ticks <= ticks || right_count_ticks <= ticks)
  {

    _sample_time = millis();
    _delta_time = (double)_sample_time - _last_sample_time;

    digitalWrite(D_MRIGTH, LOW);
    digitalWrite(D_MLEFT, HIGH);
    analogWrite(V_MLEFT, PWM_MOTOR_L);
    analogWrite(V_MRIGTH, PWM_MOTOR_R);
    if (_delta_time >= TIEMPO_MUESTREO)
    {
      //Extraccion de las velocidades
      double vel_right = get_speed_R();
      double vel_left = get_speed_L(); 
      //Comparador de velocidades para asgurar el movimiento RLU del Robot
      _errDifVel[1] = _errDifVel[0];
      _errDifVel[0] = _offset - (vel_right - vel_left); //diferencial objetivo - diferencial real
      _difVel = ((_errDifVel[1] + _errDifVel[0]) / 2) * _KIcomp;
      
      /* Serial.print("V_RIGHT ");
      Serial.print(vel_right);
      Serial.print(" V_LEFT ");
      Serial.print(vel_left);
      Serial.print(" Dif_Vel ");
      Serial.print(_difVel);
      Serial.print(" PWD_L ");
      Serial.print(PWM_MOTOR_L);
      Serial.print(" PWD_R ");
      Serial.println(PWM_MOTOR_L); */

      //aplicacion del control de velocidad en las ruedas
      PWM_MOTOR_R = PID_Right(VEL_DES, vel_right) + _difVel;
      PWM_MOTOR_L = PID_Left(VEL_DES, vel_left) - _difVel;

      _last_sample_time = _sample_time;
    }
  }
}

/**
 * @function: back_x_cm
 * @param: int ticks -> El numero de pulsos que debe de sensar en el encoder
 * @return: null
 * @description: Hace un desplazamiento del robot hacia atras, segun los centimetros indicados
 * por el numero de pulsos que recibe. Ademas implementa el control PID de velocidad
*/

void back_x_cm(int ticks)
{
  left_count_ticks = 0;
  right_count_ticks = 0;

  while (left_count_ticks <= ticks || right_count_ticks <= ticks)
  {

    _sample_time = millis();
    _delta_time = (double)_sample_time - _last_sample_time;

    digitalWrite(D_MRIGTH, HIGH);
    digitalWrite(D_MLEFT, LOW);
    analogWrite(V_MLEFT, M_FRONT);
    analogWrite(V_MRIGTH, M_FRONT);
    if (_delta_time >= TIEMPO_MUESTREO)
    {
      double vel_right = get_speed_R();
      double vel_left = get_speed_L(); 
      //Comparador de velocidades para asgurar el movimiento RLU del Robot
      _errDifVel[1] = _errDifVel[0];
      _errDifVel[0] = _offset - (vel_right - vel_left); //diferencial objetivo - diferencial real
      _difVel = ((_errDifVel[1] + _errDifVel[0]) / 2) * _KIcomp;

      PWM_MOTOR_R = PID_Right(VEL_DES, vel_right);
      PWM_MOTOR_L = PID_Left(VEL_DES, vel_left);
      _last_sample_time = _sample_time;
    }
  }
}

/**
 * @function: evadeObs
 * @param: int ticks -> El numero de pulsos que debe de sensar en el encoder
 * @return: null
 * @description: funcion provicional que le indica al robot la distancia que debe de recorrer
 * para esquivar el obstaculo una ves que lo encuentre
*/
void evadeObs(int ticks){
  left_count_ticks = 0;
  right_count_ticks = 0;

  while (left_count_ticks <= ticks || right_count_ticks <= ticks)
  {
    digitalWrite(D_MRIGTH, LOW);
    digitalWrite(D_MLEFT, HIGH);
    analogWrite(V_MLEFT, PWM_MOTOR_L);
    analogWrite(V_MRIGTH, PWM_MOTOR_R);
  }
}

void reset_Values_PID()
{
  left_position = 0;
  left_delta_position = 0;
  left_last_position = 0;
  right_position = 0;
  right_last_position = 0;
  right_delta_position = 0;
  _sample_time = 0;
  _last_sample_time = 0;
  _delta_time = 0;
  PWM_MOTOR_L = 60;
  PWM_MOTOR_R = 60;
  errLeft[0] = errLeft[1] = errLeft[2] = 0;
  errRight[0] = errRight[1] = errRight[2] = 0;
  _errDifVel[0] = _errDifVel[1] = 0;
  _difVel = 0;
  _Rd[0] = _Rd[1] = 0.0;
  _Ri[0] = _Ri[1] = 0.0;
}

void stop_Robot()
{
  reset_Values_PID();
  analogWrite(V_MLEFT, M_STOP);
  analogWrite(V_MRIGTH, M_STOP);
}

/**
 * function: back_cm
 * description: the robot back off x cm
 * param: none
 * return: void
*/
void back_cm()
{
  left_count_ticks = 0;
  right_count_ticks = 0;
  const int tope = 23;

  while (left_count_ticks < tope)
  {
    analogWrite(V_MLEFT, M_BACK);
    analogWrite(V_MRIGTH, M_BACK);
    digitalWrite(D_MRIGTH, HIGH);
    digitalWrite(D_MLEFT, HIGH);
  }
}

void advance_obs()
{
  left_count_ticks = 0;
  right_count_ticks = 0;
  const int tope = 115;

  while (left_count_ticks < tope)
  {
    analogWrite(V_MLEFT, M_FRONT);
    analogWrite(V_MRIGTH, M_FRONT);
  }
}

int getTicks(float angulo)
{

  int _ticks = (int)(0.66667 * angulo);
  return _ticks;
}

void turn_X_degrees_R(int pulsos)
{
  left_count_ticks = 0;
  right_count_ticks = 0;
  int tope = pulsos;

  while (right_count_ticks <= tope)
  {
    digitalWrite(D_MRIGTH, HIGH);
    digitalWrite(D_MLEFT, HIGH);
    analogWrite(V_MRIGTH, M_FRONT);
    analogWrite(V_MLEFT, M_FRONT);
  }
}

void turn_X_degrees_L(int pulsos)
{
  left_count_ticks = 0;
  right_count_ticks = 0;
  int tope = pulsos;

  while (left_count_ticks <= tope)
  {

    digitalWrite(D_MRIGTH, LOW);
    digitalWrite(D_MLEFT, LOW);
    analogWrite(V_MLEFT, M_FRONT);
    analogWrite(V_MRIGTH, M_FRONT);
  }
}

void loop()
{

  stop_Robot();

  if (Serial.available() > 0)
  {
    command = Serial.readStringUntil('\n');
    if (command == "sonar_f")
    {
      Serial.println(sensor_Sonar(TRIGGER_F, ECHO_F));
    }
    else if (command == "sonar_b")
    {
      Serial.println(sensor_Sonar(TRIGGER_B, ECHO_B));
    }
    else if (command == "IR_R")
    {
      Serial.println(read_IR_R(50));
    }
    else if (command == "IR_L")
    {
      Serial.println(read_IR_L(50));
    }
    else if (command == "findGoal")
    {
      findGoal(g_seguidor());
    }
    else if (command == "bumper_R")
    {
      Serial.println(digitalRead(CONTACT_R));
    }
    else if (command == "bumper_L")
    {
      Serial.println(digitalRead(CONTACT_L));
    }
    else if (command == "turn_R")
    {
      rotation_right_45();
    }
    else if (command == "turn_L")
    {
      rotation_left_45();
    }
    else if (command == "evadeObs")
    {
      evadeObs(getTicksShift(ADVANCEOBST));
    }
    else if (command == "back")
    {
      back_cm();
    }
    else if (command == "stop")
    {
      stop_Robot();
    }

    //turn_D xxx
    else if (command.length() > 6)
    {
      String cmd_dir = command.substring(0, 7);
      String s_ang = command.substring(7, command.length());
      float unidad = s_ang.toFloat();

      if (cmd_dir == "turn_L ")
      {
        turn_X_degrees_L(getTicks(unidad));
      }
      else if (cmd_dir == "turn_R ")
      {
        turn_X_degrees_R(getTicks(unidad));
      }
      else if (cmd_dir == "advmts ")
      {
        advance_x_cm(getTicksShift(unidad));
      }
      else if (cmd_dir == "bckmts ")
      {
        back_x_cm(getTicksShift(unidad));
      }
    }
  }
}