//pines de control de velocidad y direccion motores
#define V_MRIGTH 5
#define D_MRIGTH 4
#define V_MLEFT 6
#define D_MLEFT 7
//Constantes Cinematica del robot
#define PULSOSPORGIRO 300
#define PI 3.14159265358979323
#define RADIOROBOT 0.045   //medida en cm
#define RADIORUEDA 0.021   //medida en cm
#define TIEMPO_MUESTREO 50 //tiempo de muestreo en millis
#define ADVANCENORMAL 0.01 //el robot avanza x mts
#define ADVANCEOBST 0.10   //el robot avanza x mts

//pines de Lectrua sensores
//sensores ultrasonicos
#define TRIGGER_F 11
#define ECHO_F 10
#define TRIGGER_B 9
#define ECHO_B 8
//sensores de contacto
#define CONTACT_R 13
#define CONTACT_L 12

//constante de direccion
#define LIGHT_FRONT 1
#define LIGHT_RIGHT 2
#define LIGHT_LEFT 3
#define LIGHT_BACK 4

int RH_ENCODER_A = 2;
int LH_ENCODER_A = 3;

//variables de velocidad
int M_FRONT = 128;
int M_STOP = 0;
int PWM_MOTOR_R = 60;
int PWM_MOTOR_L = 60;

//constantes para comparar la proximidad de obstaculos
float DIST_PERP = 12.5; //Distancias con obstaculos que son perpendiculares al centro del robot
float DIST_AGDS = 15.0; //Distancias con obstaculos que estan en angulos agudos respecto al centro del robot

//variables para el Comparador de velocidades
double _offset = 0;                //valor de offset en la difereicia de velocidades
double _difVel = 0;                //salida integral de la diferecial de velocidades
const double _KIcomp = 0.0175;     //Constante de integracion para el comparador
double _errDifVel[2] = {0.0, 0.0}; //registro de errores en el comparador

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

//variables para el control PID de velocidad
double _kpv = 0.275;  //contante de proporcionalidad
double _kiv = 0.125; //constante de integracion
double _kdv = 0.01;  //contante de derivacion

const double VDES = 45.0;

double _Rd[2] = {0.0, 0.0};           //Rn de la llanta derecha
double errRight[3] = {0.0, 0.0, 0.0}; //error en la velocidad de la llanta derecha

int _Ri[2] = {0.0, 0.0};             //Rn de la llanta izquierda
double errLeft[3] = {0.0, 0.0, 0.0}; //error de la velocidad en la llanta izquierda

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
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
}

//Funciones para la lectura de sensores
float sensor_Sonar(int Trigger, int Echo)
{
  //front
  float distance;
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

float read_IR_R(int muestreo)
{
  int suma = 0;
  for (int i = 0; i < muestreo; i++)
  {
    suma = suma + analogRead(A0);
  }
  float adc = (suma / muestreo) * 0.0048828125;
  float Distancia_cm = 13 * pow(adc, -1);
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
    break;
  case LIGHT_RIGHT:
    turn_X_degrees_R(getTicks(90)); //la meta esta a la derecha
    break;
  case LIGHT_LEFT:
    turn_X_degrees_L(getTicks(90)); //la meta esta a la izquierda
    break;
  case LIGHT_BACK:
    turn_X_degrees_R(getTicks(180)); //la meta esta atras
    break;
  default:
    stop_Robot();
    break;
  }
}

// encoder event for the interrupt call
void leftEncoderEvent()
{
  right_count_ticks++;
  right_position++;
}

// encoder event for the interrupt call
void rightEncoderEvent()
{

  left_count_ticks++;
  left_position++;
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

  while ((right_count_ticks <= pulsos) || (left_count_ticks <= pulsos))
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

  while ((right_count_ticks <= pulsos) || (left_count_ticks <= pulsos))
  {

    digitalWrite(D_MRIGTH, LOW);
    digitalWrite(D_MLEFT, LOW);
    analogWrite(V_MLEFT, M_FRONT);
    analogWrite(V_MRIGTH, M_FRONT);
  }
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
  if (_Rd[0] > 128)
    _Rd[0] = 128;

  if (_Rd[0] < 0)
    _Rd[0] = 0;

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
  if (_Ri[0] > 128)
    _Ri[0] = 128;

  if (_Ri[0] < 0)
    _Ri[0] = 0;

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
int getTicksShift(float meters)
{

  int pulsos = 0;

  if (meters == 0)
    return 0;

  pulsos = (int)((PULSOSPORGIRO * meters) / (2 * PI * RADIORUEDA));

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
      //Comparador de velocidades para asgurar el movimiento RLU del Robot
      _errDifVel[1] = _errDifVel[0];
      _errDifVel[0] = _offset - (get_speed_R() - get_speed_L()); //diferencial objetivo - diferencial real
      _difVel = ((_errDifVel[1] + _errDifVel[0]) / 2) * _KIcomp;
      //aplicacion del control de velocidad en las ruedas
      PWM_MOTOR_R = PID_Right(53.0, get_speed_R());
      PWM_MOTOR_L = PID_Left(53.0, get_speed_L());
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
    analogWrite(V_MLEFT, PWM_MOTOR_L);
    analogWrite(V_MRIGTH, PWM_MOTOR_R);
    if (_delta_time >= TIEMPO_MUESTREO)
    {
      PWM_MOTOR_R = PID_Right(53.0, get_speed_R());
      PWM_MOTOR_L = PID_Left(53.0, get_speed_L());
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
    analogWrite(V_MLEFT, M_FRONT);
    analogWrite(V_MRIGTH, M_FRONT);
  }
}

/**
 * @function: reset_Values_PID
 * @param: void
 * @return: void
 * @description: funcion que reinicia los valores de todos los registros de error, registros de
 * posicion, muestreos de tiempo y valores del PWM una vez que el robot se detiene
*/
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

/**
 * @function: stop_Robot
 * @param: void
 * @return: void
 * @description: detiene el avance del robot y resetea los valores de los sensores y actuadores
*/
void stop_Robot()
{
  reset_Values_PID();
  analogWrite(V_MLEFT, 0);
  analogWrite(V_MRIGTH, 0);
  delay(500);
}

void loop()
{

  //states controller
  int state = 0;
  //variables to save the measures taken for sensors
  float distance_front = 0.0;
  float distance_back = 0.0;
  float IR_R = 0.0;
  float IR_L = 0.0;

  //execute robot's behaivors
  while (true)
  {
    //get measures sensors
    distance_front = sensor_Sonar(TRIGGER_F, ECHO_F);
    distance_back = sensor_Sonar(TRIGGER_B, ECHO_B);
    IR_L = read_IR_L(20);
    IR_R = read_IR_R(20);

    //Finite Machine State
    switch (state)
    {
    case 0:
      findGoal(g_seguidor());
      //detecta un obstaculo a la derecha
      if (IR_R <= DIST_AGDS)
      {
        stop_Robot();
        state = 1;
      }
      else
      {
        if (IR_L <= DIST_AGDS)//detecta un obstaculo a la izquierda
        {
          stop_Robot();
          state = 2;
        }
        else
        {
          if (distance_front <= DIST_PERP) //detecta un obstaculo al frente
          {
            stop_Robot();
            state = 3;
          }else
          {
            if (distance_back <= DIST_PERP)//detecta un obstaculo atras
            {
              stop_Robot();
              state = 4;
            }
            
          }
          
        }
      }
      advance_x_cm(getTicksShift(ADVANCENORMAL));
      break;
    case 1:
      Serial.println("EVADE OBSTACLE AT THE RIGHT");
      turn_X_degrees_L(getTicks(45));
      stop_Robot();
      evadeObs(getTicksShift(ADVANCEOBST));
      stop_Robot();
      state = 0;
      break;
    case 2:
      Serial.println("EVADE OBSTACLE AT THE LEFT");
      turn_X_degrees_R(getTicks(45));
      stop_Robot();
      evadeObs(getTicksShift(ADVANCEOBST));
      stop_Robot();
      state = 0;
      break;
    case 3:
      Serial.println("EVADE OBSTACLE IN FRONT");
      turn_X_degrees_R(getTicks(90));
      stop_Robot();
      evadeObs(getTicksShift(ADVANCEOBST));
      stop_Robot();
      state = 0;
      break;
    case 4:
      Serial.println("EVADE OBSTACLE AT BACK");
      turn_X_degrees_R(getTicks(90));
      stop_Robot();
      evadeObs(getTicksShift(ADVANCEOBST));
      stop_Robot();
      state = 0;
    }
    /* Serial.print("IR_R: ");
    Serial.println(IR_R);
    Serial.print("IR_L: ");
    Serial.println(IR_L);
    Serial.print("Sonar_F: ");
    Serial.println(distance_front);
    Serial.print("Sonar_B: ");
    Serial.println(distance_back);*/
    //delay(250);
  } //end while
}