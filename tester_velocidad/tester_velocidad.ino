//pines de control de velocidad y direccion motores
#define V_MRIGTH 5
#define D_MRIGTH 4
#define V_MLEFT 6
#define D_MLEFT 7

//Constantes Cinematica del robot
#define PULSOSPORGIRO 300
#define PI 3.14159265358979323
#define RADIOROBOT 4.5     //medida en m
#define RADIORUEDA 2.1     //medida en m
#define TIEMPO_MUESTREO 50 //tiempo de muestreo en millis

int RH_ENCODER_A = 3;
int LH_ENCODER_A = 2;

//variables de velocidad
const int MAX_PWM_VAL = 255; //Tope de los Valores Maximos para el PDW
const int MIN_PWM_VAL = 0; //Tope de los Valores Maximos para el PDW
int PWM_MOTOR_R = 60;
int PWM_MOTOR_L = 60;

//variables contadoras de ticks en el encoder
volatile unsigned long left_count_ticks = 0;
volatile unsigned long right_count_ticks = 0;

//variables para el registro de posiciones de las ruedas del robot
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
//importante: ajustar las constantes _kp _ki _kd cada que se modifique el valor de la velocidad Deseada para mejorar la respuesta del Control PID
double _kpv = 0.275;  //contante de proporcionalidad
double _kiv = 0.5; //constante de integracion
double _kdv = 0.275;  //contante de derivacion

const double VEL_DES = 60.0;

double _Rd[2] = {0.0, 0.0};           //Rn de la llanta derecha
double errRight[3] = {0.0, 0.0, 0.0}; //error en la velocidad de la llanta derecha

int _Ri[2] = {0.0, 0.0};             //Rn de la llanta izquierda
double errLeft[3] = {0.0, 0.0, 0.0}; //error de la velocidad en la llanta izquierda

void setup()
{
    Serial.begin(9600);
    pinMode(D_MRIGTH, OUTPUT);
    pinMode(D_MLEFT, OUTPUT);

    //configuracion de los pines de interrucion para los encoders
    attachInterrupt(0, leftEncoderEvent, FALLING);
    attachInterrupt(1, rightEncoderEvent, FALLING);
}

// encoder event for the interrupt call
void rightEncoderEvent()
{

    right_count_ticks++;
    right_position++; //actualizo la posicion
}

// encoder event for the interrupt call
void leftEncoderEvent()
{

    left_count_ticks++;
    left_position++;
}

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

double get_speed_R()
{

    //calcular el diferencial de posicion
    right_delta_position = right_position - right_last_position;
    right_last_position = right_position;                                       //guardamos el registro de las posiciones de la rueda
    double frecuencia = (1000) / TIEMPO_MUESTREO;                               //muestreos en un segundo
    double wr = ((2 * PI * right_delta_position) / PULSOSPORGIRO) * frecuencia; //velocidad angular de la rueda r/s
    return RADIORUEDA * wr;                                                     //velociad lineal de la rueda en cm/s
}

double get_speed_L()
{

    //calcular el diferencial de posicion
    left_delta_position = left_position - left_last_position;
    left_last_position = left_position;                                        //guardamos el registro de las posiciones de la rueda
    double frecuencia = (1000) / TIEMPO_MUESTREO;                              //muestreos en un segundo
    double wr = ((2 * PI * left_delta_position) / PULSOSPORGIRO) * frecuencia; //velocidad angular de la rueda r/s
    return RADIORUEDA * wr;                                                    //velociad lineal de la rueda en cm/s
}

void prototipoAdvance(int pulsos)
{

    left_count_ticks = 0;
    right_count_ticks = 0;

    while (left_count_ticks <= pulsos || right_count_ticks <= pulsos)
    {

        _sample_time = millis();                                //extraemos los millis que lleva activo el MIC
        _delta_time = (double)_sample_time - _last_sample_time; //obtenemos el perido que lleva activo desde el ultimo registro guardado

        digitalWrite(D_MRIGTH, LOW);
        digitalWrite(D_MLEFT, HIGH);
        analogWrite(V_MRIGTH, PWM_MOTOR_R);
        analogWrite(V_MLEFT, PWM_MOTOR_L);

        //*si el periodo que lleva activo corresponde al tiempo de muestreo que deseamos obtener, Se activa el Control PID
        //si aun no alcanzamos el tiempo de muestreo entonces se ejecutna las demas funciones del robot
        if (_delta_time >= TIEMPO_MUESTREO)
        {
            
            //Solo se debe de invocar una unica vez a las funciones para calcular la velocidad
            double vel_right = get_speed_R();
            double vel_left = get_speed_L();

            //left_delta_position = left_position - left_last_position;
            //Serial.print(' ');
            Serial.print(vel_right);
            Serial.print(' ');
            Serial.print(vel_left);
            Serial.print(' ');
            Serial.println(VEL_DES);

            PWM_MOTOR_R = PID_Right(VEL_DES, vel_right);
            PWM_MOTOR_L = PID_Left(VEL_DES, vel_left);

            _last_sample_time = _sample_time;
        }
    }
}

void prototypestop()
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
    PWM_MOTOR_L = 0;
    PWM_MOTOR_R = 0;
    errLeft[0] = errLeft[1] = errLeft[2] = 0;
    errRight[0] = errRight[1] = errRight[2] = 0;
    _Rd[0] = _Rd[1] = 0.0;
    _Ri[0] = _Ri[1] = 0.0;

    digitalWrite(D_MRIGTH, LOW);
    digitalWrite(D_MLEFT, HIGH);
    analogWrite(V_MRIGTH, PWM_MOTOR_R);
    analogWrite(V_MLEFT, PWM_MOTOR_L);
    delay(1000);
}

void loop()
{
    prototypestop();
    prototipoAdvance(10000);
    prototypestop();
}
