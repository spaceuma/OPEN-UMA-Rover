// Código Arduino para control en posición y velocidad y lectura de corrientes de los motores de un rover.

//Librerias
#include <TimerOne.h>
#include<avr/interrupt.h>
#include<avr/io.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Definición GPS
TinyGPSPlus gps;
static const int RXPin = A8; //Este va conectado al pin TX del GPS
static const int TXPin= A9; //Este va conectado al pin RX del GPS
static const int GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);

//Definicion de los pines del encoder 1
#define encoder1PinA  2
#define encoder1PinB  3

//Definicion de los pines del encoder 2
#define encoder2PinA 21
#define encoder2PinB 20

//Definicion de los pines donde esta conectado la señal PWM en arduino y puentes H
#define pinPWM1 9
#define pinPWM2 6

//Definicion de los pines para la direccion de las ruedas (PWM)
#define  dirPWM1 8
#define  dirPWM2 7


//Defincion de los pines donde se encuentran los sensores de corriente.
const int sensorCurrent1 = A1;
const int sensorCurrent2 = A2;


//Definicion de las constantes para el PID 1
#define kp1 3
#define ki1 30
#define kd1 0

//Definicion de las constantes para el PID 2
#define kp2 3
#define ki2 30
#define kd2 0


//Variables del GPS
float lat;
float lng;
float meters;
float course;
float speed;
float day;
float month;
float year;
float hour;
float minute;
float second;
float sat;
float h;


//Corriente -- variable para lectura de corriente
float lectura_analog1;
float lectura_analog2;


//PWM -- variable para designar valor al PWM
byte valorPWM1;
byte valorPWM2;



//Variables para interrupciones
//Guarda el numero de pulsos de la rueda 1
float pulsos1;
//Guarda el numero de pulsos de la rueda 1, 10 ms antes que es el tiempo al cual esta configurada la interrupcion
float pulsos1Ant;
//Diferencia de pulsos que se han dado en los 10 ms (velpulsos=pulsos-pulsosAnt)
float velpulsos1;
//Guarda el numero de pulsos de la rueda 2
float pulsos2;
//Guarda el numero de pulsos de la rueda 2, 10 ms antes que es el tiempo al cual esta configurada la interrupcion
float pulsos2Ant;
//Diferencia de pulsos que se han dado en los 10 ms (velpulsos=pulsos-pulsosAnt)
float velpulsos2;



//Designacion del Tiempo interrupcion temporizada 10ms
unsigned long tiempoint = 10000;


//Variables globales para PID 1
int volatile sn1;
int volatile en1;
int volatile enOld1;
int volatile mn1;


//Variables globales para PID 2
int volatile sn2;
int volatile en2;
int volatile enOld2;
int volatile mn2;

//Variables para establecer limites minimo y maximo de los PID (efecto wind-up)
int volatile snmin;
int volatile snmax;

//Referencia para los PID
//Estas referencia son el numero de pulsos que se deben de dar para mantener una velocidad constante
int volatile ref1;
int volatile ref2;

//Para la recepcion de las velocidades aportadas por simulink

//En este buffer se guardan 4 bytes, los 2 primeros se corresponden a la velocidad de la rueda 1 y los 2 siguientes a la velocidad de la rueda 2.
byte buffIn [4] = {0, 0, 0, 0};
//Variable auxiliar para juntar los 2 bytes para obtener la referencia de velocidad de las ruedas 1 y 2 respectivamente.
byte long auxi1;
byte long auxi2;
int g;



void setup()
{
  //Inicialización de las variables GPS
  lat=0;
  lng=0;
  meters=0;
  course=0;
  speed=0;
  day=0;
  month=0;
  year=0;
  hour=0;
  minute=0;
  second=0;
  sat=0;
  h=0;

  
  //Inicializacion de las variables
  pulsos1 = 0;
  pulsos1Ant = 0;
  velpulsos1 = 0;
  pulsos2 = 0;
  pulsos2Ant = 0;
  velpulsos2 = 0;


  //Designacion del valor inicial del PWM (0 -- 0%, 64 -- 25%, 127 -- 50%, 191 -- 75%, 255 -- 100%)
  valorPWM1 = 0;
  valorPWM2 = 0;


  //Inicializacion de las variables de los PID
  sn1 = 0;
  en1 = 0;
  enOld1 = 0;
  mn1 = 0;
  sn2 = 0;
  en2 = 0;
  enOld2 = 0;
  mn2 = 0;

  //Establecimiento de unos limites ya que el motor tiene una velocidad maxima dada por sus caracteristicas.
  snmin= -600;
  snmax= 600;



  //INTERRUPCIONES EXTERNAS -- ENCODERS -- CONTADOR DE PULSOS
  //Configuracion como pines de entrada los pines donde estan los canales A y B del encoder 1.
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);

  //Configuracion como pines de entrada los pines donde estan los canales A y B del encoder 2.
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);


  //Configuracion de las interrupciones del encoder 1, las cuales se han configurado para que salte en disparo de subida y bajada.
  attachInterrupt(0, doEncoder1A, CHANGE);
  attachInterrupt(1, doEncoder1B, CHANGE);

  //Configuracion de las interrupciones del encoder 2, las cuales se han configurado para que salte en disparo de subida y bajada.
  attachInterrupt(2, doEncoder2A, CHANGE);
  attachInterrupt(3, doEncoder2B, CHANGE);


  //Pines de lectura de corriente como entradas.
  pinMode(sensorCurrent1, INPUT);
  pinMode(sensorCurrent2, INPUT);



  //INTERRUPCION TIEMPO (TIMER1) -- CALCULO VELOCIDAD
  //Configuracion interrupcion temporizada para calculo de la velocidad.
  //Intervalo de tiempo de disparo en microsegundos.
  Timer1.initialize(tiempoint);

  //Attach de la interrupcion con el servicio de gestion
  Timer1.attachInterrupt(ISR_cVelocidad);




  //PWM -- TRANSMITE POTENCIA Y DIRECCION
  //Configuracion como salida los pines de direccion de las ruedas.
  pinMode(dirPWM1, OUTPUT);
  pinMode(dirPWM2, OUTPUT);
  //Configuración de velocidad inicial deseada de las ruedas en funcion del valorPWM
  analogWrite(pinPWM1, valorPWM1);
  analogWrite(pinPWM2, valorPWM2);


  //Habilitacion de interrupciones
  //Habilita todas las interrupciones a nivel de programacion del microcontrolador.
  sei();
  //Habilita todas las interrupciones a nivel de programación de arduino.
  interrupts();
  //Inicializacion y Configuracion de la velocidad del puerto serie, la cual se ha establecido a 9600 bits/s.
  Serial.begin(115200);
  ss.begin(GPSBaud);
  while (!Serial) {
    ; // Espera hasta que se conecte el puerto serie.
  }

}

void loop()
{
  //Espera hasta que halla 4 bytes para ser recibidos (2 bytes por cada velocidad de rueda)
  if (Serial.available() > 3)
  {
    h=2;
    lat=gps.location.lat(); // Latitud
    lng=gps.location.lng(); // Longitud
    meters=gps.altitude.meters(); // Altura
    course=gps.course.deg(); // Orientación
    speed=gps.speed.kmph(); // Velocidad
    day=gps.date.day(); // Día
    month=gps.date.month(); // Mes
    year=gps.date.year(); // Año
    hour=gps.time.hour(); // Hora
    hour=hour+h;
    minute=gps.time.minute(); // Minuto
    second=gps.time.second(); // Segundo
    sat=gps.satellites.value();
    //La funcion serial.readbytes lee datos del buffer serie y los guarda en una variable buffer la cual tiene una capacidad de 4 bytes.
    Serial.readBytes(buffIn, 4);

    //Limpieza del buffer
    Serial.flush();

    //En la variable auxi1 guardamos los 2 primeros bytes ubicados en buffIn, los cuales al agruparlos se corresponden con los pulsos que se tienen que dar
    //para tener la referencia de la velocidad seleccionada para el motor 1.
    auxi1 = (byte)buffIn[1] << 8;
    auxi1 += (byte)buffIn[0];
    //En la variable ref guardamos la variable auxi la cual es de tipo byte y la convertimos a tipo entero de 32 bits.
    ref1 = (long int)auxi1;
    //En la variable auxi2 guardamos los 2 ultimos bytes ubicados en buffIn, los cuales al agruparlos se corresponden con los pulsos que se tienen que dar
    //para tener la referencia de la velocidad seleccionada para el motor 2.
    auxi2 = (byte)buffIn[3] << 8;
    auxi2 += (byte)buffIn[2];
    //En la variable ref guardamos la variable auxi la cual es de tipo byte y la convertimos a tipo entero de 32 bits.
    ref2 = (long int)auxi2;
    //Lectura de la corriente de los motores
    lectura_analog1 = analogRead(sensorCurrent1);
    lectura_analog2 = analogRead(sensorCurrent2);

    //Envio de datos al simulink (escritura por puerto serie)
    Serial.write((byte*)&pulsos1, 4);
    delay(5);
    Serial.write((byte*)&pulsos2, 4);
    delay(5);
    Serial.write((byte*)&velpulsos1, 4);
    delay(5);
    Serial.write((byte*)&velpulsos2, 4);
    delay(5);
    Serial.write((byte*)&ref1, 4);
    delay(5);
    Serial.write((byte*)&ref2, 4);
    delay(5);
    Serial.write((byte*)&lat, 4);
    delay(5);
    Serial.write((byte*)&lng, 4);
    delay(5);
    Serial.write((byte*)&meters, 4);
    delay(5);
    Serial.write((byte*)&course, 4);
    delay(5);
    Serial.write((byte*)&speed, 4);
    delay(5);
    Serial.write((byte*)&day, 4);
    delay(5);
    Serial.write((byte*)&month, 4);
    delay(5);
    Serial.write((byte*)&year, 4);
    delay(5);
    Serial.write((byte*)&hour, 4);
    delay(5);
    Serial.write((byte*)&minute, 4);
    delay(5);
    Serial.write((byte*)&second, 4);
    delay(5);
    Serial.write((byte*)&sat,4);
    delay(5);
    Serial.flush();
    smartDelay(1000);
  }
}

//Función smartDelay para GPS. Busca datos para el GPS mientras Arduino no hace nada
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read()); 
  } while (millis() - start < ms);
}

//Interrupciones encoder 1
//Canal A
void doEncoder1A()
{
  sei();
  interrupts();

  if (bit_is_set(PINE, 4))
  {
    if (bit_is_set(PINE, 5))
    {
      pulsos1--;
    }
    else
    {
      pulsos1++;
    }
  }
  else
  {
    if (bit_is_set(PINE, 5))
    {
      pulsos1++;
    }
    else
    {
      pulsos1--;
    }
  }
}

//Canal B
void doEncoder1B()
{
  sei();
  interrupts();
  if (bit_is_set(PINE, 5))
  {
    if (bit_is_set(PINE, 4))
    {
      pulsos1++;
    }
    else
    {
      pulsos1--;
    }
  }
  else
  {
    if (bit_is_set(PINE, 4))
    {
      pulsos1--;
    }
    else
    {
      pulsos1++;
    }
  }
}

//Interrupciones encoder 2
//Canal A
void doEncoder2A()
{
  sei();
  interrupts();
  if (bit_is_set(PIND, 0))
  {
    if (bit_is_set(PIND, 1))
    {
      pulsos2++;
    }
    else
    {
      pulsos2--;
    }
  }
  else
  {
    if (bit_is_set(PIND, 1))
    {
      pulsos2--;
    }
    else
    {
      pulsos2++;
    }
  }
}

//Canal B
void doEncoder2B()
{
  sei();
  interrupts();
  if (bit_is_set(PIND, 1))
  {
    if (bit_is_set(PIND, 0))
    {
      pulsos2--;
    }
    else
    {
      pulsos2++;
    }
  }
  else
  {
    if (bit_is_set(PIND, 0))
    {
      pulsos2++;
    }
    else
    {
      pulsos2--;
    }
  }
}



//Interrupcion para obtencion de la velocidad (salta cada 10ms)
void ISR_cVelocidad()
{
  sei();
  interrupts();

  //Calculo la diferencia de pulsos que se dan en 10 ms en la rueda 1.
  velpulsos1 = pulsos1 - pulsos1Ant;
  //Se actualiza el valor de los pulsos que se llevan.
  pulsos1Ant = pulsos1;

  //Calculo la diferencia de pulsos que se dan en 10 ms en la rueda 2.
  velpulsos2 = pulsos2 - pulsos2Ant;
  //Se actualiza el valor de los pulsos que se llevan.
  pulsos2Ant = pulsos2;


  //Calculo del error.
  en1 = ref1 - (velpulsos1);
  en2 = ref2 - (velpulsos2);
  //Llamamiento a las funciones PID.
  PIDControl1();
  PIDControl2();
  //Llamamiento a las funciones que actuan sobre los motores.
  actuador1();
  actuador2();


}


//Funcion PID 1
void PIDControl1()
{
  sei();
  interrupts();

  //Calculo del sumatorio del error.
  sn1 = sn1 + en1;

  //Condicionales por si el error sobresale de los limites de velocidad maxima del motor 1.
  if (sn1 > snmax)
  {
    sn1 = snmax;
  }
  else if (sn1 < snmin)
  {
    sn1 = snmin;
  }
  //Calculo del valor PWM1.
  mn1 = (kp1 * en1) + ((ki1 * sn1) / 100) + (kd1 * (en1 - enOld1));
  //Se actualiza el error anterior por el actual.
  enOld1 = en1;
}

//Funcion para mandar la potencia y direccion al motor 1.
void actuador1()
{
  sei();
  interrupts();
  //Condicional por si sobrepasa los limites de velocidad máxima.
  if (mn1 > 255 || mn1 < -255)
  {
    valorPWM1 = 80;
    //Escribe en los pines del PWM el valor seleccionado.
    analogWrite(pinPWM1, valorPWM1);
  }
  else
  {
    //En caso de no superar la velocidad máxima establece el valor de PWM correspondiente a la velocidad objetivo.
    valorPWM1 = (unsigned char)fabs(mn1);
    //Escribe en los pines del PWM el valor seleccionado.
    analogWrite(pinPWM1, valorPWM1);
  }
  //Se manda la direccion del PWM1
  if (mn1 >= 0)
  {
    //Si el valor de PWM a establecer es positivo, envia una señal verdadera para sentido positivo.
    digitalWrite(dirPWM1, HIGH);
  }
  else
  {
    //En caso contrario si el valor de PWM a establecer es negativo, envia una señal falsa para sentido negativo.
    digitalWrite(dirPWM1, LOW);
  }
}


//Funcion PID 2
void PIDControl2()
{
  sei();
  interrupts();
  //Calculo del sumatorio del error.
  sn2 = sn2 + en2;

  //Condicionales por si el error sobresale de los limites de velocidad maxima del motor 2.
  if (sn2 > snmax)
  {
    sn2 = snmax;
  }
  else if (sn2 < snmin)
  {
    sn2 = snmin;
  }
  
  //Calculo del valor PWM.
  mn2 = (kp2 * en2) + ((ki2 * sn2) / 100) + (kd2 * (en2 - enOld2));
  //Se actualiza el error anterior por el actual.
  enOld2 = en2;
}

//Funcion para mandar la potencia y direccion al motor 2.
void actuador2()
{
  sei();
  interrupts();
  //Condicional por si sobrepasa los limites de velocidad máxima.
  if (mn2 > 255 || mn2 < -255)
  {
    valorPWM2 = 80;
    //Escribe en los pines del PWM el valor seleccionado.
    analogWrite(pinPWM2, valorPWM2);
  }
  else
  {
    //En caso de no superar la velocidad máxima establece el valor de PWM correspondiente a la velocidad objetivo.
    valorPWM2 = (unsigned char)fabs(mn2);
    //Escribe en los pines del PWM el valor seleccionado.
    analogWrite(pinPWM2, valorPWM2);
  }
  //Mandamos la direccion del PWM2
  if (mn2 >= 0)
  {
    //Si el valor de PWM a establecer es positivo, envia una señal verdadera para sentido positivo.
    digitalWrite(dirPWM2, HIGH);
  }
  else
  {
    //En caso contrario si el valor de PWM a establecer es negativo, envia una señal falsa para sentido negativo.
    digitalWrite(dirPWM2, LOW);
  }
}
