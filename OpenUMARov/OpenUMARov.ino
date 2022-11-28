// Código Arduino para control en posición y velocidad y lectura de corrientes de los motores de un rover.

//Librerias
#include <TimerOne.h>
#include<avr/interrupt.h>
#include<avr/io.h>
#include <SoftwareSerial.h>

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

//KP = 3, KI = 30

//Definicion de las constantes para el PID 1
#define kp1 0.5
#define ki1 20
#define kd1 0

//Definicion de las constantes para el PID 2
#define kp2 0.5
#define ki2 20
#define kd2 0

//Definicion filtro digital exponencial
#define alpha 0.6


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
int volatile mn1Old;
int volatile mn1Oldest;

//Variables globales para PID 2
int volatile sn2;
int volatile en2;
int volatile enOld2;
int volatile mn2;
int volatile mn2Old;
int volatile mn2Oldest;

//Variables para establecer limites minimo y maximo de los PID (efecto wind-up)
int volatile snmin;
int volatile snmax;

//Referencia para los PID
//Estas referencia son el numero de pulsos que se deben de dar para mantener una velocidad constante
int volatile ref1;
int volatile ref2;

//Para la recepcion de las velocidades aportadas por el nodo de ROS

//En estos dos buffers se guardan los 4 bytes que llegan por puerto serie, los 2 primeros se guardan en buffIn_1 y se corresponden 
//a la velocidad de la rueda 1, y los 2 siguientes se guardan en buffIn_2 y se corresponden a la velocidad de la rueda 2.
byte buffIn_1 [2] = {0, 0};
byte buffIn_2 [2] = {0, 0};
//Variable auxiliar para juntar los 2 bytes para obtener la referencia de velocidad de las ruedas 1 y 2 respectivamente.
byte long auxi1;
byte long auxi2;
int g;



void setup()
{

  auxi1 = 0;
  auxi2 = 0;
  ref1 = 0;
  ref2 = 0;
  
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
  mn1Old = 0;
  mn1Oldest = 0;
  sn2 = 0;
  en2 = 0;
  enOld2 = 0;
  mn2 = 0;
  mn2Old = 0;
  mn2Oldest = 0;
  

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
  // Se definen los milisegundos maximos de espera de datos desde el puerto serie.
  Serial.setTimeout(1000);
  while (!Serial) {
    ; // Espera hasta que se conecte el puerto serie.
  }

}

void loop()
{
  //Espera hasta que halla alguna informacion disponible en el puerto serie.
  if (Serial.available())
  {
    //La funcion serial.readbytes lee datos del buffer serie y los guarda en dos variables buffer que tienen una capacidad de 2 bytes cada una.      
    Serial.readBytes(buffIn_1, 2);
    Serial.readBytes(buffIn_2, 2);
    // Espera de un segundo.
    // delay(100);

    //Espera que se envien los datos correctamente
    Serial.flush();

    //En la variable auxi1 guardamos los 2 bytes ubicados en buffIn_1, los cuales al agruparlos se corresponden con los pulsos que se tienen que dar
    //para tener la referencia de la velocidad seleccionada para el motor 1.
    auxi1 = (byte)buffIn_1[1] << 8;
    auxi1 += (byte)buffIn_1[0];
    //En la variable ref guardamos la variable auxi la cual es de tipo byte y la convertimos a tipo entero de 32 bits.
    ref1 = (long int)auxi1;
    //En la variable auxi2 guardamos los 2 bytes ubicados en buffIn_2, los cuales al agruparlos se corresponden con los pulsos que se tienen que dar
    //para tener la referencia de la velocidad seleccionada para el motor 2.
    auxi2 = (byte)buffIn_2[1] << 8;
    auxi2 += (byte)buffIn_2[0];
    //En la variable ref guardamos la variable auxi la cual es de tipo byte y la convertimos a tipo entero de 32 bits.
    ref2 = (long int)auxi2;

    //Envio por puerto serie a ROS2
    Serial.println(pulsos1);
    Serial.println(pulsos2);
    
    Serial.println(velpulsos1);
    Serial.println(velpulsos2); 
  
    Serial.println(ref1);
    Serial.println(ref2);
    
    // Serial.flush();

    // Se limpia el buffer, abriendo y cerrando el puerto serie.
    Serial.end();
    Serial.begin(115200);
    // Espera de un segundo
    delay(50);
  } 
  // Si no hay ninguna informacion disponible para ser recibida, se limpia el buffer y se espera hasta que llegue algun dato.
  else {
    // Se limpia el buffer, abriendo y cerrando el puerto serie.
    Serial.end();
    Serial.begin(115200);
    // Espera de un segundo.
    delay(100);  
  }
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
  
  //Filtro digital de suavizado
  //mn1 = alpha*mn1Old + (1-alpha)*mn1;
  mn1 = alpha*mn1 + alpha*(1-alpha)*mn1Old + (1-alpha)*(1-alpha) * mn1Oldest; //Debe tener alpha elevada en consecuencia
  //Se actualiza el error anterior por el actual.
  enOld1 = en1;
  mn1Oldest = mn1Old;
  mn1Old = mn1;  
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
  
  //Filtro digital de suavizado
  //mn2 = alpha*mn2Old + (1-alpha)*mn2; //Debe tener alpha baja en consecuencia
  mn2 = alpha*mn2 + alpha*(1-alpha)*mn2Old + (1-alpha)*(1-alpha) * mn2Oldest; //Debe tener alpha elevada en consecuencia
  //Se actualiza el error anterior por el actual.
  enOld2 = en2;
  mn2Oldest = mn2Old;
  mn2Old = mn2;  
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
