/*
SKETCH PARA MODULO A TODAS VELOCIDADES DE LA EXPOSICION PURO SWING
Adaptado para su uso con Shield Pendulo (version inicial)

Version 26 de septiembre 2014:
.- Añadido el control de amplitud.
.- Corregido el calculo de desfase de la rueda. Se regula con el potenciometro central, unido a A1
.- Extra Ton sigue ajustandose con A0

Disponible en GitHub: https://github.com/manolomira/PuroSwing-Pendule

Basado en sistema de pendulo perpetuo
28-Noviembre-2013

Sensores de barrera optica para el pendulo a trav´s de placa 
    amplificadora
Sensor Hall para la rueda

Salida PWM para controlar un modulo basado en controlador en puente L298
*/


#include <Bounce.h>
#include <Wire.h> 
#include <Metro.h>


#define _7SEG (0x38)   /* I2C address for 7-Segment */


// *************** VARIABLES DEL PENDULO ***************

int amplitud, amplitud_max= 38;

// Define entradas y salidas
const int SWInner = 2;       // Entrada de la barrera optica de referencia 
                             // del pendulo
const int LEDInner = 3;      // Indica que se activo la barrera optica de 
                             // referencia del pendulo

const int FinIman = 4;       // Entrada de la barrera optica que indica que 
                             // el pendulo esta cerca del electroiman

const int ElectMag = 5;      // Salida que activa el electriman

// La SALIDA   6 esta reservada para indicar que se activo la barrera FinIman
// La ENTRADA  7 esta reservada como libre1 en el conector SENS_PEND

const int LED_Enclavado = 9; // Salida que indica modo enclavado (L1)

const int LEDElectMag = 11;  // ****** PROVISIONAL *********
                             // Indica que el electroiman esta activado

// la ENTRADA 12 esta reservada como libre2 en el conector SENS_PEND

const int pot1 = A0;         // potenciometro ajuste Extra TON (ANULADO)
const int pot2 = A1;         // potenciometro ajuste Extra desfase
const int pot3 = A2;         // P3 (sin uso)
const int eAnalog = A3;      // Entrada analogica de ajuste
// Las entradas analogicas A4 y A5 se reservan para su uso como bus I2C

const long intervaloSeguridad = 1000;        // tiempo maximo que se activa el electroiman por seguridad

Bounce SWInnerBounce = Bounce( SWInner,50 ); // Referencia del pendulo
Bounce FinImanBounce = Bounce( FinIman,50 ); // Cerca del electroiman

long milliseconds;
long tiempoActual;          // OBSOLETA
long tiempoAnterior;          // OBSOLETA
long tiempoOn;          // OBSOLETA
long tiempoOff;          // OBSOLETA
  
int periodoPendulo;
int intervaloActual;     // OBSOLETA
int intervaloAnterior;    // OBSOLETA
int extraTon;              // OBSOLETA            // tiempo extra calculado a partir del potenciometro

unsigned long T_Pendulo;


String texto;



// ************* Variables display 7 Segmentos ***********

float pendiente2;
float t;
float vsin;
float pi=3.141;
float T=1.7;
int potR=A1;  //Entrada analogica 1, la A0 la usa el pendulo
int valorpot=0;
float valor_anterior;
float valor_media; /*Actualizo pantalla cada .5ms y calculo la media de 50 valores*/
int i;
float A=0.320;  /*Amplitud, 2A= 64cm*/

Metro lectura = Metro(500); /* Visualizamos cada 0.5s el valor del potenciometro */
Metro media_valor = Metro(10); /*Leemos pot cada 10ms. para hacer la media*/

const byte NumberLookup[20] =   {0x3F,0x06,0x5B,0x4F,0x66,
                                 0x6D,0x7D,0x07,0x7F,0x6F,0xBF,
                                 0x86,0x77,0x7C,0x39,0x5E,0x79,0x71,
                                 0x80,0x40};

/***************************************************************************
 Function Name: setup

 Purpose: 
   Initialize hardwares.
****************************************************************************/




void setup()
{
// *************** VARIABLES DEL PENDULO ***************
  pinMode(SWInner, INPUT);        // optoacoplador de llegada al REED
  pinMode(FinIman,INPUT);         // optoacoplador de llegada al electroiman

  pinMode(LEDInner, OUTPUT);      // indica la activacion del del REED
  pinMode(LEDElectMag, OUTPUT);   // indica la activacion del electroiman
  pinMode(ElectMag, OUTPUT);      // Salida electroiman
  pinMode(LED_Enclavado, OUTPUT); // Salida que indica modo enclavado (L1)
  
  
  Serial.begin(115200);   
  
  
    
    // *************** DEL 7SEG ***************
 
    Wire.begin();        /* Join I2C bus */
    Serial.begin(9600); 
  
    pendiente2=(T/4)/(499);  /*Relacion entre t y el valor del potenciometro*/
    valorpot=analogRead(potR);
  
    delay(500);          /* Allow system to stabilize */
  
    /* Configure 7-Segment to 12mA segment output current, Dynamic mode, 
     and Digits 1, 2, 3 AND 4 are NOT blanked */
     
    Wire.beginTransmission(_7SEG);    
    Wire.write(0);
    Wire.write(B01000111);
    Wire.endTransmission();

}


//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
void loop()
{
   
  // *************** GESTION DEL PENDULO ***************

  
  SWInnerBounce.update ();       // Actualiza las instalacias Bounce del pendulo y detecta flancos de bajada.
  FinImanBounce.update (); 

  ActualizaPendulo();            // Actualiza las variables asociadas al pendulo y enciende sus actuadores


  // *************** GESTION DEL 7SEG ***************
  
  if (lectura.check() == 1)  /*lectura es objeto tipo metro, activa cada 0.5s*/
    {
      
        valor_anterior=analogRead(potR);
        Serial.print(int(valor_anterior), DEC);
        Serial.print(" ");
        valor_anterior=valor_anterior-23;
        if(valor_anterior<0) valor_anterior=0;
        valor_media = 0.0;
        i=0;
        while(i<50)
        {
            if(media_valor.check()==1) /*Activa cada 10ms*/
            {
                valor_media=valor_anterior+valor_media;
                valor_anterior=analogRead(potR);
                valor_anterior=valor_anterior-23;
                if(valor_anterior<0) valor_anterior=0;
                i++;
            }
        }
        valor_media=valor_media/50; /*Tengo 50 valores leyendo cada 10ms.(en 500ms)*/
        valorpot=int (valor_media);
        Serial.print(valorpot, DEC);
        Serial.print(" ");
        //Serial.print(valor_media, 2);
        //Serial.print(" ");
        if(valorpot>499) 
        {
            valorpot=999-valorpot;
            if(valorpot<0) valorpot=0;
            
        }   
        
        
        
        t=valorpot*pendiente2;
        vsin=(A*2*pi/T)*sin(2*pi*t/T);
        Serial.print(valorpot,DEC);
        Serial.print("   ");
        Serial.print(" Velocidad : ");
        Serial.print(vsin,3);
        Serial.println();
    
        

            Send7SEG(4, NumberLookup[int(vsin)%10+10]); /* Ponemos el . decimal*/
            Send7SEG(3, NumberLookup[int(vsin*10)%10]);
            Send7SEG(2, NumberLookup[int(vsin*100)%10]);
            Send7SEG(1, NumberLookup[int(vsin*1000)%10]);
            
   
    }





}
  

//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
void ActualizaPendulo()
{
  
  static int zona;
         // zona indica en que zona del recorrido se encuentra el pendulo
         // zona = 0 es la zona opuesta al sensor de control
         // zona = 1 es desde el paso por el centro hasta el sensor de control
         //          calcula los parametros e imprime resultados en esta
         //          zona para evitar retrasar el apagado del pendulo
         // zona = 2 es la zona mas alla del sensor de control
         // zona = 3 es la zona entre el sensor de control y el centro
         //          EN ESTA ZONA SE PRODUCE EL ACTIVAMIENTO DEL ELECTROIMAN
  
  static unsigned long T_0, T_1, T_2, T_3;                   // marca de tiempo de entrada en cada zona
  static unsigned long T_ON, T_Seguridad, T_OFF;             // marca de tiempo para los eventos eventos del pendulo
  int extraT_ON;
 
  static int T_zona_0, T_zona_1, T_zona_2, T_zona_3;         // tiempo en cada zona (milisegundos)
  
  unsigned long milisegundos;                                // Recoge el valor de millis() en cada llamada a la funcion
  milisegundos = millis();
  
  boolean pulsoControl = SWInnerBounce.fallingEdge();      // Detecta flancos de bajada en los detectores opticos.
  boolean pulsoCentro  = FinImanBounce.fallingEdge();


  // *** APAGA EL ELECTROIMAN Y EL LED DE SEÑALIZACION ASOCIADO ***
  if (milisegundos > min (T_Seguridad, T_OFF)) digitalWrite (ElectMag, LOW);                         // Desconecta el electroiman
  if (milisegundos > (T_ON + 15))              digitalWrite (LEDElectMag, LOW);
 
  
  // *** ejecuta las transiciones entre los zonas en funcion de los detectores opticos
  //     en cada cambio de zona registra la marca de tiempo de salida y el tiempo que permanecio en ella
  //     limita el valor registrado de permanencia en cada zona a 10.000 por conpatibilidad con el tipo int 

  switch (zona) {
    case 0:
      if (pulsoCentro)
        {T_1 = milisegundos;  T_zona_0 = min (10000, T_1 - T_0); zona = 1;
        
         // *** HACE LOS CALCULOS EN ESTA ZONA PARA NO RETRASAR EL APAGADO DEL ELECTRIMAN *** 
         // *** CALCULO DE VARIABLES ASOCIADAS AL PENDULO ***

         amplitud = 0.0002 * T_zona_2 * T_zona_2 - 0.1472 * T_zona_2 + 49.125;
         periodoPendulo = T_zona_0 + T_zona_1 + T_zona_2 + T_zona_3;
         T_Pendulo = T_0;            // corresponde al momento del paso por el centro en direccion 
                                     // contraria a la zona del sensor de control


         // *** IMPRESION DE VALORES ASOCIADOS AL PENDULO ***
/*
         Serial.println("--------------------------------");
         Serial.print ("|  PerPend = ");     Serial.print (T_zona_0 + T_zona_1 + T_zona_3);
         Serial.print (" + ");               Serial.print (T_zona_2);
         Serial.print (" = ");               Serial.print (periodoPendulo);

         Serial.print ("\tTpendulo = ");     Serial.print (T_Pendulo);

         Serial.print ("\tT_iman =  ");        Serial.print (T_zona_3);
         Serial.print (" +");               Serial.print (extraT_ON);

         Serial.print ("\t >> amplitud = "); Serial.print (amplitud);
         Serial.print (" (");                Serial.print (amplitud_max);
         Serial.print (")");

         Serial.println ();
*/


         texto = "|  PerPend = " + String(T_zona_0 + T_zona_1 + T_zona_3) + " + " + T_zona_2;
         texto = texto + " = " + periodoPendulo;

         texto = texto + "\tTpendulo = " + T_Pendulo;

         texto = texto + "\tT_iman =  " + T_zona_3 + " +" + extraT_ON;

         texto = texto + "\t >> amplitud = " + amplitud + " (" + amplitud_max + ")";


         Serial.println("--------------------------------");
         Serial.println (texto);
         Serial.println ("--------------------------------");

         Serial.print   ("tiempo de impresion = ");
         Serial.println (millis () - T_1);
      }
      break;
 
    case 1:
      if (pulsoControl)
        {T_2 = milisegundos;  T_zona_1 = min (10000, T_2 - T_1); zona = 2;}
      break;
 
    case 2:
      if (pulsoControl) 
        {T_3 = milisegundos;  T_zona_2 = min (10000, T_3 - T_2); zona = 3;        

        // *** AL ENTRAR EN ZONA 3 SE PRODUCE EL ACTIVAMIENTO DEL ELECTROIMAN  ***
        T_Seguridad = T_3 + 1000;             // establece el maximo tiempo encendido del electroiman
        T_OFF       = T_Seguridad;            // establece provisionalmente el momento de apagado del electroiman

       if (amplitud < amplitud_max)          // solo enciende electroiman si la amplitud es menor de la establecida
        {
          digitalWrite (ElectMag, HIGH);      // Conecta el electroiman y LEDElectroMag como control
          digitalWrite (LEDElectMag, HIGH);
        }
      }
      break;
      
    case 3:
      if (pulsoCentro)
      { T_0 = milisegundos;  T_zona_3 = min (10000, T_0 - T_3); zona = 0;


       // *** AL SALIR DE LA ZONA 3 SE CALCULA EL MOMENTO DE APAGADO DEL ELECTROIMAN ***
       extraT_ON = T_zona_3 / 6;
       extraT_ON = map (analogRead (pot1), 0, 1023, 100, 0);
       T_OFF     = T_0 + extraT_ON;                            // establece el valor correcto para T_OFF

     }
       break;
    default: 
      zona = 0;  // si por algun motivo la variable se sale de margenes vuelve a la zona 0
  }
}




/***************************************************************************
 Function Name: Send7SEG

 Purpose: 
   Send I2C commands to drive 7-segment display.
****************************************************************************/

void Send7SEG (byte Digit, byte Number)
{
  Wire.beginTransmission(_7SEG);
  Wire.write(Digit);
  Wire.write(Number);
  Wire.endTransmission();
}  







