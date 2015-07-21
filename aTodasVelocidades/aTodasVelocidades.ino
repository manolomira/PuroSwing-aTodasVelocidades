/*
SKETCH PARA MODULO A TODAS VELOCIDADES DE LA EXPOSICION PURO SWING
Adaptado para su uso con Shield Pendulo (version inicial)



Disponible en GitHub: https://github.com/manolomira/PuroSwing-aTodasVelocidades

Basado en sistema de pendulo perpetuo por bombeo actualizado en 2015

Sensores de barrera optica para el pendulo a traves de placa amplificadora
*/



int maxEncoder = 160;   // indica el valor maximo del recorrido del encoder
int amplitud_max= 38;   // amplitud maxima de bombeo del pendulo




#include <Bounce.h>
#include <Wire.h> 

// Define entradas y salidas
const int SWInner  = 7;      // Pin de la barrera optica lateral
  Bounce SWInnerBounce = Bounce( SWInner,50 ); // Referencia del pendulo

const int FinIman  = 4;      // Pin de la barrera optica central
  Bounce FinImanBounce = Bounce( FinIman,50 ); // Cerca del electroiman


const int encoderPinA =  2;  // Entrada A del encoder
const int encoderPinB = 13;  // Entrada B del encoder
const int resetPin    = 12;  // Entrada de reset del contador del encoder


const int LEDInner = 3;      // LED asociado a la barrera lateral
const int ElectMag = 5;      // Pin asociado al electroiman
const int LEDElectMag = 11;  // LED asociado al electroiman

const int pot1 = A0;         // potenciometro ajuste Extra TON
const int pot2 = A1;         // potenciometro ajuste Extra desfase
const int pot3 = A2;         // P3 (sin uso)
const int eAnalog = A3;      // Entrada analogica de ajuste


// La SALIDA   6 esta reservada para indicar que se activo la barrera FinIman
// La ENTRADA  7 esta reservada como libre1 en el conector SENS_PEND
// la ENTRADA 12 esta reservada como libre2 en el conector SENS_PEND


int amplitud;
int periodoPendulo;
unsigned long T_Pendulo;
String texto;


volatile  int encoderPos = 0;
int encoderPosAnt;
float velocidad;
float posicion;


// ************* Variables display 7 Segmentos ***********
#define _7SEG (0x38)   /* I2C address for 7-Segment */
  // valores de los 9 digitos sin y con punto decimal
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

  Serial.begin(57600); 

  Serial.println ("Iniciando conexion con I2C y esbleciendo parametros del display");


    Wire.begin();        /* Join I2C bus */
    delay(500);          /* Allow system to stabilize */
  
    /* Configure 7-Segment to 12mA segment output current, Dynamic mode, 
     and Digits 1, 2, 3 AND 4 are NOT blanked */     
    Wire.beginTransmission(_7SEG);    
    Wire.write(0);
    Wire.write(B01000111);
    Wire.endTransmission();
    
    
// *************** CONFIGURA ENTRADAS Y SALIDAS ***************
  pinMode(SWInner, INPUT);        // barrera optica lateral
  pinMode(FinIman,INPUT);         // barrera optica central
  pinMode(ElectMag, OUTPUT);      // Salida que activa el electroiman

  pinMode(LEDInner, OUTPUT);      // indica la activacion dela barrera lateral
  pinMode(LEDElectMag, OUTPUT);   // indica la activacion del electroiman


  pinMode(encoderPinA, INPUT); 
  digitalWrite(encoderPinA, HIGH);       // turn on pullup resistor
  pinMode(encoderPinB, INPUT); 
  digitalWrite(encoderPinB, HIGH);       // turn on pullup resistor

  pinMode(resetPin, INPUT);       // barrera de centro del recorrido del cursor

  attachInterrupt(0, doEncoder, RISING);  // encoder pin on interrupt 0 - pin 2


  Serial.println("Set Up Completo");                // a personal quirk
}




/***************************************************************************
 Function Name: loop

 Purpose: 
   lazo infinito del programa.
****************************************************************************/

void loop()
{
  // Actualiza las instalacias Bounce del pendulo y detecta flancos de bajada.
    SWInnerBounce.update ();
    FinImanBounce.update ();

  // Actualiza las variables asociadas al pendulo y enciende sus actuadores
    ActualizaPendulo();
    
  // Actualiza el valor de la velocidad si se movio el cursor
  if (encoderPos != encoderPosAnt)
  {
    encoderPos = max(encoderPos, - maxEncoder);
    encoderPos = min(encoderPos,   maxEncoder);
    
    posicion  = encoderPos;
    velocidad = posicion;
 
    
    
    encoderPosAnt = encoderPos;
 
    Serial.print(" encoder : ");
    Serial.print(encoderPos, DEC);
 
    Serial.print("\t");
    Serial.print(" posicion cursor : ");
    Serial.print(posicion,DEC);
        
    Serial.print("\t");
    Serial.print(" velocidad : ");
    Serial.print(velocidad,3);

    Serial.println();
    
    Serial.print (" numero de bits enviados a Wire= ");
    Serial.println ( Escribe7SEG (velocidad));
  }
}


/***************************************************************************
 Function Name: ActualizaPendulo

 Purpose: 
   Controla el mecanismo de bombeo electromagnetico del pendulo.
   Dede ser llamada en cada ciclo de loop
****************************************************************************/

void ActualizaPendulo()
{
  /* Controla el funcionamiento del bombeo del pendulo por medio de dos sensores. Los sensores 
  estan filtrados por medio de instancias de la libreria Bounce.
    - SWInnerBounce controla el paso a pocos cent´metros del centro en la zona en la que se 
      activa el electroiman. 
    - FinImanBounce esta en el centro del recorrido y controla la desconexion del electroiman.
    
  El programa detecta la secuencia centro -> centro -> lado -> lado -> ... vuelta al principio. 
 
    - zona = 0 es la zona opuesta al sensor de control. Al entrar en esta zona se desconecta el
               electroiman. 
    - zona = 1 desde el paso por el centro hasta la llegada al sensor de control. En esta zona 
               relativamente amplia se hacen los calculo para no interferir con la deteccion de 
               las barreras opticas.
    - zona = 2 es la zona mas alla del sensor de control. El timepo en esta zona se usa para 
               estimar la amplitud del movimiento del pendulo.
    - zona = 3 es la zona entre el sensor de control y el centro. EN ESTA ZONA SE PRODUCE EL 
               ACTIVAMIENTO DEL ELECTROIMAN
  
  Cada zona tiene asignadas variables T_x que registran el valor de millis() en el momento de 
  entrada en la zona, y T_zonax que registra el tiempo de permanencia en cada zona.
  T_ON y T_OFF registran el momento de conexion y desconexion del electroiman
  extraT_ON    es el tiempo que permanece el electroiman conectado una vez dentro de la zona 0
  T_Seguridad  es el tiempo maximo de conexion del electroiman
  
  */
  
  static int zona, zonaAnterior; 
  static unsigned long T_0, T_1, T_2, T_3; 
  static int T_zona_0, T_zona_1, T_zona_2, T_zona_3; 
  static unsigned long T_ON, T_Seguridad, T_OFF; 
  int extraT_ON;
  unsigned long milisegundos;  

      // Recoge el valor de millis() en cada llamada a la funcion
  milisegundos = millis();
  
      // Detecta flancos de bajada en los detectores opticos.
  boolean pulsoControl = SWInnerBounce.fallingEdge();
  boolean pulsoCentro  = FinImanBounce.fallingEdge();


  
  // *** ejecuta las transiciones entre los zonas en funcion de los detectores opticos
  //     en cada cambio de zona registra la marca de tiempo de salida y el tiempo que 
  //     permanecio el pendulo en la zona. Limita el valor registrado de permanencia en cada 
  //     zona a 10.000 por conpatibilidad con el tipo int 

  switch (zona) {

    case 0:
      //zona = 0 es la zona opuesta al sensor de control. 
           
      // *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
           //         Al entrar en esta zona se establece el momento de desconexion del electroiman
           if (zonaAnterior != zona)
           {
             // extraT_ON = T_zona_3 / 6;
             extraT_ON = map (analogRead (pot1), 0, 1023, 100, 0);
        
             // establece el valor  para T_OFF
             T_OFF     = T_0 + extraT_ON;
           }
 
      // *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
           //         De esta zona se sale con un pulso del detector central
           if (pulsoCentro)
           {
             T_1 = milisegundos;  
             T_zona_0 = min (10000, T_1 - T_0); 
             zona = 1;
           }
     break;


 
     case 1:
       // zona = 1 desde el paso por el centro hasta la llegada al sensor de control. 
               
       // *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
       // Al entrar en esta zona se establece el momento de desconexion del electroiman
           if (zonaAnterior != zona)
           {
             amplitud = 0.0002 * T_zona_2 * T_zona_2 - 0.1472 * T_zona_2 + 49.125;
             periodoPendulo = T_zona_0 + T_zona_1 + T_zona_2 + T_zona_3;
             T_Pendulo = T_0;            // corresponde al momento del paso por el centro en direccion 
                                         // contraria a la zona del sensor de control


             texto = "|  PerPend = " + String(T_zona_0 + T_zona_1 + T_zona_3) + " + " + T_zona_2;
             texto = texto + " = " + periodoPendulo;

             texto = texto + "\tTpendulo = " + T_Pendulo;

             texto = texto + "\tT_iman =  " + T_zona_3 + " +" + extraT_ON;

             texto = texto + "\t >> amplitud = " + amplitud + " (" + amplitud_max + ")";


             Serial.println("--------------------------------");
             Serial.println (texto);
             Serial.print ("    pot 1= "); Serial.println (analogRead (pot1)); 
             Serial.println ("--------------------------------");

             Serial.print   ("tiempo de impresion = ");
             Serial.println (millis () - T_1);
           }


      // *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
      // De esta zona se sale con un pulso del detector lateral
           if (pulsoControl)
           {
             T_2 = milisegundos; 
             T_zona_1 = min (10000, T_2 - T_1); 
             zona = 2;
           }
       break;
 
    case 2:
    // zona = 2 es la zona mas alla del sensor de control. El timepo en esta zona se usa para 
    // estimar la amplitud del movimiento del pendulo.
 
      // *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
      // De esta zona se sale con un pulso del detector lateral
           if (pulsoControl)
           {
             T_3 = milisegundos;  
             T_zona_2 = min (10000, T_3 - T_2); 
             zona = 3;
           }
      break;
      
    case 3:
    // zona = 3 es la zona entre el sensor de control y el centro. EN ESTA ZONA SE PRODUCE LA 
    //ACTIVACION DEL ELECTROIMAN
 
      // *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
        // AL ENTRAR EN ZONA 3 SE PRODUCE LA ACTIVACION DEL ELECTROIMAN
           if (zonaAnterior != zona)
           {
             // establece el maximo tiempo encendido del electroiman (i segundo)
               T_Seguridad = T_3 + 1000;
             // establece provisionalmente el momento de apagado del electroiman
               T_OFF       = T_Seguridad;

             // solo enciende electroiman si la amplitud es menor de la establecida
               if (amplitud < amplitud_max)
                 {
                   // Conecta el electroiman y LEDElectroMag como control
                   digitalWrite (ElectMag, HIGH);
                   digitalWrite (LEDElectMag, HIGH);
                 }
            }

     // *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
       // De esta zona se sale con un pulso del detector central
           if (pulsoCentro)
           {
             T_0 = milisegundos;
             T_zona_3 = min (10000, T_0 - T_3); 
             zona = 0;
           }
             
       break;
    default: 
      zona = 0;  // si por algun motivo la variable se sale de margenes vuelve a la zona 0
  }
  
  zonaAnterior = zona;
  
  
       // Si se supera el tiempo de encendido del electroiman o el de seguridad APAGA EL ELECTROIMAN
    if (milisegundos > min (T_Seguridad, T_OFF)) digitalWrite (ElectMag, LOW); 
    if (milisegundos > (T_ON + 15))              digitalWrite (LEDElectMag, LOW);
 
 
}


/***************************************************************************
 Function Name: Escribe7SEG

 Purpose: 
   Escribe el numeroen el display de 7 segmentos.
****************************************************************************/

int Escribe7SEG (float numero)
{
  int longitud = 0;
  longitud = Send7SEG (4, NumberLookup [int (numero)        % 10 + 10]);  // Ponemos el . decimal
  longitud += Send7SEG (3, NumberLookup [int (numero * 10)   % 10]);
  longitud += Send7SEG (2, NumberLookup [int (numero * 100)  % 10]);
  longitud += Send7SEG (1, NumberLookup [int (numero * 1000) % 10]);
  return longitud;
}


/***************************************************************************
 Function Name: Send7SEG

 Purpose: 
   Send I2C commands to drive 7-segment display.
****************************************************************************/

int Send7SEG (byte Digit, byte Number)
{
  int longitud;
  
  Wire.beginTransmission(_7SEG);
  longitud = Wire.write(Digit);
  longitud += Wire.write(Number);
  if (Wire.endTransmission() != 0) longitud = -1;
  return longitud;
}  



/***************************************************************************
 Function Name: doEncoder

 Purpose: 
   Recoge los datos del canal B del encoder y del reset.
   Est´ definida para ser llamada durnate una interrupci´n por el canal A
****************************************************************************/

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  
  // La interrupcion se lanza en los flancos de subida del pinA
  // El valor en pinB determina la direccion de giro
  // Si el pin de reset esta a nivel bajo pone a 0 el valor del contador
  
  //if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
  
  if (!digitalRead (resetPin))
  {
    encoderPos = 0;
  }
  else
  {
    if (digitalRead(encoderPinB)) {
      encoderPos++;
    } 
    else {
      encoderPos--;
    }
  }
}



