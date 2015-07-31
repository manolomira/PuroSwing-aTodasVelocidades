/*
SKETCH PARA MODULO A TODAS VELOCIDADES DE LA EXPOSICION PURO SWING
Adaptado para su uso con Shield Pendulo (version inicial)



Disponible en GitHub: https://github.com/manolomira/PuroSwing-aTodasVelocidades

Basado en sistema de pendulo perpetuo por bombeo actualizado en 2015

Sensores de barrera optica para el pendulo a traves de placa amplificadora
*/
// ATENCION: LEVANTAR LA PATA 3 DEL CONECTOR SENS PEND //
// ATENCION: LEVANTAR LA PATA 3 DEL CONECTOR SENS PEND //

/* CONECTOR BARRERAS OPTICAS PENDULO SUB-D 9 MACHO EN CHASIS
          1 -> Electroiman -                   (naranja)
          2 -> Electroiman +                   (amarillo)
          3 -> Comun barreras (Gnd)            (violeta + negro)
          4 -> barrera centro: retorno         (verde)
          5 -> Barrera centro: alimentacion    (azul)
          6 -> Alimentacion LED + 24V          (rojo)
          7 -> Alimentacion LED Gnd            (marron)
          8 -> Barrera lateral: alimentacion   (gris)
          9 -> Barrera lateral: retorno        (blanco)          */
          
/* CONECTOR ENCODER Y DISPLAY SUB-D 9 PINES HEMBRA EN CHASIS
          1 -> Encoder canal A                 (Amarillo)
          2 -> Alimentacion encoders + 5V      (Rojo)
          3 -> libre
          4 -> Alimentacion display + 9V       (Violeta)
          5 -> I2C  - SDA                      (verde)
          6 -> Encoder canal B                 (Naranja)
          7 -> Alimentacion comun Gnd          (Marron)
          8 -> libre
          9 -> I2C  - SCL                      (Azul)            */

/* CONECTOR BARRERA PASO CURSOR POR CERO DIN 5PINES 180º
          1 -> libre
          2 -> Comun Barreras Gnd              (Azul)
          3 -> Alimentacion barrera            (Rojo + Gris)
          4 -> libre
          5 -> Retorno barrera                 (Blanco)          */

/* CONECTOR DISPLAY I2C
          1 -> Alimentacion display + 9V       (Violeta)
          2 -> I2C  - SCL                      (Azul)
          3 -> I2C  - SDA                      (verde)
          4 -> Alimentacion display Gnd        (Gris)            */

/* AJUSTE DEL ENCODER
El recorrido del cursor es de 700 mm. El encoder cuenta 300 pulsos en ese recorrido. 
El pulso de reset esta a 10 pulsos de encoder hacia la izquierda.
Reset pone el encoder en 10.
La conversion es  posicion =  encoder * 830 / 300.   */


int minEncoder = -160, maxEncoder = 140;   // indica loslimites del recorrido del encoder
int amplitud_max =  38;                    // amplitud maxima de bombeo del pendulo




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


float amplitud;
int periodoPendulo;
unsigned long T_Pendulo;
unsigned long T_actualiza;
boolean penduloParado;


volatile  int encoderPos = 0;
int encoderPosAnt;
float velocidad;
float posicion;


// ************* Variables display 7 Segmentos ***********
#define _7SEG (0x38)   /* I2C address for 7-Segment */
  // valores de los 9 digitos sin y con punto decimal
const byte NumberLookup[34] =   {
  0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,  // digitos de 0 a 9
  0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,  // digitos con punto
  0x77,0x7C,0x39,0x5E,0x79,0x71,                      // A,B,C,D,E,F
  0x00,0x40,0x63,0x5C,                  // espacio, guion, cuadro alto, cuadro bajo
  0x80,0xC0,0xE3,0xDC};                 // idem con punto

/***************************************************************************
 Function Name: setup

 Purpose: 
   Initialize hardwares.
****************************************************************************/

void setup()
{

  Serial.begin(57600); 

  Serial.println ("Iniciando conexion con I2C y esbleciendo parametros del display");
    
    
    amplitud = amplitud_max;


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

  // Escribe algo que no es un numero en el display
  Send7SEG (4, NumberLookup [28]);
  Send7SEG (3, NumberLookup [29]);
  Send7SEG (2, NumberLookup [28]);
  Send7SEG (1, NumberLookup [29]);

  Serial.println("Set Up Completo");  
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
    
    
  if (encoderPos != encoderPosAnt)
  {
    // Actualiza el display
    ActualizaDisplay();
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
  static int extraT_ON;
  static int erroresSensor;
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


 
// Gestiona el funcionamiento del pendulo en cada zona 
// Solo se realizan acciones al entrar en cada zona

  if (zonaAnterior != zona)
  {
    switch (zona) {
      case 0:
      //zona = 0 es la zona opuesta al sensor de control. 
           // Al entrar en esta zona se establece el momento de desconexion del electroiman
           // ExtraT_ON es fijo y determinado por la posicion de pot1
           extraT_ON = map (analogRead (pot1), 0, 1023, 100, 0);

           // establece el valor  para T_OFF
           T_OFF     = T_0 + extraT_ON;
        break;

      case 1:
      // zona = 1 desde el paso por el centro hasta la llegada al sensor de control. 
           // En esta zona no se esperan señales de las barreras opticas por eso
           // es el momento para hacer las tareas que ocupan mas tiempo

           periodoPendulo = T_zona_0 + T_zona_1 + T_zona_2 + T_zona_3;
           T_Pendulo = T_0;            // corresponde al momento del paso por el centro en direccion 
                                       // contraria a la zona del sensor de control
             
           float amplitudCalculada;
           amplitudCalculada = 0.0002 * T_zona_2 * T_zona_2 - 0.1472 * T_zona_2 + 49.125;

           // Nueva interpolacion lineal para T_2 entre 350 y 700 ms
           // amplitudCalculada = 0.0818 * T_zona_2 + 15.692;

             
           // Si falla alguno de los sensores y elperiodo registrado es excesivo no se tiene 
           //    en cuenta para el calculo de amplitud. Se incrementa el contador de errores
           erroresSensor ++;
           if (periodoPendulo < 2000) 
           {
             erroresSensor = 0;
             // Si la diferencia con el valor medio no es muy grande calcula un nuevo valor 
             // progresivo. En caso contrario toma como valido el nuevo valor.
             if (abs (amplitud - amplitudCalculada) > 2.0) amplitud = amplitudCalculada * 0.1;
             else amplitud = amplitud * 0.9 + amplitudCalculada * 0.1;
           }

            
           Serial.println("--------------------------------");

           Serial.print ("|  PerPend = "); Serial.print (T_zona_0 + T_zona_1 + T_zona_3);
           Serial.print (" + "); Serial.print (T_zona_2);
           Serial.print (" = "); Serial.print (periodoPendulo);

           Serial.print ("\tTPendulo = "); Serial.print (T_Pendulo);
             
           Serial.print ("\tT_iman = "); Serial.print (T_zona_3);
           Serial.print (" + "); Serial.print (extraT_ON);
             
           if (periodoPendulo < 2000)
           {
             Serial.print ("\t >> amplitud = "); Serial.print (amplitudCalculada ,2);
             Serial.print (" -> "); Serial.print (amplitud);
             Serial.print (" ("); Serial.print (amplitud_max);
             Serial.print (")");
           }
           else
           {
             Serial.print ("\t >> Error sensores ("); Serial.print (erroresSensor);
             Serial.print (" err)");
           }

           Serial.println ();             
      
           // Actualiza el display
           ActualizaDisplay ();
        break;

      case 3:
      // zona = 3 es la zona entre el sensor de control y el centro. EN ESTA ZONA SE PRODUCE LA 
      //ACTIVACION DEL ELECTROIMAN
            // AL ENTRAR EN ZONA 3 SE PRODUCE LA ACTIVACION DEL ELECTROIMAN

            // establece el maximo tiempo encendido del electroiman (i segundo)
            T_Seguridad = T_3 + 1000;

            // establece provisionalmente el momento de apagado del electroiman
            T_OFF       = T_Seguridad;

            // solo enciende electroiman si la amplitud es menor de la establecida
            if ((amplitud < amplitud_max) && ((T_zona_0 + T_zona_1 + T_zona_2 + T_zona_3) < 2000))
            {
              // Conecta el electroiman y LEDElectroMag como control
              digitalWrite (ElectMag, HIGH);
              digitalWrite (LEDElectMag, HIGH);
            }
        break;
      
      default:
      // Registra el momento de la ultima actualizacion para comprobar que el pendulo sigue funcionando
      T_actualiza = milisegundos;
    }
  }


// GESTIONA EL CAMBIO DE ZONAS
//   *** Ejecuta las transiciones entre las zonas en funcion de los detectores opticos
//       En cada cambio de zona registra la marca de tiempo (millis) de salida
//       y el tiempo que permanecio en esa zona. El tiempo de permanencia queda limitado a 
//       10000 ms para evitar desbordar la variable tipo int.

  zonaAnterior = zona;

  switch (zona) {
    case 0:
      // zona = 0 es la zona opuesta al sensor de control.
      // De esta zona se sale con un pulso del detector central
           if (pulsoCentro)
           {
             T_1 = milisegundos;  
             T_zona_0 = min (10000, T_1 - T_0); 
             zona = 1;
           }
      break;
 
    case 1:
      // zona = 1 desde el paso por el centro hasta la llegada al sensor de control. 
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
    // ACTIVACION DEL ELECTROIMAN
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
  
  // Si se supera el tiempo de encendido del electroiman o el de seguridad APAGA EL ELECTROIMAN
  if (milisegundos > min (T_Seguridad, T_OFF)) digitalWrite (ElectMag, LOW); 
  if (milisegundos > (T_ON + 15))              digitalWrite (LEDElectMag, LOW);
  
  
  // Comprueba si el pendulo esta parado. Si no se recibe señal de las barreras en 10 segundos 
  // considera que el pendulo esta parado.
  penduloParado = false;
  if (milisegundos > T_actualiza + 10000)
  {
    penduloParado = true;
    ActualizaDisplay ();
  }
  
 
}


/***************************************************************************
 Function Name: Escribe7SEG

 Purpose: 
   Escribe el numeroen el display de 7 segmentos.
****************************************************************************/

int Escribe7SEG (float numero)
{
  int longitud = 0;
  
  if (numero >= 0)
  {
    longitud =  Send7SEG (4, NumberLookup [int (numero)        % 10 + 10]);  // Ponemos el . decimal
    longitud += Send7SEG (3, NumberLookup [int (numero * 10)   % 10]);
    longitud += Send7SEG (2, NumberLookup [int (numero * 100)  % 10]);
    longitud += Send7SEG (1, NumberLookup [int (numero * 1000) % 10]);
  }
  else 
  {
    for (int i = 1; i < 5; i++)
    {
      longitud +=  Send7SEG (i, NumberLookup [27]);
    }
  }

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
  
  if (digitalRead (resetPin))
  {
    encoderPos = - 15;
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









void ActualizaDisplay ()
{
  // Actualiza el valor de la velocidad si se movio el cursor
  // Comprueba que no se haya salido de limites
    encoderPos = max(encoderPos, minEncoder);
    encoderPos = min(encoderPos, maxEncoder);
    
    if (!penduloParado)
    {
      // Calcula la posicion del cursor asumiendo un recorrido total de 100 cm y 10 vueltas (320 pulsos)
      posicion  = 100.0 * float(encoderPos) / 300.0;
    
      if ((amplitud > 24) && (amplitud < 46))
      {
        if (abs(posicion) > amplitud + 5)
        {
          velocidad = -1.0;
        }
        else if (abs(posicion) > amplitud)
        {
          velocidad = 0.0;
        }
        else
        {
          float velMaxima = 1.25;
          velocidad = velMaxima * pow((1.0 - pow(posicion/amplitud, 2.0)), 0.5);
        }
      }
      else velocidad = -99;
    
      encoderPosAnt = encoderPos;
 
      Serial.print(" encoderPos : ");
      Serial.print(encoderPos, DEC);
 
      Serial.print("\t");
      Serial.print(" posicion cursor : ");
      Serial.print(posicion,3);

      Serial.print("\t");
      Serial.print(" velocidad : ");
      Serial.print(velocidad,3);

      Serial.println();

      Serial.print (" numero de bits enviados a Wire= ");
      Serial.println ( Escribe7SEG (velocidad)); 
    }
    
    else 
    {
      // Indica paro en el display. 
      // El texto aparece al mover el cursor, y desparece al moverlo otra vez, 
      // una vez que el pendulo vuelva a funcionar.
      Send7SEG (4, B01110011);
      Send7SEG (3, B01110111);
      Send7SEG (2, B01010000);
      Send7SEG (1, B01011100);
    }
}
