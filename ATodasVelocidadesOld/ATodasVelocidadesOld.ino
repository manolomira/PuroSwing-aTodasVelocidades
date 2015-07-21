
// En modulo "A todas velocidades" hay un pendulo perpetuo y un display de 7 segmentos para visualizar la velocidad en cada punto
// Se unen por tanto los programas pendulo_perpetuo_mm_bitA y Display7Seg_Vel_Pend_Sen ya testado.


#include <Bounce.h>
#include <Wire.h> 
#include <Metro.h>

#define _7SEG (0x38)   /* I2C address for 7-Segment */


// *************** VARIABLES DEL PENDULO ***************

// Define entradas y salidas
const int LEDInner = 3;     // Indica que se activo la barrera optica de referencia
const int LEDElectMag = 9;   // Indica que el electroiman esta activado
const int SWInner = 7;      // Entrada de la barrera optica de referencia
const int ElectMag = 5;      // Salida que activa el electriman
const int FinIman = 4;       // Barrera que indica que el pendulo esta cerca de la bobina
const int LEDFinIman = 6;    // Indica que se activa barrera cerca de la bobina

const int pot = A0;          // entrada analogica potenciometro de ajuste
boolean bitAmplitud;         // =1 si supera controlAmplitud
boolean bithigh;             // =1 si toca pulso

const long intervaloSeguridad = 1000;        // tiempo maximo que se activa el electroiman por seguridad
const int controlAmplitud = 25;


Bounce SWInnerBounce = Bounce( SWInner,50 ); 
Bounce FinImanBounce = Bounce( FinIman,50 ); 


long milliseconds;
long tiempoActual;
long tiempoAnterior;
long tiempoOn;
long tiempoOff;
  
int periodoPendulo;
int intervaloActual;
int intervaloAnterior;
int extraTon;                          // tiempo extra calculado a partir del potenciometro


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
    // *************** DEL PENDULO ***************
    pinMode(SWInner, INPUT);        // optoacoplador de llegada a barrera
    pinMode(FinIman,INPUT);         // optoacoplador de llegada al electroiman

    pinMode(LEDInner, OUTPUT);      // indica la activacion de barrera referencia
    pinMode(LEDElectMag, OUTPUT);   // indica la activacion del electroiman
    pinMode(LEDFinIman, OUTPUT);      // indica la activacion de barrera cerca bobina
    pinMode(ElectMag,OUTPUT);       // Salida electroiman

    milliseconds = millis();
  
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

void loop()
{  
  
  // *************** GESTION DEL PENDULO ***************
  
  SWInnerBounce.update ();       // Actualiza las instancias Bounce del pendulo y detecta flancos de bajada.
  FinImanBounce.update (); 

  ActualizaPendulo();             // Actualiza las variables asociadas al pendulo y enciende sus actuadores
  
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
 
 //  ********** Funcion ActualizaPendulo ********** 
void ActualizaPendulo()
{

  int tiempoElectroImnan;
  
  boolean pulsoControl = SWInnerBounce.fallingEdge();      // Detecta flancos de bajada en los detectores opticos.
  boolean pulsoFinIman = FinImanBounce.fallingEdge();
  
  milliseconds = millis();                                 // Actualiza el contador de tiempo.
                                                           // Actualiza el ajuste de tiempo extra de encendido de electroiman.
  extraTon = map(analogRead(A0), 0, 1023, -10, 50);  if (extraTon < 0) extraTon = 0;


  if (pulsoControl == true)                                // Cuando detecta el pulso de control actualiza los tiempos.
  {
    tiempoAnterior = tiempoActual;
    tiempoActual   = milliseconds;
    
    intervaloAnterior = intervaloActual;
    intervaloActual = tiempoActual - tiempoAnterior;

    periodoPendulo = intervaloAnterior + intervaloActual;
    
     
    digitalWrite(LEDInner, HIGH);                          // Enciende LEDinner como referencia cuando se activa el sensor 

                                                           // intervaloActual > intervaloAnterior es la condicion 
                                                           //      cuando el pendulo esta de vuelta por lo que activa
                                                           //      el electroiman.   
    if ((intervaloAnterior > 0) && (intervaloActual < intervaloAnterior) )   
    {
        bithigh = HIGH;  //Toca pulso, se da si la amplitud no supera el maximo permitido (controlAmplitud)     
        if (bitAmplitud == LOW)
        {
            digitalWrite (ElectMag, HIGH);                       // Conecta el electroiman y LEDElectroMag como control
            digitalWrite (LEDElectMag, HIGH);
        } 
        tiempoOn = milliseconds;                             // Registra el momento de conexion.   
        tiempoOff = milliseconds + intervaloSeguridad;       // Inicialmente fija tiempoOff al limite del intervalo de seguridad


    } 
    

  }
 
                                                            // Apaga LEDInner 15 ms despues de activarlo.
   if (milliseconds > (tiempoActual + 15))   digitalWrite(LEDInner, LOW);        


   if ((pulsoFinIman == true))                               // Registra el paso por el detector de apagado del Electroiman. 
   {
       digitalWrite(LEDFinIman, HIGH);
       tiempoOff = milliseconds;                               // Establece el valor real para tiempoOff. 
   }

                                                            // Apaga el Electroiman pasado pasado el tiempo de ajuste fino 
                                                                     // tras la activacion del detector de fin o cuando 
                                                                     // haya pasado el intervalo de Seguridad
    tiempoElectroImnan = tiempoOff - tiempoOn;
     
    extraTon=int(315-(intervaloActual*0.4057));  //t2=intervaloActual . Valores calculados de la tabla excel 
   
   
   if (((digitalRead(ElectMag) == HIGH) || bithigh == HIGH) && ((milliseconds > (tiempoOn + intervaloSeguridad)) || (milliseconds > (tiempoOff + extraTon))))
   {
      bithigh = LOW; 
      digitalWrite (ElectMag, LOW);                         // Desconecta el electroiman
      digitalWrite (LEDElectMag, LOW);
      
      //parabola:107,69-t2*0,3789+t2*t2*0,003795    Valor calculado de tabla excel 
      int amplitud = 107.69 - (float(intervaloActual)*0.3789) + pow(float( intervaloActual),2) * 0.0003795;  
      //if (amplitud > controlAmplitud) bitAmplitud = HIGH;  //Control de si nos pasamos de amplitud
      bitAmplitud = LOW;
      // else bitAmplitud = LOW;
      
            
      // Despues imprime los datos del ciclo completado. 

      Serial.print (tiempoElectroImnan);
      Serial.print ("   Periodo = ");
      Serial.print (intervaloAnterior);
      Serial.print (" + ");
      Serial.print (intervaloActual);
      Serial.print ("  = ");
      Serial.print (periodoPendulo);
      Serial.print ("   ttot = ");
      Serial.print (tiempoOff - tiempoOn);
      Serial.print (" + ");
      Serial.print (extraTon);
      Serial.print ("  >> amplitud = ");
      Serial.print (amplitud);
      Serial.print (" A0: ");
      Serial.print (analogRead(A0));
      Serial.println();
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



