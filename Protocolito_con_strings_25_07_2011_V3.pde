/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 Programa para la entrega del prototipo 
 Todos los derechos de autor reservados para:
 Ángel Oswaldo García Rosas
 y
 Centro de Investigación Científica y de Estudios Superiores
 de Ensenada CICESE
 Lab. energías alternas CICESE
 Ensenada Baja California México
 25 de noviembre del 2011
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* Preprocesador */

/* Bibliotecas */

#include <SPI.h>
#include <Wire.h>

/* ~~~~~~~~~~ Constantes ~~~~~~~~~~~ */
#define WRITE 128
#define READ  0x00
/* ************ Declarar los pines usados para la comunicacion ******** */
#define DATAOUT     11      //MOSI
#define DATAIN      12      //MISO 
#define SPICLOCK    13      //sck
#define SS          10      //ss
/* -------------------------------------------------------------------- */

/* Variables */

/* Protocolo */
char octeto_recibido;
byte contador;
byte estado;
byte direccion;
byte mando;
byte suma;
byte ack;
unsigned long datos;
unsigned int mi_suma;
byte verificar_suma;
unsigned long timeout_ints, timeout;

/* char por el manejo de "strings" */
char Start_1_byte;
char Start_2_byte;

char Final_1_byte;
char Final_2_byte; 

/*  Concatenar Strings usando el operator += y concat() */

char     myDir;
char     lectura;

String   ModBus_Start;
String   ModBus_Final;

String   pack_Final;
String   pack_Start;

char     Start_1;
char     Start_2; 
 
char     Final_1;
char     Final_2; 

/* Variables de la medicion de energia */
byte dato1, dato2, dato3;
unsigned long energia_activa, energia_aparente;
unsigned long Energia_Activa_acumulada, Energia_Aparente_acumulada;
unsigned long Numero_segundos_activa, Numero_segundos_aparente;

/* Variables base de tiempo */
unsigned int milis, retardo;
unsigned int interrupciones_TMR1;
unsigned int segundos;

unsigned long datos_tx_energia_activa;
unsigned long datos_tx_energia_aparente;
int segundos_tx_energia_activa;
int segundos_tx_energia_aparente;

/* Variables de temperatura */
float temp_ambiente;

// TMP102 registros apuntadores
int ptrTemp = 0x0; // Temperatura	   
int ptrConf =  0x1; // Configuracion
int ptrTLow =  0x2; // Temperatura mas baja
int ptrTHigh = 0x3; // Temperatura mas alta
int i2cAddr;	           // Direccionnamiento interno en el chip
// Temperaturas flotantes 
float tempflt, lowtempflt, hitempflt;

int Temperatura_Ambiente_Tx;
int Temperatura_Ambiente;
/* ------------------------------------------------------------------- */
/* Configuración */
void setup() 
{
  myDir = 61; /* Direccion de este modulo */
  
  /* Retardo de la base de tiempos */
  retardo = 15625;
  
  /* Cadena identificadora de inicio y final de mensaje */
  ModBus_Start = String(':') + String(myDir);
  ModBus_Final = String('\r');
  ModBus_Final += String('\n');

  Configura_SPI();  
  Inicializa_485();
  Serial.begin(9600); /* Configura UART a 9600bps, 8N1 */

  Configura_Contador_16_bits();
  sei();
  Wire.begin();
  /* temperatura */
  i2cAddr = 0x48; // Direccionar el chip como conectado a GND
}

/* -------------------------------------------------------------- */
/* lazo principal */
void loop() 
{


/* --------------------------------------------------------------------------- */
/* Tiempo de acumulación de energía en el ADE */
  
       if(segundos==10)
          { 
//                Serial.println(Lee_energia_activa());
              Energia_Activa_acumulada += Lee_energia_activa();
              Numero_segundos_activa += segundos;
              Energia_Aparente_acumulada += Lee_energia_aparente();
              Numero_segundos_aparente += segundos;
              segundos = 0;
        }
       

 /* ---------------------------------------------------------------------------- */        
    /* PROTOCOLO */
   if(estado==0){  
    
       if(Serial.available()>0){
              Start_1 = Serial.read();
              pack_Start = String(Start_1);
              if(pack_Start == ':')
                {
                  contador++;
                  estado = 1;
                }
                
              else
                { 
//                  contador = 0;
                  pack_Start = 0;
                  Start_1 = 0;
                  Serial.flush();
              } // else
       } // if serial available
   } // if estado == 0  
       
 /* ---------------------------------------------------------------------------- */         
    else if(estado==1){
       if(Serial.available()>0){
              Start_2 = Serial.read();  
              pack_Start += String(Start_2); 
                 if(pack_Start == ModBus_Start)
                   {
                   contador++;
                   direccion = Start_2;
                   estado = 2;
                   }
                       
                 else 
                  {
                   contador = 0;
                   Start_1 = 0;
                   Start_2 = 0;
                   pack_Start = 0;
                   estado = 0;
                 }  //else
           }// Serial available
    } //estado == 1
 /* ---------------------------------------------------------------------------- */                  
    else if(estado == 2){
      
        if(Serial.available()>0){
              
            contador++;
       
                switch(contador){
       
                  case 3:
                      mando = Serial.read(); // ascii to bin
                     break;

                  case 4: 
                     lectura = Serial.read();
                     datos |= (lectura<<16);
                    break;
      
                  case 5:
                     lectura = Serial.read();
                     datos |= (lectura<<8);
                    break;
      
                  case 6:
                     lectura = Serial.read();
                     datos |= lectura;
                    break;

                  case 7:
                     suma=Serial.read();
                    break;       
           
                  case 8:
                       Final_1 = Serial.read();
                       pack_Final = String(Final_1);
                     break;         
           
                  case 9:
                       Final_2 = Serial.read();
                       pack_Final += String(Final_2);
             
                        if(pack_Final == ModBus_Final)
                          {
                            Procesa_comando();
                            Reinicia_Variables_Protocolo();
                            estado = 0;  
                         }             
                  
                        /* time out */
                        else
                          {
                            //delay(500);    /*  El time out está dando problemas */
                            Procesa_comando();
                            Serial.flush();
                            Reinicia_Variables_Protocolo();
                            estado = 0;
                        }             
                  
                     break;
           
           
             } //switch
         } // if serial available 
      } // if estado == 2     
 } // loop
/* ********** Rutina de servicio de interrupcion al vector de comparacion timer 1 ************** */
ISR(TIMER1_COMPA_vect)
{
  segundos++; 
};
/* ++++++++++++++++++++ FUNCIONES +++++++++++++++++++++++ */
/* Inicializa o configura 485 */
void Inicializa_485(){
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT); 
}
/* --------------------------------------------------- */
/* Habilita el envio por RS-485 */
void Tx_485(){
  digitalWrite(2, HIGH); 
  digitalWrite(3, HIGH);
  delay(6);
}
/* -------------------------------------------------- */
/* Habilita el envio por RS-485 */
void Rx_485(){
  digitalWrite(2, LOW); 
  digitalWrite(3, LOW);
  delay(6);
}
/* -------------------------------------------------- */

/* *************************************   Contador de 16 bits *********************************** */
void Configura_Contador_16_bits()
{
  //Modo puesta a uno en los comparadores
   TCCR1A = 0x00;    
   TCCR1B = 0xD;     // reloj del contador 16mhz/1024
   OCR1A  =  retardo;   // valor de comparacion para tardar un segundo
   TIMSK1 = 0x2;     // Interrupcion generada por el comparador
}
/* ----------------------------------------------------- */
unsigned long Lee_energia_activa(){
  digitalWrite( SS, HIGH);
  digitalWrite( SS, LOW);
  SPI.transfer(0x03); /* Leer con reinicio el RAENERGY energia activa */  
  delayMicroseconds(4); /* retardo segun la hoja de datos */
  energia_activa = SPI.transfer(0xFF)<<16; // << 16 Bit mas significativo primero, mandar unos para generar reloj para la lectura 
  delayMicroseconds(1); /* Esperar un retardo segun la hoja de datos para enviar el segundo dato */
  energia_activa |= SPI.transfer(0xFF)<<8;
  delayMicroseconds(1); /* Esperar un retardo segun la hoja de datos para enviar el segundo dato */
  energia_activa |= SPI.transfer(0xFF);
  delayMicroseconds(1);
  digitalWrite( SS, HIGH);
  return energia_activa;
  energia_activa = 0; 
}
/* ------------------------------------------------------- */
unsigned long Lee_energia_aparente(){
  digitalWrite( SS, HIGH);
  digitalWrite( SS, LOW);
  SPI.transfer(0x06); /* Leer con reinicio el RVAENERGY energia aparente*/  
  delayMicroseconds(4); /* retardo segun la hoja de datos */
  energia_aparente = SPI.transfer(0xFF)<<16; // Bit mas significativo primero, mandar unos para generar reloj para la lectura 
  delayMicroseconds(1); /* Esperar un retardo segun la hoja de datos para enviar el segundo dato */
  energia_aparente |= SPI.transfer(0xFF)<<8; /* Segundo byte */  
  delayMicroseconds(1); /* Esperar un retardo segun la hoja de datos para enviar el segundo dato */
  energia_aparente |= SPI.transfer(0xFF); /* tercer byte */  
  delayMicroseconds(1);
  digitalWrite( SS, HIGH);
  return energia_aparente;
  energia_aparente = 0;
}
/* ---------------------------------------------------------- */
void Configura_SPI(){
    SPI.begin(); /* Inicializa SPI */
    SPI.setDataMode(SPI_MODE1); /* MODO SPI = 1: clk pol =0, muestrea en flanco de bajada   */    
    SPI.setBitOrder(MSBFIRST); /* byte mas significativo primero */
    SPI.setClockDivider(SPI_CLOCK_DIV128); /* Reloj del SPI = 16Mhz/64 */
}
/* ---------------------------------------------------------- */
void Configura_ADE7753(){
  digitalWrite( SS, HIGH);
  digitalWrite( SS, LOW);
  SPI.transfer(128|0x09); /* Escribe al mode register */  
  delayMicroseconds(4); /* retardo segun la hoja de datos */
  SPI.transfer(0x80); // Bit mas significativo primer = modo de solo acumulacion positiva de energia 
  delayMicroseconds(1); /* Esperar un retardo segun la hoja de datos para enviar el segundo dato */
  SPI.transfer(0x00); /* Segundo byte */
  digitalWrite( SS, HIGH);  
  return;
};

/* -------------------------------------------------------------------------------------------------- */
int Procesa_Suma_Tx(){

}
/* -------------------------------------------------------------------------------------------------- */
void Reinicia_Variables_Protocolo(){
    estado = 0;
    contador = 0;
    Start_1 = 0;
    Start_2 = 0;
    pack_Start = 0;
    pack_Final = 0; 
}

/* -------------------------------------------------------------------------------------------------- */
void Manda_MSJ_ModBus(unsigned long datos_a_enviar){
      
}
/* -------------------------------------------------------------------------------------------------- */
/* FUNCIONES DE TEMPERATURA I2C */
float Lee_temp_ambiente(){

 /* Leer temperatura ambiente */
  Wire.beginTransmission(i2cAddr);  // Manda la direccion del dispositivo i2c a operar
  Wire.send(ptrTemp);		    // manda la direccion del apuntador del registro que se leera
  Wire.endTransmission();	    // termina la transmision

  Wire.requestFrom(i2cAddr, 2);     // solicita la respuesta del registro apuntador
  byte byte1 = Wire.receive();      // primer byte de temperatura
  byte byte2 = Wire.receive();      // segundo byte de temperatura

  
  /* Calcular temperatura  */
  int tempint = byte1 << 8;	   // recorre a la izquierda el primer byte 8 lugares en un registro de 2 bytes
  tempint = tempint | byte2;	   // agrega el segundo byte
  tempint = tempint >> 4;	   // recorre 4 lugares a la derecha el registro(la temp. es de 12 bits de resolucion
  float tempflt = float(tempint) * .0625; // calcula la temperatura actual en un registro de informacion flotante
  return tempflt;
}
/* -------------------------------------------------- */
/* Procesar comando */
void Procesa_comando(){

  /* Poner un switch comando */
  switch(mando){
/* -------------------------------------------------- */    
    /* Energia activa */
    case 'a':
     
      Energia_Activa_acumulada+=Lee_energia_activa();
      Numero_segundos_activa+=segundos;
      segundos = 0;
      datos_tx_energia_activa = Energia_Activa_acumulada;
      segundos_tx_energia_activa = Numero_segundos_activa;
      
      Tx_485();   
      Serial.print("M");
      Serial.print(myDir, DEC);
      Serial.print("  ");
      Serial.print("a");
      Serial.print("  ");
      Serial.print(datos_tx_energia_activa);
      Serial.print("  ");
      Serial.println(segundos_tx_energia_activa);   
      segundos_tx_energia_activa=0;
      datos_tx_energia_activa=0;
      Energia_Activa_acumulada = 0;    
      Numero_segundos_activa = 0;
      Rx_485();
      break;
      
/* -------------------------------------------------- */      
    /* Energia aparente */  
    case 'b':
        
      Energia_Aparente_acumulada+=Lee_energia_aparente();
      Numero_segundos_aparente+=segundos;
      segundos = 0;
      datos_tx_energia_aparente = Energia_Aparente_acumulada;
      segundos_tx_energia_aparente = Numero_segundos_aparente;
      
      Tx_485();   
      Serial.print("M");
      Serial.print(myDir, DEC);
      Serial.print("  ");
      Serial.print("b");
      Serial.print("  ");
      Serial.print(datos_tx_energia_aparente);
      Serial.print("  ");
      Serial.println(segundos_tx_energia_aparente);   
      segundos_tx_energia_aparente=0;
      datos_tx_energia_aparente=0;
      Energia_Aparente_acumulada = 0;    
      Numero_segundos_aparente = 0;
      Rx_485();
    
      break;
/* -------------------------------------------------- */      
    /* Ambas Energias */  
    case 'c':
        
      Energia_Activa_acumulada+=Lee_energia_activa();
      Numero_segundos_activa+=segundos;
      Energia_Aparente_acumulada+=Lee_energia_aparente();
      Numero_segundos_aparente+=segundos;
      segundos = 0;
      datos_tx_energia_activa = Energia_Activa_acumulada;
      segundos_tx_energia_activa = Numero_segundos_activa;
      datos_tx_energia_aparente = Energia_Aparente_acumulada;
      segundos_tx_energia_aparente = Numero_segundos_aparente;
    
      temp_ambiente = Lee_temp_ambiente();    
    
      Tx_485();   
      Serial.print("M");
      Serial.print(myDir, DEC);
      Serial.print("  ");
      Serial.print("c");
      Serial.print("  ");
      Serial.print(datos_tx_energia_activa);
      Serial.print("  ");
      Serial.print(segundos_tx_energia_activa);   
      Serial.print("  ");
      Serial.print(datos_tx_energia_aparente);
      Serial.print("  ");
      Serial.print(segundos_tx_energia_aparente);   
      Serial.print("  ");
      Serial.println(temp_ambiente, DEC);   
    
      temp_ambiente = 0;    
      segundos_tx_energia_activa = 0;
      datos_tx_energia_activa = 0;
      Energia_Activa_acumulada = 0;    
      Numero_segundos_activa = 0;
    
      segundos_tx_energia_aparente = 0;
      datos_tx_energia_aparente = 0;
      Energia_Aparente_acumulada = 0;    
      Numero_segundos_aparente = 0;
      
      Rx_485();
      
      break;    
      
  }   
}

