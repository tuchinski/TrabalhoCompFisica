#include <MPU6050.h>//Libreria acelerometro


// -------------------------------- PULSE SENSOR ---------------
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
const int OUTPUT_TYPE = SERIAL_PLOTTER;

const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13;    // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

int media_anterior = 0;
//int frequencia_anterior = 0;
long tempo_bpm = 0;
long tempo_respiracao = 0;

PulseSensorPlayground pulseSensor;

//********************************************************//

// -------------------------------- MPU 6050 ---------------
MPU6050 mpu;        // declaração do sensor
int16_t ax, ay, az, gx, gy, gz; // variáveis do sensor

const int numReadings = 40; // número de amostra para média
int readings[numReadings]; // Vetor que guarda as leituras
int index = 0; // Indice atual da leitura
int total = 0; // Total
float averageX = 0; // Média

///// Frequência Respiratória ///
//long range = 5; // este es el rongo por el que se disparará la salida 2 y pasa a estado lógico 1
long ultimaMedicao; // Valor da ultima medição
int ciclo = 0; // 1=alto 0=baixo
int picoTensao;
int valeTensao = 1023;
///////********************/////////
boolean estadoBPM = true;  // guarda o estado do LED(aceso ou apagado)
boolean estadoLed = true;  // guarda o estado do LED(aceso ou apagado)
int intervaloAceso = 100; // tempo que o led fica aceso
int IntervaloApagado = 100;  // tempo que o led fica apagado

int IntervaloBPM = 15000;   // tempo led está apagado
int IntervaloBPM2 = 1;    // tempo led está apagado

unsigned long tempoAnteriorAceso = 0; //guarda o tempo de referência para comparação
unsigned long tempoAnteriorApagado = 0; //guarda o tempo de referência para comparação
unsigned long tempoAnteriorBPM = 0; //guarda o tempo de referência para comparação
unsigned long tempoAnteriorBPM2 = 0; //guarda o tempo de referência para comparação
///******************************************************+
int pulsos = 0;
int pulsos2 = 0;
int sinal = 0;
//******************************************************************************************//


void setup()
{
  Serial.begin(9600);
  // ----------- MPU6050 ---------------------
  // Iniciar todas as leituras com o valor 0
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
  pinMode(13, OUTPUT);
  mpu.initialize();   //Iniciamos acelerometro
  if (!mpu.testConnection())
  {
    while (1);
  }

  // ----------PULSE SENSOR -------------
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  //CASO O PULSE SENSOR NAO INICIE, O LED 13 VAI PISCAR
  if (!pulseSensor.begin()) {
    for (;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }
}

void loop()
{

  if (pulseSensor.sawStartOfBeat()) {
    int ms = pulseSensor.getInterBeatIntervalMs();
    int batida = pulseSensor.getBeatsPerMinute();
    pulseSensor.outputBeat();
    if ((millis() - tempo_bpm) > 1000) {
    Serial.print("PULSE SENSOR!!!!!\nMS: ");
    Serial.println(ms);

    Serial.print("Beat: ");
    Serial.println(batida);
    }
  }



  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// pegando os dados do sensor;
  gx = (gx / 10); // parametriza os dados

  //***************  FILTRO EIXO "GX"  *********************//
  // Subtraimos a ultima leitura:
  total = total - readings[index];
  // Leitura do sensor
  readings[index] = gx;
  // Adicionamos o valor no total
  total = total + readings[index];
  // Avança o index
  index = index + 1;

  if (index >= numReadings)
    index = 0;

  // media
  averageX = total / numReadings;

  averageX = averageX + averageX;

  if (averageX <= 150) {
    averageX = 140;
    digitalWrite(13, LOW);
  }//140
 
  if ((millis() - tempo_respiracao) > 1000) {
    Serial.print("Sinal: ");
    Serial.println(averageX);
    ////// BPM //////
    Serial.print(" FREQUENCIA RESPIRATORIA: ");
    Serial.println(pulsos2 * 4);
    tempo_respiracao = millis();
  }
//  media_anterior = averageX;


  //************************* CALCULO FRECUENCIA RESPIRATORIA  **********************************************//
  sinal = averageX; //Guardando o valor na variavel sinal
  /////////***********************************************************//////////////////
  if (sinal >= ( ultimaMedicao + 20) )
  {
    ultimaMedicao = sinal;
    ciclo = 1;
    if (sinal > picoTensao)
    {
      picoTensao = sinal;
    }
  }

  if (sinal <= ( ultimaMedicao - 20))
  {
    ultimaMedicao = sinal;
    ciclo = 0;
    if (sinal < valeTensao)
    {
      valeTensao = sinal;
    }
  }

  //  //********************************* 1 minuto ****************************************//
  if (millis() - tempoAnteriorBPM >= IntervaloBPM) {
    estadoBPM = false;
    pulsos2 = pulsos;
    tempoAnteriorBPM = millis();
    pulsos = 0;
  }
  if (millis() - tempoAnteriorBPM2 >= IntervaloBPM2) {

    estadoBPM = true;

    tempoAnteriorBPM2 = millis();
  }
  ///////////////////////////////////////
  if ((millis() - tempoAnteriorAceso >= intervaloAceso) && estadoLed == true && ciclo == 0)
  {

    estadoLed = false;
    picoTensao = sinal;
    valeTensao = sinal;
    digitalWrite(13, LOW); //apago el led
    //tone(buzzer, 2500);
    tempoAnteriorApagado = millis();
  }

  if ((millis() - tempoAnteriorApagado >= IntervaloApagado) && estadoLed == false && ciclo == 1)
  {
    pulsos++;
    picoTensao = sinal;
    valeTensao = sinal;
    estadoLed = true;
    digitalWrite(13, HIGH);
    tempoAnteriorAceso = millis();
  }
}
