// Fonte do código
// https://easytromlabs.com/arduino/arduino-lab-20-leitura-dos-dados-de-um-multimedidor-trifasico-de-energia-via-rs-485-modbus-rtu/

// União entre o código millis e o Modbus.

/// Exemplo de Uma Aplicação Millis

/*////////////////////////////////////////////////////////////////////////////////////
 
 unsigned long Tempo = millis();
unsigned long TempoSubtraido;
unsigned long TempoDeEspera = 10000;// 60000;
void setup() {
  Serial.begin(9600);
}
void loop() {

  TempoSubtraido = millis() - Tempo;
  if(TempoSubtraido > TempoDeEspera)
  {
     Serial.print("Time: ");
     Serial.println(Tempo); // prints time since program started
     Tempo = millis();
  }
}
/////////////////////////////////////////////////////////////////////////////////////*/


///////////////// Bibliotecas ///////////////////////
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <SPI.h> 
#include <Ethernet.h>
///////////////////////////////////////////////////

////////////////////////// Variáveis Globais de rede ////////////////////////////////////////
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(10, 244, 1, 4);
IPAddress server(142, 250, 79, 196);  // google

String barCode = "123456";

char msg[50];
int x = 0;
char c;
int readState = 0;

EthernetClient cliente;
///////////////////////////////////////////////////////////////////////////////////////////////

//Data Enable (DE) and Receiver Enable (/RE)
#define MAX485_DE      23     //pino digital
#define MAX485_RE_NEG  22     //pino digital

// Instancia Objeto ModbusMaster
ModbusMaster node;

uint8_t resultMain;

int endereco_Modulos[] {
  20, // alocando endereço do multimedidor ( ele está configurando a entrada SDA do arduino ?)
};

int vetor_endereco[] {                          //registradores HEX do modbus (read input register)
  0x02, // 0 - Tensão Trifásica [V] - UO
  0x04, // 1 - Corrente Trifásica [A] - IO
  0x06, // 2 - Fator de potencia Trifásico - FP
  0x08, // 3 - Potência Aparente Trifásica [VA] - SO
  0x0A, // 4 - Potência Reativa Trifásica [VAr] - QO
  0x0C, // 5 - Potência Ativa Trifásica [W] - PO
  0x0E, // 6 - Frequencia [Hz] - F
  0x10, // 7 - Tensão Linha-Neutro 1 (V)
  0x12, // 8 - Tensão Linha-Neutro 2 (V)
  0x14, // 9 - Tensão Linha-Neutro 3 (V)
  0x16, // 10 - Corrente Linha 1(A)
  0x18, // 11 - Corrente Linha 2(A)
  0x1A, // 12 - Corrente Linha 3(A)
  0x1C, // 13 - Potência Ativa Linha 1(W)
  0x1E, // 14 - Potência Ativa Linha 2(W)
  0x20, // 15 - Potência Ativa Linha 3(W)
  0x28, // 16 - Potência Aparente Linha 1(W)
  0x2A, // 17 - Potência Aparente Linha 2(W)
  0x2C, // 18 - Potência Aparente Linha 3(W
  0x28, // 19 - Fator de potência Linha 1(W)
  0x2A, // 20 - Fator de potência Linha 2(W)
  0x2C, // 21 - Fator de potência Linha 3(W)
  0x62  // 22 - Contador Parcial de Energia - EAP (kWh)
};

float vetor_resultado[23];    //tamanho do vetor -> 23 posições devido a 23 leituras (mas en exadecimal está pulando algumas posições)

//Union para obter o valor de leitura:
union leitura {           //leitura para transformar o valor em float das grandezas elétricas
  unsigned char vetor[4];
  float valorFloat;
} rs485;    


// Habilita as Variáveis de Tempo
 unsigned long Tempo = millis();
unsigned long TempoSubtraido;
unsigned long TempoDeEspera = 1000;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************
  Função LOOP
*************************************************************/
void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


void setup() {
  // Atribui pinos como saída
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // inicializa modo de recebimento
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Atribui velocidade de comunicação (Baud Rate)
  Serial.begin(9600); //defini a leitura serial
  Serial2.begin(9600, SERIAL_8N2);  //comunicar com o multimedidor

  // Callbacks - Permite configurar o transeiver RS485 corretamente
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  ////////////////////////// Setup de Rede ///////////////////////

   Ethernet.begin(mac, ip);
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************
  Função LOOP
*************************************************************/
void loop() {

  Internet();
  
  TempoSubtraido = millis() - Tempo;
  if(TempoSubtraido > TempoDeEspera)
  {
     Serial.print("Time: ");
     Serial.println(Tempo); // prints time since program started
     Tempo = millis();
     leitura_medidor(endereco_Modulos[0]);
    Serial.println("--> Aguarda 30 segundos para proxima leitura ");
    TempoDeEspera = 90000;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************
  Fazendo conexão com a rede
*************************************************************/
void Internet()
{
//  EthernetClient client = server.available();            //CRIA UMA CONEXÃO COM O CLIENTE
   if(cliente.connect(server, 80)) {
   // Serial.println("Deu certo a conexão") ;
  }
 
  if(cliente.available() > 0) {
     c = cliente.read();
   
      if(c == '*') {       
        readState = 1;
      } else if(readState == 1) {       
        if(c == ',') {
          readState = 0;
          Serial.print(msg);
          x = 0;
        };       
        msg[x] = c;
        x++;
      };
  }

  if (!cliente.connected()) {
    Serial.println();
    Serial.println("disconnecting...");
    cliente.stop();
    delay(50);
    Serial.println("disconnected!");
  }
 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************
  Realiza a leitura dos dados via RS485 do Multimedidor
*************************************************************/

float leitura_medidor(uint8_t deviceID) {

  node.begin(deviceID, Serial2); //ver como transformar uma unica porta em duas -> mudar Serial2.
  Serial.println();
  Serial.print("lendo dispositivo no endereço: ");
  Serial.println(deviceID);

  for (int i = 0; i < 23; i ++) {
    resultMain = node.readInputRegisters(vetor_endereco[i], 2); // Realiza a leitura dos registradores
   // delay(50);

    if (resultMain == node.ku8MBSuccess) {
      rs485.vetor[0] = (node.getResponseBuffer(0x00) >> 8) & 0xFF;  //Parte Alta Word 1
      rs485.vetor[1] = node.getResponseBuffer(0x00) & 0xFF;         //Parte Baixa Word 1
      rs485.vetor[2] = (node.getResponseBuffer(0x01) >> 8) & 0xFF;  //Parte Alta Word 2
      rs485.vetor[3] = node.getResponseBuffer(0x01) & 0xFF;        //Parte Baixa Word 2
    }
    else {
      Serial.println("--> Falha de comunicacao com o medidor");
      rs485.vetor[0] = 00;      //Parte Alta Word 1
      rs485.vetor[1] = 00;      //Parte Baixa Word 1
      rs485.vetor[2] = 00;      //Parte Alta Word 2
      rs485.vetor[3] = 00;      //Parte Baixa Word 2
    }
    vetor_resultado[i] = rs485.valorFloat;
    Serial.print("Indice: ");
    Serial.println(i);
    Serial.print("Valor: ");
    Serial.println(rs485.valorFloat);
    Serial.println("---------------------------------------------------");
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
