// Fonte do código
// https://easytromlabs.com/arduino/arduino-lab-20-leitura-dos-dados-de-um-multimedidor-trifasico-de-energia-via-rs-485-modbus-rtu/

// União entre o código millis e o Modbus.

/// Exemplo de Uma Aplicação Millis

// Tentar apresentar todas as varáveis m uma única apresentação

// deu certo o V0
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
#include <MySQL_Connection.h>
#include <MySQL_Cursor.h>
#include <avr/wdt.h>
///////////////////////////////////////////////////

////////////////////////// Variáveis Globais de rede ////////////////////////////////////////
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
//IPAddress ip(10, 244, 1, 4);
IPAddress ip(192, 168, 1, 4);
IPAddress server(185, 42, 117, 115);  // Meu Computador
char user[] = "u4gnbqjncr8kcznp";              
char password[] = "clFpGRTokoJfirSUAj9f"; 

char sentenca[512];


char INSERT_SQL[] = "INSERT INTO bzayjfy8zw7gkcajc7aj.Kron (Dispositivo, VBD0, IBD0, FPBD, S0BD,  Q0BD, P0BD,FBD ,V1BD, V2BD, V3BD,IL1BD, IL2BD, IL3BD,PABD, PBBD, PCBD,QABD, QBBD, QCBD,  FPABD, FPBBD, FPCBD, EAPBD) VALUES ( '%d', '%s', '%s','%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s','%s','%s')";

EthernetClient cliente;
MySQL_Connection conn((Client *)&cliente);
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

float vetor_resultado[23];    //vetor que rece todos os resultados medidos

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Variáveis 



//Union para obter o valor de leitura:
union leitura {           //leitura para transformar o valor em float das grandezas elétricas
  unsigned char vetor[4];
  float valorFloat;
  float V0;
  float I0;
  float FP;
  float S0;
  float Q0;
  float P0;
  float F;
  float V1;
  float V2;
  float V3;
  float IL1;
  float IL2;
  float IL3;
  float PA;
  float PB;
  float PC;
  float QA;
  float QB;
  float QC;
  float FPA;
  float FPB;
  float FPC;
  float EAP;
} rs485;    

char Converter_V0[5];
char VBD0[5];
char Converter_I0[5];
char IBD0[5];
char Converter_FP[5];
char FPBD[5];
char Converter_S0[5];
char S0BD[5];
char Converter_Q0[5];
char Q0BD[5];
char Converter_P0[5];
char P0BD[5];
char Converter_F[5];
char FBD[5];
char Converter_V1[5];
char V1BD[5];
char Converter_V2[5];
char V2BD[5];
char Converter_V3[5];
char V3BD[5];
char Converter_IL1[5];
char IL1BD[5];
char Converter_IL2[5];
char IL2BD[5];
char Converter_IL3[5];
char IL3BD[5];
char Converter_PA[5];
char PABD[5];
char Converter_PB[5];
char PBBD[5];
char Converter_PC[5];
char PCBD[5];
char Converter_QA[5];
char QABD[5];
char Converter_QB[5];
char QBBD[5];
char Converter_QC[5];
char QCBD[5];
char Converter_FPA[5];
char FPABD[5];
char Converter_FPB[5];
char FPBBD[5];
char Converter_FPC[5];
char FPCBD[5];
char Converter_EAP[5];
char EAPBD[5];
char Converter[5];


// Habilita as Variáveis de Tempo
 unsigned long Tempo = millis();
unsigned long TempoSubtraido;
unsigned long TempoDeEspera = 1000;
unsigned long TempoDeReset = 6000000;// 60000;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************
  Funções Modbus
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

     wdt_disable();
  
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

   if(TempoDeReset < millis())
  {   
    Serial.println("Reset");
    wdt_enable(WDTO_15MS);
    TempoDeReset = 100 + millis();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************
  Fazendo conexão com a rede e Banco de dados
*************************************************************/
void Internet()
{

   Serial.println("Executando sentença");
   MySQL_Cursor *cur = new MySQL_Cursor(&conn);
    
   Serial.println("Conectando...");
   if (conn.connected()) 
   {
       sprintf(sentenca, INSERT_SQL,1,VBD0,IBD0,FPBD,S0BD,Q0BD,P0BD,FBD,V1BD,V2BD,V3BD,IL1BD,IL2BD,IL3BD,PABD,PBBD,PCBD,QABD,QBBD,QCBD,FPABD,FPBBD,FPCBD,EAPBD);      
  
        cur->execute(sentenca); 
      delete cur; 
   }
   else
   {
      conn.close();
      Serial.println("Connecting...");
      delay(200); //
      if (conn.connect(server,3306,user,password)) 
      {
        delay(500);
        Serial.println("Successful reconnect!");
     
      }
      else 
      {
          Serial.println("Cannot reconnect! Drat.");
      }
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

    rs485.V0 = vetor_resultado[0];
    dtostrf(rs485.V0, 5, 2, VBD0);       
    snprintf(Converter_V0, sizeof(Converter_V0),"%.2f",Converter_V0);  
    Serial.print("Indice: 0 ");
    Serial.print("Valor: ");
    Serial.print(rs485.V0);
    Serial.print(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 0 ");
    Serial.print("Valor: ");
    Serial.print(VBD0);
    Serial.print(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
      

    rs485.I0 = vetor_resultado[1];
    dtostrf(rs485.I0, 5, 2, IBD0);  
    snprintf(Converter_I0, sizeof(Converter_I0),"%.2f",Converter_I0); 
    Serial.print("Indice: 1 ");
    Serial.print("Valor: ");
    Serial.print(rs485.I0);
    Serial.print(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 1 ");
    Serial.print("Valor: ");
    Serial.print(IBD0);
    Serial.print(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
    
    rs485.FP = vetor_resultado[2];
    dtostrf(rs485.FP, 5, 2, FPBD);  
    snprintf(Converter_FP, sizeof(Converter_FP),"%.2f",Converter_FP);
    Serial.print("Indice: 2 ");
    Serial.print("Valor: ");
    Serial.print(rs485.FP);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 2 ");
    Serial.print("Valor: ");
    Serial.print(FPBD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    
    
    rs485.S0 = vetor_resultado[3];
    dtostrf(rs485.S0, 5, 2,  S0BD);
    snprintf(Converter_S0, sizeof(Converter_S0),"%.2f",Converter_S0);
    Serial.print("Indice: 3 ");
    Serial.print("Valor: ");
    Serial.print(rs485.S0);
    Serial.print(" VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 3 ");
    Serial.print("Valor: ");
    Serial.print(S0BD);
    Serial.println(" VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    
    
    rs485.Q0 = vetor_resultado[4];
    dtostrf(rs485.Q0, 5, 2, Q0BD);  
    snprintf(Converter_Q0, sizeof(Converter_Q0),"%.2f",Converter_Q0);
    Serial.print("Indice: 4 ");
    Serial.print("Valor: ");
    Serial.print(rs485.Q0);
    Serial.print(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 4 ");
    Serial.print("Valor: ");
    Serial.print(Q0BD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    
    rs485.P0 = vetor_resultado[5];
    dtostrf(rs485.P0, 5, 2, P0BD);  
    snprintf(Converter_P0, sizeof(Converter_P0),"%.2f",Converter_P0);
    Serial.print("Indice: 5 ");
    Serial.print("Valor: ");
    Serial.print(rs485.P0);
    Serial.print(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 5 ");
    Serial.print("Valor: ");
    Serial.print(P0BD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    
    rs485.F = vetor_resultado[6];
    dtostrf(rs485.F, 5, 2, FBD);  
    snprintf(Converter_F, sizeof(Converter_F),"%.2f",Converter_F);
    Serial.print("Indice: 6 ");
    Serial.print("Valor: ");
     Serial.print(rs485.F);
    Serial.print(" Hz ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 6 ");
    Serial.print("Valor: ");
    Serial.print(FBD);
    Serial.println(" Hz ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
    
    rs485.V1 = vetor_resultado[7];
    dtostrf(rs485.V1, 5, 2,  V1BD); 
    snprintf(Converter_V1, sizeof(Converter_V1),"%.2f",Converter_V1);
    Serial.print("Indice: 7 ");
    Serial.print("Valor: ");
     Serial.print(rs485.V1);
    Serial.print(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
     Serial.print("Indice: 7 ");
    Serial.print("Valor: ");
    Serial.print( V1BD);
    Serial.println(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
    
    rs485.V2 = vetor_resultado[8];
    dtostrf(rs485.V2, 5, 2,V2BD); 
    snprintf(Converter_V2, sizeof(Converter_V2),"%.2f",Converter_V2);
    Serial.print("Indice: 8 ");
    Serial.print("Valor: ");
     Serial.print(rs485.V2);
    Serial.print(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 8 ");
    Serial.print("Valor: ");
    Serial.print(V2BD);
    Serial.println(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
    
    rs485.V3 = vetor_resultado[9];
    dtostrf(rs485.V3, 5, 2, V3BD); 
    snprintf(Converter_V3, sizeof(Converter_V3),"%.2f",Converter_V3);
    Serial.print("Indice: 9 ");
    Serial.print("Valor: ");
     Serial.print(rs485.V3);
    Serial.print(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 9 ");
    Serial.print("Valor: ");
    Serial.print(V3BD);
    Serial.println(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
  
    rs485.IL1 = vetor_resultado[10];
    dtostrf(rs485.IL1, 5, 2,IL1BD); 
    snprintf(Converter_IL1, sizeof(Converter_IL1),"%.2f",Converter_IL1);
    Serial.print("Indice: 10 ");
    Serial.print("Valor: ");
    Serial.print(rs485.IL1);
    Serial.print(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 10 ");
    Serial.print("Valor: ");
    Serial.print(IL1BD);
    Serial.println(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");  
    
    rs485.IL2 = vetor_resultado[11];
    dtostrf(rs485.IL2, 5, 2, IL2BD); 
    snprintf(Converter_IL2, sizeof(Converter_IL2),"%.2f",Converter_IL2);
    Serial.print("Indice: 11 ");
    Serial.print("Valor: ");
    Serial.print(rs485.IL2);
    Serial.print(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 11 ");
    Serial.print("Valor: ");
    Serial.print(IL2BD);
    Serial.println(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
  
    rs485.IL3 = vetor_resultado[12];
    dtostrf(rs485.IL3, 5, 2, IL3BD); 
    snprintf(Converter_IL3, sizeof(Converter_IL3),"%.2f",Converter_IL3);
    Serial.print("Indice: 12 ");
    Serial.print("Valor: ");
     Serial.print(rs485.IL3);
    Serial.print(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 12 ");
    Serial.print("Valor: ");
    Serial.print(IL3BD);
    Serial.println(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
  
    rs485.PA = vetor_resultado[13];
    dtostrf(rs485.PA, 5, 2, PABD);
     snprintf(Converter_PA, sizeof(Converter_PA),"%.2f",Converter_PA);
    Serial.print("Indice: 13 ");
    Serial.print("Valor: ");
     Serial.print(rs485.PA);
    Serial.print(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 13 ");
    Serial.print("Valor: ");
    Serial.print(PABD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");  
  
    rs485.PB = vetor_resultado[14];
    dtostrf(rs485.PB, 5, 2, PBBD); 
    snprintf(Converter_PB, sizeof(Converter_PB),"%.2f",Converter_PB);
    Serial.print("Indice: 14 ");
    Serial.print("Valor: ");
     Serial.print(rs485.PB);
    Serial.print(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 14 ");
    Serial.print("Valor: ");
    Serial.print(PBBD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
  
    rs485.PC = vetor_resultado[15];
    dtostrf(rs485.PC, 5, 2, PCBD);
    snprintf(Converter_PC, sizeof(Converter_PC),"%.2f",Converter_PC); 
    Serial.print("Indice: 15 ");
    Serial.print("Valor: ");
     Serial.print(rs485.PC);
    Serial.print(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 15 ");
    Serial.print("Valor: ");
    Serial.print(PCBD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
  
    rs485.QA = vetor_resultado[16];
    dtostrf(rs485.QA, 5, 2, QABD); 
    snprintf(Converter_QA, sizeof(Converter_QA),"%.2f",Converter_QA); 
    Serial.print("Indice: 16 ");
    Serial.print("Valor: ");
     Serial.print(rs485.QA);
    Serial.print(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 16 ");
    Serial.print("Valor: ");
    Serial.print(QABD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
  
    rs485.QB = vetor_resultado[17];
    dtostrf(rs485.QB, 5, 2, QBBD); 
    snprintf(Converter_QB, sizeof(Converter_QB),"%.2f",Converter_QB); 
    Serial.print("Indice: 17 ");
    Serial.print("Valor: ");
     Serial.print(rs485.QB);
    Serial.print(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 17 ");
    Serial.print("Valor: ");
    Serial.print(QBBD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
  
    rs485.QC = vetor_resultado[18];
    dtostrf(rs485.QC, 5, 2, QCBD); 
    snprintf(Converter_QC, sizeof(Converter_QC),"%.2f",Converter_QC);
    Serial.print("Indice: 18 ");
    Serial.print("Valor: ");
    Serial.print(rs485.QC);
    Serial.print(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 18 ");
    Serial.print("Valor: ");
    Serial.print(QCBD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
  
    rs485.FPA = vetor_resultado[19];
    dtostrf(rs485.FPA, 5, 2, FPABD); 
    snprintf(Converter_FPA, sizeof(Converter_FPA),"%.2f",Converter_FPA);
    Serial.print("Indice: 19 ");
    Serial.print("Valor: ");
     Serial.print(rs485.FPA);
    Serial.print(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 19 ");
    Serial.print("Valor: ");
    Serial.print(FPABD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
  
    rs485.FPB = vetor_resultado[20];
    dtostrf(rs485.FPB, 5, 2, FPBBD); 
    snprintf(Converter_FPB, sizeof(Converter_FPB),"%.2f",Converter_FPB);
    Serial.print("Indice: 20 ");
    Serial.print("Valor: ");
     Serial.print(rs485.FPB);
    Serial.print(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 20 ");
    Serial.print("Valor: ");
    Serial.print(FPBBD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
  
    rs485.FPC = vetor_resultado[21];
    dtostrf(rs485.FPC, 5, 2, FPCBD);
    snprintf(Converter_FPC, sizeof(Converter_FPC),"%.2f",Converter_FPC); 
    Serial.print("Indice: 21 ");
    Serial.print("Valor: ");
    Serial.print(rs485.FPC);
    Serial.print(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 21 ");
    Serial.print("Valor: ");
    Serial.print(FPCBD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------"); 
  
    rs485.EAP = vetor_resultado[22];
    dtostrf(rs485.EAP, 5, 2, EAPBD); 
    snprintf(Converter_EAP, sizeof(Converter_EAP),"%.2f",Converter_EAP);
    Serial.print("Indice: 22 ");
    Serial.print("Valor: ");
     Serial.print(rs485.EAP);
    Serial.print(" KWh ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 22 ");
    Serial.print("Valor: ");
    Serial.print(EAPBD);
    Serial.println(" KWh");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////// Mudando de Vetor

    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 0 ");
    Serial.print("Valor: ");
    Serial.print(VBD0);
    Serial.print(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 1 ");
    Serial.print("Valor: ");
    Serial.print(IBD0);
    Serial.print(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 2 ");
    Serial.print("Valor: ");
    Serial.print(FPBD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 3 ");
    Serial.print("Valor: ");
    Serial.print(S0BD);
    Serial.println(" VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 4 ");
    Serial.print("Valor: ");
    Serial.print(Q0BD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 5 ");
    Serial.print("Valor: ");
    Serial.print(P0BD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 6 ");
    Serial.print("Valor: ");
    Serial.print(FBD);
    Serial.println(" Hz ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

 Serial.println("---------------------------------------------------");
    Serial.print("Indice: 7 ");
    Serial.print("Valor: ");
    Serial.print(V1BD);
    Serial.println(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 8 ");
    Serial.print("Valor: ");
    Serial.print(V2BD);
    Serial.println(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 9 ");
    Serial.print("Valor: ");
    Serial.print(V3BD);
    Serial.println(" V ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 10 ");
    Serial.print("Valor: ");
    Serial.print(IL1BD);
    Serial.println(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 11 ");
    Serial.print("Valor: ");
    Serial.print(IL2BD);
    Serial.println(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 12 ");
    Serial.print("Valor: ");
    Serial.print(IL3BD);
    Serial.println(" A ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 13 ");
    Serial.print("Valor: ");
    Serial.print(PABD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 14 ");
    Serial.print("Valor: ");
    Serial.print(PBBD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 15 ");
    Serial.print("Valor: ");
    Serial.print(PCBD);
    Serial.println(" W ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 16 ");
    Serial.print("Valor: ");
    Serial.print(QABD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

Serial.println("---------------------------------------------------");
    Serial.print("Indice: 17 ");
    Serial.print("Valor: ");
    Serial.print(QBBD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 18 ");
    Serial.print("Valor: ");
    Serial.print(QCBD);
    Serial.println(" VAr ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 19 ");
    Serial.print("Valor: ");
    Serial.print( FPABD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 20 ");
    Serial.print("Valor: ");
    Serial.print(FPBBD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 21 ");
    Serial.print("Valor: ");
    Serial.print(FPCBD);
    Serial.println(" W/VA ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");

    Serial.println("---------------------------------------------------");
    Serial.print("Indice: 22 ");
    Serial.print("Valor: ");
    Serial.print(EAPBD);
    Serial.println(" KWh ");
    Serial.println("  ");
    Serial.println("---------------------------------------------------");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
