#include <ArduinoWebsockets.h>
#include <WiFi.h>

#define zeroCross 34

const uint8_t btn[]={0,13,12,14,27,26,25,33,32,35}; //Botoes de entrada esp total de 9
bool  flagBtn[] = {0,0,0,0,0,0,0,0,0,0,0};
bool  btnPressed[] = {0,0,0,0,0,0,0,0,0,0,0};
bool  btnSolto[] = {0,0,0,0,0,0,0,0,0,0,0};
bool  tipoBtn[]={0,0,1,1,1,1,1,1,1,1,0}; // 0 pulso, 1 comun, ultimo btn = botao config
uint8_t debounceTecla = 10;

const uint8_t saida[]={0,4,16,17,5,18,19,21,22,23}; //SAidas esp total de 9
bool  flagSaida[] = {0,0,0,0,0,0,0,0,0,0};
uint8_t saidaBtn[]= {0,1,2,3,4,5,6,7,8,9};  //Saida que o botao vai acionar
uint8_t tipoSaida[]=  {0,1,0,0,0,2,2,2,2,2};  //tipo saida, 0 = comum / 1 = Dimmer / 2 = desativada
uint8_t dimSaida[]= {0,0,0,0,0,0,0,0,0,0};  //valor do dimmer
bool dimUpDown[]=   {0,0,0,0,0,0,0,0,0,0};  //flag altera direção do dimmer

uint64_t  holdMillis[11];                   //millis para controle botao segurado
uint16_t  timeLongo[11];                    //tempo espera botao longo
bool flagHold[]=   {0,0,0,0,0,0,0,0,0,0,0};  //flag botao segurado
bool longPress[]=  {0,0,0,0,0,0,0,0,0,0,0};  //flag botao Longo
uint16_t tempoEspera = 1000;                //tempo espera para interações tecla segurada
uint8_t tempoChange = 20;     //tempo para incremento decremento brilho;

bool autoFade = 0;  //flag fade automatico;
uint8_t tempoAutoFade = 10;
bool fadeIn[]=   {0,0,0,0,0,0,0,0,0,0,0};  //flag fade liga;
bool fadeOut[]=   {0,0,0,0,0,0,0,0,0,0,0};  //flag fade desliga;
int16_t brilho = 50;

const uint8_t ledOut[]={2,15}; //Saidas led, ledout0 = ledBuilt
bool flagLed[]={0,0};

const char* ssid = "JR BJJ"; //Enter SSID
const char* password = "CASA12345"; //Enter Password
const byte maxClients = 4;

using namespace websockets;
WebsocketsClient clients[maxClients];
WebsocketsServer server;

#define maxBrightness 1000 // brilho maximo em us
#define minBrightness 8200 // brilho minimo em us
#define TRIGGER_TRIAC_INTERVAL 50 // tempo quem que o triac fica acionado
#define IDLE -1
volatile uint8_t PINO_DIM;
volatile bool isPinHighEnabled = false;
volatile long currentBrightness = minBrightness;
int brilho_convertido = 0;
bool brilhoMudou = 0;

hw_timer_t * timerToPinHigh;
hw_timer_t * timerToPinLow;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile bool ledDebug = 0;
volatile uint8_t countDimmer = 0;
volatile int16_t dimmer[5] ={7480,6040,4600,3880,2800};

bool enviarEstado = 0;


void IRAM_ATTR ISR_turnPinLow(){ // desliga o pino dim
  portENTER_CRITICAL_ISR(&mux); // desativa interrupÃ§oes
    digitalWrite(PINO_DIM, LOW);
    isPinHighEnabled = false;
  portEXIT_CRITICAL_ISR(&mux); // ativa as interrupÃ§oes novamente
}

void IRAM_ATTR setTimerPinLow(){ // executa as configuracoes de pwm e aplica os valores da luminosidade ao dimmer no tempo em que ra ficar em low
  timerToPinLow = timerBegin(2, 80, true);
  timerAttachInterrupt(timerToPinLow, &ISR_turnPinLow, true);
  timerAlarmWrite(timerToPinLow, TRIGGER_TRIAC_INTERVAL, false);
  timerAlarmEnable(timerToPinLow);
}

void IRAM_ATTR ISR_turnPinHigh(){ // liga o pino dim
  portENTER_CRITICAL_ISR(&mux);  // desativa interrupÃ§oes
    digitalWrite(PINO_DIM, HIGH); 
    setTimerPinLow();
  portEXIT_CRITICAL_ISR(&mux); // ativa as interrupÃ§oes novamente
}

void IRAM_ATTR setTimerPinHigh(long brightness){ // executa as configuracoes de pwm e aplica os valores da luminosidade ao dimmer no tempo que ira ficar em high
  isPinHighEnabled = true;
  timerToPinHigh = timerBegin(1, 80, true);
  timerAttachInterrupt(timerToPinHigh, &ISR_turnPinHigh, true);
  timerAlarmWrite(timerToPinHigh, brightness, false); //brightness, false);
  timerAlarmEnable(timerToPinHigh);
}

void IRAM_ATTR ISR_zeroCross()  {// funÃ§ao que Ã© chamada ao dimmer registrar passagem por 0
  ledDebug = !ledDebug;
  digitalWrite(ledOut[1], ledDebug);
  if(currentBrightness == IDLE) return;
  
  portENTER_CRITICAL_ISR(&mux); // desativa interrupÃ§oes
    //digitalWrite(PINO_DIM, LOW);
    //isPinHighEnabled = false;
    if(!isPinHighEnabled){
      setTimerPinHigh(currentBrightness); // define o brilho      
    } 
  portEXIT_CRITICAL_ISR(&mux); // ativa as interrupÃ§oes novamente
}

void turnLightOn(){ // liga o dimmer no brilho maximo
  portENTER_CRITICAL(&mux);// desativa interrupÃ§oes
    currentBrightness = maxBrightness;
    digitalWrite(PINO_DIM, HIGH);
  portEXIT_CRITICAL(&mux);// ativa as interrupÃ§oes novamente
}

void turnLightOff(){// deliga o dimmer
  portENTER_CRITICAL(&mux); // desativa interrupÃ§oes
    currentBrightness = IDLE;
    digitalWrite(PINO_DIM, LOW);
  portEXIT_CRITICAL(&mux); // ativa as interrupÃ§oes novamente
}


void verificaBtns(){ 
  for(int i=1; i<=sizeof(btn)-1; i++){    //botao pressionado
    if(digitalRead(btn[i]) == 0 && flagBtn[i] == 0){
      delay(debounceTecla); //debounce delay
      if(digitalRead(btn[i]) == 0){ //se continua apertado liga flag
        flagBtn[i] = 1;             //flag btn
        btnPressed[i] = 1;          //flag btn pressionado 
        Serial.printf("Botao %2d pressionado, Entrada %d\n", i, btn[i]);
        if(tipoSaida[i] ==1){  //  se o tipo de saida for "1" saida dimmer executa ações para tecla segurada
          holdMillis[i] = millis();   //carrega holdmillis para verificar botao segurado
          timeLongo[i] = tempoEspera; //2000;        //carrega tempo para verificar tecla segurada
          flagHold[i] = 1;            // seta flag que botao foi pressionado
          longPress[i] = 0;
        }
      }
    }
  }

  for(int i =1; i<=sizeof(btn)-1; i++){   //botao segurado
    if(flagHold[i] == 1){
      if(millis() - holdMillis[i] > timeLongo[i]){
        holdMillis[i] = millis();
        timeLongo[i] = tempoChange; //20;carrega tempo incremento decremento brilho
        longPress[i] = 1;
       // Serial.printf("Botao %2d Hold, Entrada %d\n", i, btn[i]);
      }
    }
  }
  

  for(int i=1; i<=sizeof(btn)-1; i++){    //botao solto
    if(digitalRead(btn[i]) && flagBtn[i]){
      delay(debounceTecla);
      if(digitalRead(btn[i])){
        flagBtn[i] = 0;
        btnSolto[i] = 1;          //flag btn solto
        Serial.printf("Botao %2d solto, Entrada %d\n", i, btn[i]);
        flagHold[i] = 0;    //apaga flag botao segurado       
      }
    }
  }  
}

void saidas(){
  for(int i =1; i<= sizeof(btnPressed)-1; i++){
    if(btnPressed[i]){
      btnPressed[i] = 0;
      if(tipoSaida[i] == 0){  // se o tipo de saida for "0" saida comum executa ações para esta saida
        
        flagSaida[saidaBtn[i]] = !flagSaida[saidaBtn[i]];
        digitalWrite(saida[saidaBtn[i]], flagSaida[saidaBtn[i]]);
        enviarEstado = 1; //envia alterações para todos clientes
      }
      if(tipoSaida[i] == 1){
        
      }          
    }
  }

  for(int i =1; i<= sizeof(longPress)-1; i++){
    if(longPress[i]){
      longPress[i] = 0;

      if(dimUpDown[i] == 1)dimSaida[i]++;
      else dimSaida[i]--;

      if(dimSaida[i] == 0){
        dimSaida[i]++;
        dimUpDown[i] = 1; //flag = 1, subir dimmer        
      }else if(dimSaida[i] >=100){
        dimSaida[i]--;
        dimUpDown[i] = 0; //flag = 0, descer dimmer
      }

      

      dimSaida[i] = constrain(dimSaida[i], 0, 100); //mantem, dentro dos limites
      //brilho = map(dimSaida[i], 0,100,0,1023);
      brilho = map(dimSaida[i], 100, 0, maxBrightness, minBrightness); //converte a luminosidade em microsegundos 
      portENTER_CRITICAL(&mux); //desliga as interrupÃ§oes
      currentBrightness = brilho; // altera o brilho
      portEXIT_CRITICAL(&mux);// liga as interrupÃ§oes    
            
    }
  }
  
  for(int i =1; i<= sizeof(btnSolto)-1; i++){
    if(btnSolto[i]){
      btnSolto[i] = 0;
      if(tipoSaida[i] == 0){  // se o tipo de saida for "0" saida comum executa ações para esta saida
        if(tipoBtn[i]==1){ // se for comum alterna saida
          flagSaida[saidaBtn[i]] = !flagSaida[saidaBtn[i]];
          enviarEstado = 1; //envia alterações para todos clientes  
        }
        digitalWrite(saida[saidaBtn[i]], flagSaida[saidaBtn[i]]);
      }
      if(tipoSaida[i] == 1){  //  se o tipo de saida for "1" saida dimmer executa ações para esta saida
        if(timeLongo[i] == tempoEspera){ // ainda nao entrou hold, entao liga ou desliga 100%
          if(fadeIn[i] || fadeOut[i]){
            fadeIn[i]=0;
            fadeOut[i]=0;
            enviarEstado = 1; //envia alterações para todos clientes
          }
          else{
            if(dimSaida[i] == 0)fadeIn[i] = 1; //se lampada esta desligada, liga lampada 100%
            else fadeOut[i] = 1;                  
          }                 
        }

        if(timeLongo[i] == tempoChange){  //ja entrou em dimmer manual
          dimUpDown[i] = !dimUpDown[i];
        }      
      }    
    }
  }  

  if(autoFade == 1)delay(tempoAutoFade);
  autoFade = 0; //apaga flag auto fade para ver se ainda tem fade occorendo
  for(int i = 1; i<=sizeof(fadeIn)-1; i++){
    if(fadeIn[i] || fadeOut[i]){
      if(fadeIn[i])dimSaida[i] ++;
      if(fadeOut[i])dimSaida[i]--;
      if(dimSaida[i] <= 0 || dimSaida[i] >= 100){
        fadeOut[i] = 0;
        fadeIn[i] = 0;
        dimSaida[i] = constrain(dimSaida[i], 0, 100); //mantem, dentro dos limites
        enviarEstado = 1; //envia alterações para todos clientes        
      }
      //brilho = map(dimSaida[i], 0,100,0,1023);      
      if(dimSaida[i] != 0)brilho = map(dimSaida[i], 100, 0, maxBrightness, minBrightness); //converte a luminosidade em microsegundos
      else brilho = IDLE; 
      portENTER_CRITICAL(&mux); //desliga as interrupÃ§oes
      currentBrightness = brilho; // altera o brilho
      portEXIT_CRITICAL(&mux);// liga as interrupÃ§oes    

      autoFade = 1; //se tem fade ocorrendo seta flag 
    }
  }

  if(brilhoMudou){
    brilhoMudou = 0;
  }  
}

void piscaLed(){
  static uint8_t countLed = 0;
  static uint64_t tempoLed = 0;
  if(millis() - tempoLed >= 200){
    tempoLed = millis();
    countLed++;
    if(countLed > 4)countLed = 0;
    if(countLed == 0)digitalWrite(ledOut[0], 1);
    else if(countLed == 1)digitalWrite(ledOut[0], 0);
  }
}

void configPins(){
  Serial.print("Pinos ");
  for(char i=1; i<=sizeof(saida)-1; i++){
    pinMode(saida[i], OUTPUT);
    digitalWrite(saida[i], 0);
    Serial.printf("%2d, ", saida[i]);
  }
  Serial.printf("configurados como saida, Total de %d pinos\n", sizeof(saida)-1);

  Serial.print("Tipo Saida: 0=Comun, 1=Dimmer, 2= Off ");
  for(char i=1; i<=sizeof(tipoSaida)-1; i++){    
    Serial.printf("%2d, ", tipoSaida[i]);
  }
  Serial.printf("Total de %d Saidas\n", sizeof(tipoSaida)-1);

  Serial.print("Pinos ");
  for(char i=1; i<=sizeof(btn)-1; i++){
    pinMode(btn[i], INPUT_PULLUP);
    Serial.printf("%2d, ",btn[i]);
  }
  Serial.printf("configurados como entrada, Total de %d pinos\n", sizeof(btn)-1);

  Serial.print("Tipo Btn ");
  for(char i=1; i<=sizeof(tipoBtn)-1; i++){    
    Serial.printf("%2d, ", tipoBtn[i]);
  }
  Serial.printf("Total de %d Btns\n", sizeof(tipoBtn)-1);

  Serial.print("Pinos ");
  for(char i=0; i<sizeof(ledOut); i++){
    pinMode(ledOut[i], OUTPUT);
    Serial.printf("%2d, ",ledOut[i]);
  }
  Serial.printf("configurados como Led, Total de %d pinos\n", sizeof(ledOut));

  pinMode(zeroCross, INPUT_PULLUP);
  Serial.printf("Pino %d configurado como Zero Cross\n", zeroCross);
}

void configWiFi(){
   WiFi.begin(ssid, password);
  
  for(int i = 0; i < 15 && WiFi.status() != WL_CONNECTED; i++) {
      Serial.print(".");
      delay(1000);
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());   //You can get IP address assigned to ESP

  server.listen(80);
  Serial.print("Is server live? ");
  Serial.println(server.available()); 
}

void handleMessage(WebsocketsClient &client, WebsocketsMessage message) { //Recebeu mensagem tratar mensagem

  auto data = message.data();
  //String estado ="";

  // Log message
  Serial.print("Mensagem Recebida: ");
  Serial.print(data);

  String msg ="OK=1 ";

  if(data.indexOf("consulta") != -1)enviarEstado = 1; //envia alterações para todos clientes;

  if(data.indexOf("OK?") != -1){
    client.send(msg);
    Serial.print(" >Mensagem OK enviada");
  }
  Serial.println(" ");

  if(data.indexOf("tipoSaida") != -1){
    msg = msg + "tipoSaida";
    for(uint8_t i=1; i<=sizeof(tipoSaida)-1; i++){
      msg = msg + "<" + String(i) + "=" + String(tipoSaida[i]) + ">";      
    }
    client.send(msg);
    Serial.print("mensagem enviada");
    Serial.println(msg);    
  }

  if(data.indexOf("changeOut") != -1){
    int inicio;
    int fim;
    enviarEstado = 1; //envia alterações para todos clientes
    if(data.indexOf("<2:") != -1){
      inicio = data.indexOf("<2:") +3;

      fim = data.indexOf(">", inicio);     

      String estado = data.substring(inicio, fim);
      if(estado.toInt() == 1){
        flagSaida[saidaBtn[2]] = !flagSaida[saidaBtn[2]]; 
      }
      else if(estado.toInt() == 0){
        flagSaida[saidaBtn[2]] = 0;
      }
      Serial.println("Saida 2 =" + estado);
      //flagSaida[saidaBtn[2]] = estado.toInt();
      Serial.println(flagSaida[saidaBtn[2]]);
      digitalWrite(saida[saidaBtn[2]], flagSaida[saidaBtn[2]]);
    }

    if(data.indexOf("<3:") != -1){
      inicio = data.indexOf("<3:") +3;

      fim = data.indexOf(">", inicio);     

      String estado = data.substring(inicio, fim);
      if(estado.toInt() == 1){
        flagSaida[saidaBtn[3]] = !flagSaida[saidaBtn[3]]; 
      }
      else if(estado.toInt() == 0){
        flagSaida[saidaBtn[3]] = 0;
      }
      Serial.println("Saida 3 =" + estado);
      //flagSaida[saidaBtn[2]] = estado.toInt();
      Serial.println(flagSaida[saidaBtn[3]]);
      digitalWrite(saida[saidaBtn[3]], flagSaida[saidaBtn[3]]);
    }

    if(data.indexOf("<1:") != -1){
      inicio = data.indexOf("<1:") +3;

      fim = data.indexOf(">", inicio);     

      String estado = data.substring(inicio, fim);

      dimSaida[1] = estado.toInt(); 
      dimSaida[1] = constrain(dimSaida[1], 0, 100);
      if(dimSaida[1] > 0)brilho = map(dimSaida[1], 100, 0, maxBrightness, minBrightness); //converte a luminosidade em microsegundos   
      else  brilho = IDLE;     
      
      portENTER_CRITICAL(&mux); //desliga as interrupÃ§oes
      currentBrightness = brilho; // altera o brilho
      portEXIT_CRITICAL(&mux);// liga as interrupÃ§oes      
      Serial.println("Dim 1 =" + estado);
      //flagSaida[saidaBtn[2]] = estado.toInt();      
    }

  }

  static uint8_t j = 0; j++; if(j>2)j = 0;
  String teste = "Teste" + String(j);
  Serial.println(teste);
  Serial.println(teste.indexOf("1"));


  /*if(data.indexOf(dsp1) != -1){
    statusLed =! statusLed;
    flagAttDispositivos = 1;
  }*/  
}

void handleEvent(WebsocketsClient &client, WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.print("Connection closed ");
    Serial.println(data);
  }
}

int8_t getFreeClientIndex() {
  static uint8_t indexVago = 3;
  // If a client in our list is not available, it's connection is closed and we
  // can use it for a new client.  
  for (byte i = 0; i < maxClients; i++) {
    if (!clients[i].available()) return i;    
  }
  indexVago++;
  if(indexVago >= maxClients)indexVago = 0; // se chegou aqui precisa liberar conexão
  clients[indexVago].close();
  Serial.printf("Cliente desconectado no index %d para liberar conexão\n", indexVago);
  return indexVago;
  //return -1;
}

void listenForClients() {
  if (server.poll()) {
    int8_t freeIndex = getFreeClientIndex();
    if (freeIndex >= 0) {
      WebsocketsClient newClient = server.accept();
      Serial.printf("Accepted new websockets client at index %d\n", freeIndex);
      newClient.onMessage(handleMessage);
      newClient.onEvent(handleEvent);      
      //newClient.send("Hello from Teensy");
      clients[freeIndex] = newClient;
      //flagAttDispositivos = 1;
    }
  }
}

bool enviarMsg(String dados){
  bool tx = 0;
  for (byte i = 0; i < maxClients; i++) {
    if (clients[i].available()){
      String msg ="";
      clients[i].send(dados);
      msg = msg + "dados enviados ao cliente " + i +" = " + dados;
      Serial.println(msg);
      tx = 1;
    } 
  }
  return tx;
}

void pollClients() {
  for (byte i = 0; i < maxClients; i++) {
    clients[i].poll();
  }
}

void enviarEstadoSaidas(){
  if(enviarEstado == 0)return;
  enviarEstado = 0;
  String  payload="changeOut";
  for(int i =1; i<= sizeof(flagSaida)-1; i++){    
    if(tipoSaida[i] == 0){  // se o tipo de saida for "0" saida comum executa ações para esta saida
      payload = payload + "<" + String(saidaBtn[i]) + ":" + String(flagSaida[saidaBtn[i]]) + ">";     
    }
    if(tipoSaida[i] == 1){
      payload = payload + "<" + String(saidaBtn[i]) + ":" + String(dimSaida[i]) + ">";  
    }    
  }
  enviarMsg(payload);
}

void setup() {
  Serial.begin(115200);
  configPins();
  configWiFi();

  currentBrightness = IDLE; //inicia dimmer desligado
  attachInterrupt(digitalPinToInterrupt(zeroCross), ISR_zeroCross, RISING);
  PINO_DIM = saida[1];
}

void loop() {
  piscaLed();
  verificaBtns();
  saidas();
  listenForClients();
  pollClients();
  enviarEstadoSaidas();
}

