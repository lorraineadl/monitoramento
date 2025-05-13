# Sistema de Monitoramento de Vazamentos com ESP32 e MQTT

Este repositório contém todo o material necessário para reproduzir, entender e expandir o projeto de monitoramento de consumo de água e detecção de vazamentos utilizando um ESP32.

---

## Hardware Utilizado

| Componente       | Descrição                                   |
|------------------|-----------------------------------------------|
| ESP32            | Microcontrolador principal                   |
| Encoder Rotativo | Simulação do fluxo de água                   |
| Relé             | Simula válvula de interrupção de fluxo       |
| LED Verde        | Indica funcionamento normal                  |
| LED Amarelo      | Indica alerta de vazamento                   |

## Documentação e Comunicação via Internet (MQTT)

Documentação de Interfaces e Protocolos
Wi‑Fi (IEEE 802.11): Conexão do ESP32 à rede local.

MQTT (TCP/IP): Protocolo de mensageria leve.

Broker: broker.hivemq.com:1883

A comunicação MQTT funciona da seguinte forma:

1. O ESP32 conecta-se à rede Wi-Fi
2. Estabelece conexão com o broker MQTT
3. Lê o encoder e publica os dados em `sensor/fluxo`
4. Pode receber comandos em `comando/rele`
5. Atualiza os atuadores e LEDs conforme o estado

## Descrição do Funcionamento e Uso

O protótipo desenvolvido tem como objetivo **monitorar o consumo de água em uma residência** e **identificar possíveis vazamentos**. Sua operação segue:  
1. O **ESP32** conecta-se a uma rede Wi‑Fi e a um **broker MQTT** (`broker.hivemq.com`).  
2. Um **encoder rotativo** simula a passagem de água, gerando pulsos contabilizados pelo microcontrolador.  
3. Ao ultrapassar um **limiar de pulsos** (ex.: 50 pulsos/s), o sistema interpreta como vazamento e **aciona um relé**, simulando o fechamento de uma válvula.  
4. Dois **LEDs indicativos** fornecem feedback visual:  
   - LED verde: sistema ativo  
   - LED amarelo: alerta de vazamento  
5. Os dados de pulsos são **publicados via MQTT** no tópico `sensor/fluxo` a cada segundo e podem ser visualizados por quaisquer clientes MQTT.

Para reproduzir, importe o circuito no **Wokwi** (https://wokwi.com/projects/427323032976049153), carregue o código no ESP32 e configure um cliente MQTT para monitorar o tópico `sensor/fluxo`.

---

## Software Desenvolvido e Documentação de Código

O firmware foi escrito em C++ para o Arduino framework e utiliza as bibliotecas `WiFi.h` e `PubSubClient.h`.

### Código-fonte principal (`main.ino`)
```cpp
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Definição dos pinos conectados aos componentes do sistema
#define SENSOR_PIN 4     // Pino do sensor de fluxo (CLK do encoder rotativo)
#define RELAY_PIN 5      // Pino do relé (também acende o LED amarelo de alerta)
#define DT_PIN 15        // Pino DT do encoder (não usado no código atual)
#define LED_NORMAL 2     // Pino do LED verde (indica sistema ligado)

// Tópicos MQTT usados para comunicação com o broker
#define TOPICO_MQTT_ENVIA "esp32/vazamento/status"    // Tópico para envio de mensagens (status do sistema)
#define TOPICO_MQTT_RECEBE "esp32/vazamento/comando"  // Tópico para receber comandos (caso queira controlar remotamente)

// Contador de pulsos do sensor de fluxo (interruptor)
volatile int pulseCount = 0;
bool relayState = false;  // Armazena o estado atual do relé (ligado/desligado)

// Objetos de rede e MQTT
WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);

// Credenciais da rede Wi-Fi e configurações do HiveMQ Cloud
const char* SSID = "hivemq.webclient.1746995554009";
const char* PASSWORD = "6PO5b$Gpu%a0?1AmTw.X";
const char* BROKER_MQTT = "b4e27f4309c240f5ad22ce9e11c131e2.s1.eu.hivemq.cloud";
const int BROKER_PORT = 8883;
const char* CLIENT_ID = "ESP32ClientLeticia";

// Função chamada automaticamente quando o sensor detectar um pulso (simulando o fluxo de água)
void IRAM_ATTR countPulses() {
  pulseCount++;
}

// Função chamada sempre que uma mensagem for recebida no tópico assinado
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String mensagem;
  for (int i = 0; i < length; i++) {
    mensagem += (char)payload[i];
  }
  Serial.print("Mensagem recebida no tópico ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(mensagem);
}

// Reconnect: Tenta se reconectar ao broker caso a conexão MQTT caia
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("Tentando conexão MQTT...");
    if (mqtt.connect(CLIENT_ID)) {
      Serial.println("Conectado ao MQTT!");
      mqtt.subscribe(TOPICO_MQTT_RECEBE);  // Assina o tópico para receber comandos
    } else {
      Serial.print("Falha, rc=");
      Serial.print(mqtt.state());
      Serial.println(" tentando novamente em 5 segundos...");
      delay(5000);
    }
  }
}

// Setup: configura os pinos, conecta ao Wi-Fi e ao broker MQTT
void setup() {
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_NORMAL, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulses, RISING); // Interrupção para contar pulsos

  Serial.begin(115200);

  // Conectando ao Wi-Fi
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");

  secureClient.setInsecure();  // Para testes no Wokwi (ignora verificação de certificado TLS)

  mqtt.setServer(BROKER_MQTT, BROKER_PORT); // Define o broker MQTT e a porta segura
  mqtt.setCallback(mqtt_callback);          // Define a função de callback para mensagens recebidas

  digitalWrite(LED_NORMAL, HIGH);  // LED verde indica sistema ligado

  // Mensagem de inicialização do sistema
  String mensagemInicial = "Sistema iniciado.";
  Serial.println(mensagemInicial);
  mqtt.publish(TOPICO_MQTT_ENVIA, mensagemInicial.c_str());

  // Verifica o fluxo inicial e envia o status
  if (pulseCount <= 50) {
    String statusInicial = "✅ Fluxo normal. Válvula aberta.";
    Serial.println(statusInicial);
    mqtt.publish(TOPICO_MQTT_ENVIA, statusInicial.c_str());
  }
}

// Loop principal do sistema
void loop() {
  if (!mqtt.connected()) {
    reconnect();  // Verifica conexão MQTT
  }
  mqtt.loop();    // Mantém a conexão MQTT ativa

  Serial.print("Pulsos do sensor: ");
  Serial.println(pulseCount);

  // Se os pulsos ultrapassarem o limite (simulando um vazamento)
  if (pulseCount > 50 && !relayState) {
    digitalWrite(RELAY_PIN, HIGH);   // Aciona relé e LED amarelo
    relayState = true;
    String alerta = "⚠️ Vazamento detectado! Fechando válvula.";
    Serial.println(alerta);
    mqtt.publish(TOPICO_MQTT_ENVIA, alerta.c_str());
  } 
  // Se o fluxo estiver normal novamente
  else if (pulseCount <= 50 && relayState) {
    digitalWrite(RELAY_PIN, LOW);    // Desliga alerta
    relayState = false;
    String status = "✅ Fluxo normal. Válvula aberta.";
    Serial.println(status);
    mqtt.publish(TOPICO_MQTT_ENVIA, status.c_str());
  }

  delay(1000); // Aguarda 1 segundo antes de verificar novamente
}
