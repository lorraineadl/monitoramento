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
#include <WiFi.h>             // Biblioteca principal para gerenciar conexões Wi‑Fi
#include <WiFiAP.h>           // Biblioteca para configurar o ESP32 como ponto de acesso (AP)
#include <WiFiClient.h>       // Biblioteca para criar clientes TCP/IP sobre Wi‑Fi
#include <PubSubClient.h>     // Biblioteca para comunicação MQTT

// Definição dos pinos conectados aos componentes
#define SENSOR_PIN 4          // Pino do sensor de fluxo (CLK do Encoder)
#define RELAY_PIN 5           // Pino do relé e LED amarelo (alerta)
#define DT_PIN 15             // Pino DT do Encoder (não usado no exemplo atual)
#define LED_NORMAL 2          // Pino do LED verde (indica sistema ligado)

// Variáveis globais de estado
volatile int pulseCount = 0;  // Contador de pulsos gerados pelo encoder
bool relayState = false;      // Estado atual do relé (ligado/desligado)

// Protótipo da função de callback para mensagens MQTT recebidas
void mqtt_callback(char* topico, byte* payload, unsigned int tamanho);

// Instância do cliente TCP para MQTT
WiFiClient mqtt_client;
// Instância do cliente MQTT, utilizando o cliente TCP acima
PubSubClient MQTT(mqtt_client);

// Rotina de interrupção: incrementa o contador de pulsos
void IRAM_ATTR countPulses() {
  pulseCount++;
}

void setup() {
  // Configura cada pino do microcontrolador
  pinMode(SENSOR_PIN, INPUT_PULLUP);    // Encoder como entrada com pull‑up interno
  pinMode(DT_PIN, INPUT_PULLUP);        // DT do encoder (poderia ser usado para direção)
  pinMode(RELAY_PIN, OUTPUT);           // Relé como saída
  pinMode(LED_NORMAL, OUTPUT);          // LED verde como saída

  // Ativa a interrupção no pino SENSOR_PIN para cada borda de subida
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulses, RISING);

  Serial.begin(115200);                 // Inicializa monitor serial para debug

  // Inicia a conexão Wi‑Fi na rede simulada do Wokwi
  WiFi.begin("Wokwi-GUEST", "");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");                  // Aguarda até obter IP e imprime pontos
  }
  Serial.println("\nWi‑Fi conectado!");

  // Configura o broker MQTT e a porta de conexão
  MQTT.setServer("broker.hivemq.com", 1883);

  // (Opcional) define a função que será chamada ao receber mensagens
  MQTT.setCallback(mqtt_callback);

  // Acende o LED verde para indicar que o sistema está ativo
  digitalWrite(LED_NORMAL, HIGH);
}

void loop() {
  // Mantém a conexão MQTT viva e processa callbacks
  MQTT.loop();

  // (Opcional) aqui entraria a lógica para conectar/reconectar ao broker:
  // if (!MQTT.connected()) MQTT.connect("ID_do_Cliente");

  Serial.print("Pulsos do sensor: ");
  Serial.println(pulseCount);           // Exibe quantos pulsos já foram contados

  // Lógica de detecção de vazamento baseado em limiar de pulsos
  if (pulseCount > 50 && !relayState) {
    digitalWrite(RELAY_PIN, HIGH);      // Ativa o relé e o LED amarelo
    relayState = true;
    Serial.println("⚠️ Vazamento detectado! Fechando válvula...");
  }
  else if (pulseCount <= 50 && relayState) {
    digitalWrite(RELAY_PIN, LOW);       // Desativa o relé e o LED amarelo
    relayState = false;
    Serial.println("✅ Fluxo normal. Válvula aberta.");
  }

  delay(1000);                          // Espera 1 segundo antes de nova leitura
}

// Função de callback para mensagens MQTT recebidas (ainda não implementada)
void mqtt_callback(char* topico, byte* payload, unsigned int tamanho) {
  // Aqui você pode decodificar 'payload' e agir conforme o tópico
  // Exemplo: controlar o relé através de mensagens em "comando/rele"
}

