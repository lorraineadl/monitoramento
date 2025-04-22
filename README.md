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
#include <WiFiClientSecure.h> // Cliente seguro (TLS)
#include <PubSubClient.h>

#define SENSOR_PIN 4
#define RELAY_PIN 5
#define DT_PIN 15
#define LED_NORMAL 2

// Tópicos MQTT (ajuste conforme quiser)
#define TOPICO_MQTT_ENVIA "vazamento/estado"
#define TOPICO_MQTT_RECEBE "vazamento/comando"

// Informações da rede Wi-Fi
const char* SSID = "Wokwi-GUEST";
const char* PASSWORD = "";

// Informações do broker HiveMQ Cloud
const char* BROKER_MQTT = "SEU_HTML";
const int BROKER_PORT = 8883;
const char* MQTT_USER = "SEU_USUARIO";
const char* MQTT_PASSWORD = "SUA_SENHA";

WiFiClientSecure wifiClient;  // Cliente seguro para TLS
PubSubClient MQTT(wifiClient);

volatile int pulseCount = 0;
bool relayState = false;

// Função de interrupção
void IRAM_ATTR countPulses() {
  pulseCount++;
}

// Callback de mensagens MQTT recebidas (se quiser tratar comandos)
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("Mensagem recebida no tópico ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_NORMAL, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulses, RISING);

  // Conectar Wi-Fi
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi conectado!");

  // Configura o cliente seguro
  wifiClient.setInsecure(); // Apenas para testes. Em produção, use certificado CA válido

  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setCallback(mqtt_callback);

  // Conectar ao broker MQTT
  while (!MQTT.connected()) {
    Serial.println("Conectando ao broker MQTT...");
    if (MQTT.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("Conectado ao MQTT!");
      MQTT.subscribe(TOPICO_MQTT_RECEBE);
      MQTT.publish(TOPICO_MQTT_ENVIA, "Sistema iniciado.");
    } else {
      Serial.print("Falha na conexão. Código: ");
      Serial.println(MQTT.state());
      delay(2000);
    }
  }

  digitalWrite(LED_NORMAL, HIGH);  // Sistema ligado
}

void loop() {
  MQTT.loop();

  Serial.print("Pulsos do sensor: ");
  Serial.println(pulseCount);

  if (pulseCount > 50 && !relayState) {
    digitalWrite(RELAY_PIN, HIGH);
    relayState = true;
    Serial.println("⚠️ Vazamento detectado!");
    MQTT.publish(TOPICO_MQTT_ENVIA, "Vazamento detectado");
  } else if (pulseCount <= 50 && relayState) {
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
    Serial.println("✅ Fluxo normal.");
    MQTT.publish(TOPICO_MQTT_ENVIA, "Fluxo normal");
  }

  delay(1000);
}


