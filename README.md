# AutoHome MQTT

# AutoHome MQTT

**AutoHome MQTT** é uma plataforma de automação residencial baseada no microcontrolador **Raspberry Pi Pico W**, utilizando o protocolo **MQTT** para comunicação. O sistema oferece:

- Monitoramento de **temperatura** e **umidade** (sensor DHT11), com publicação periódica via MQTT.  
- Controle remoto de **iluminação** (sala, cozinha e quarto), acionado por tópicos MQTT.  
- Abertura/fechamento de **portão** via servo motor, comandado por MQTT.  
- Emissão de **alertas sonoros** (buzzer) via tópico MQTT.  
- Feedback local em **display OLED** (SSD1306), exibindo IP do dispositivo e leituras do DHT11.  
- Indicação de estado em **matriz de LEDs WS2812** (5×5).  
- Botão físico para entrar em modo bootloader USB e atualização de firmware.  

---

## Funcionalidade Geral

1. **Inicialização**  
  - Conecta o Pico W à rede Wi-Fi (SSID e senha definidos nas macros).  
  - Configura I²C para o OLED, PIO para os LEDs WS2812, PWM para o servo e buzzer, e interrupção para o botão físico.  
  - Inicializa o cliente MQTT e resolve o DNS para `MQTT_SERVER`.  
  - Ao conectar-se ao broker, publica “online” e se inscreve nos tópicos de controle.

2. **Leituras do Sensor DHT11**  
  - A cada `TEMP_WORKER_TIME_S` (10 s), um timer acionado dispara a leitura do sensor DHT11.  
  - Publica a temperatura em `/temperatura` e a umidade em `/umidade` (QoS 1, retain).

3. **Comunicação MQTT**  
  - **Tópicos de assinatura**:  
    - `/sala` – comando “On”/“Off” ou “1”/“0” para ligar/desligar a luz da sala.  
    - `/cozinha` – mesma lógica para a luz da cozinha.  
    - `/quarto` – mesma lógica para a luz do quarto.  
    - `/portao` – comando “Open”/“Close” ou “1”/“0” para abrir/fechar o portão.  
    - `/alerta` – comando para acionar o buzzer.  
    - `/exit` – solicita a desconexão do cliente MQTT e encerra o loop principal.  
  - **Publicações**:  
    - `/online` (will topic) – sinal “1” enquanto conectado; “0” ao desconectar.  
    - `/temperatura` e `/umidade` – valores numéricos em string, atualizados a cada 10 s.  
    - `/led/state` – estado interno do LED integrado ao módulo CYW43 (On/Off).

4. **Controle de Dispositivos**  
  - Ao receber mensagens dos tópicos, atualiza a variável `things` e aciona:  
    - **LEDs WS2812**: LEDs em posições fixas (5×5) para sala (índice 14), cozinha (índice 12) e quarto (índice 10) que mudam de cor (vermelho, verde e azul) ao serem ativados.  
    - **Servo**: move para o ângulo certo com pulsos (2400 µs para abertura e 500 µs para fechamento) para o portão.  
    - **Buzzer**: toca um alerta (392 Hz por 1 s) quando `/alerta` é publicado.

5. **Exibição no OLED SSD1306**  
  - Mostra o IP atual do Pico W na linha superior e, abaixo, as leituras de temperatura e umidade.  
  - Atualiza sempre que ocorre uma nova leitura ou ao conectar/desconectar do broker.

6. **Modo Bootloader USB**  
  - O botão físico conectado ao GP6 ativa `reset_usb_boot(0, 0)` (após debounce de 400 ms), colocando o Pico em modo UF2 para atualização de firmware.

---

## Materiais Necessários

- **Raspberry Pi Pico W**  
- Sensor **DHT11** (para medir temperatura e umidade)  
- Display **OLED SSD1306** (I²C, 128×64)  
- **Matriz de LEDs WS2812** (25 LEDs, 5×5)  
- **Servo motor** (ex.: SG90)  
- **Buzzer** (ativo ou passivo)  
- Botão momentâneo (GPIO com pull-up interno para bootloader)  
- Cabos jumper e protoboard  
- Fonte 5 V (via USB ou fonte externa, conforme o consumo do servo e dos LEDs)

---

## Pré-requisitos de Software

1. **Pico SDK** devidamente configurado (CMake ≥ 3.13, toolchain ARM GCC).  
2. **pico-extras** para drivers OLED e PIO (incluir como submódulo ou ajustar o `CMakeLists.txt`).  
3. **LWIP** com suporte a MQTT e TLS (para conexões seguras, se necessário).  
4. Ferramenta para upload UF2 (usa o modo bootloader ou `picotool`).

---

## Conexões de Hardware

| Componente             | GPIO do Pico W            | Observações                                        |
|------------------------|---------------------------|----------------------------------------------------|
| **SSD1306 OLED**       | SDA → GP14 / SCL → GP15   | I²C1 a 400 kHz, endereço I²C 0x3C                  |
| **DHT11**              | GP9                       | Linha de dados configurada via software            |
| **WS2812 (LED)**       | GP7                       | Usado com PIO0 (programa `ws2812.pio`), 800 kHz      |
| **Servo motor**        | GP8                       | PWM configurado para 50 Hz (wrap 20000, clkdiv 125)  |
| **Buzzer**             | GP21                      | PWM para gerar o tom (configuração de frequência/duração) |
| **Botão Bootloader**   | GP6                       | Pull-up interno, ativado em borda de descida          |
| **I²C (para OLED)**    | GP14 (SDA) / GP15 (SCL)   | Configure os pull-ups internos via software          |

> **Observação:**  
> - Modifique `DHT_PIN` (GP9) e `BUZZER_PIN` (GP21) conforme necessário, mantendo-os atualizados no `main.c`.  
> - Os índices fixos para os LEDs WS2812 (14, 12, 10) são definidos em `enum THINGS_MATRIX_5X5_POSITION`.

---

## Compilação e Deploy

1. **Clonar o Repositório**  
  ```bash
  git clone https://github.com/seu-usuario/AutoHome-MQTT.git
  cd AutoHome-MQTT
  ```
  Configure a variável de ambiente do Pico SDK:
  ```bash
  export PICO_SDK_PATH=/caminho/para/pico-sdk
  ```

2. **Criar e Acessar a Pasta de Build**  
  ```bash
  mkdir build && cd build
  ```

3. **Gerar os Arquivos de Build com CMake**  
  ```bash
  cmake ..
  ```

4. **Compilar o Firmware**  
  ```bash
  make -j4
  ```

5. **Atualizar o Firmware no Pico W**  
  - Pressione e segure o botão físico (GP6) para entrar em modo bootloader USB.  
  - Conecte o Pico W ao computador (ele será reconhecido como “RPI-RP2”).  
  - Copie o arquivo `main.uf2` (ou `firmware.uf2`) para a unidade:
    ```bash
    cp main.uf2 /Volumes/RPI-RP2/
    ```

6. **Configuração de Wi-Fi e MQTT**  
  No início do `main.c`, ajuste as macros:
  ```c
  #define WIFI_SSID      "SEU_SSID"
  #define WIFI_PASSWORD  "SUA_SENHA"

  #define MQTT_SERVER    "IP_OU_HOST_DO_BROKER"
  #define MQTT_USERNAME  "USUARIO_MQTT"      // ou deixe vazio se não usar autenticação
  #define MQTT_PASSWORD  "SENHA_MQTT"        // ou deixe vazio se não usar autenticação
  ```
  - Use o IP ou hostname do broker no `MQTT_SERVER` (ex.: 192.168.1.107).  
  - Caso o broker exija autenticação, defina `MQTT_USERNAME` e `MQTT_PASSWORD`.  
  - Para conexões TLS, ajuste as opções de compilação do LWIP e inclua o certificado (defina `MQTT_CERT_INC` apontando para o arquivo .pem).

---

## Uso

1. Ligue o dispositivo e aguarde a conexão Wi-Fi.  
2. O Pico W fará a resolução de DNS para o `MQTT_SERVER` e, se conectado, publicará “1” em `/online` e se inscreverá nos tópicos de controle.  
3. O display OLED mostrará o IP do dispositivo e as leituras de temperatura e umidade.

### Controle via MQTT

Utilize um cliente MQTT (como mosquitto_pub, MQTT Explorer ou Node-RED) para publicar nos seguintes tópicos:
- `/sala` – “On” ou “Off”
- `/cozinha` – “On” ou “Off”
- `/quarto` – “On” ou “Off”
- `/portao` – “Open” ou “Close”
- `/alerta` – aciona o alerta sonoro (por exemplo, “1”)

Exemplos:
```bash
mosquitto_pub -h 192.168.1.107 -t /sala   -m "On"
mosquitto_pub -h 192.168.1.107 -t /portao -m "Open"
mosquitto_pub -h 192.168.1.107 -t /alerta -m "1"
```

Sempre que uma mensagem é recebida:
- Os LEDs WS2812 mudam de estado na posição correspondente.
- O servo se move para abrir/fechar o portão.
- O buzzer toca em caso de alerta.

### Publicação de Telemetria

A cada 10 s, os dados do DHT11 são publicados:
- Temperatura em `/temperatura` (formato "xx.xx")
- Umidade em `/umidade` (formato "yy.yy")

Para monitorar:
```bash
mosquitto_sub -h 192.168.1.107 -t /temperatura -v
mosquitto_sub -h 192.168.1.107 -t /umidade -v
```

### Desligamento Controlado

Para desconectar o cliente MQTT e encerrar o programa, publique:
```bash
mosquitto_pub -h 192.168.1.107 -t /exit -m "any"
```

### Atualização de Firmware

Pressione o botão físico (GP6) para entrar no modo bootloader USB e copie o arquivo UF2 gerado após a compilação.

