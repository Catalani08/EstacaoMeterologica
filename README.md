# ⛈️ Projeto Integrador: Estação Meteorológica Profissional com IoT

![Status do Projeto](https://img.shields.io/badge/Status-Em%20Desenvolvimento-yellow)
![Licença](https://img.shields.io/badge/License-MIT-blue.svg)

> **Breve Descrição:** Desenvolvimento de uma **Estação Meteorológica IoT** de nível profissional e baixo custo. O sistema utiliza um **ESP32** para coletar dados ambientais (T, U, P, Luminosidade, Vento e Chuva) e enviá-los de forma eficiente e segura via protocolo **MQTT** para o **AWS IoT Core**. Os dados são persistidos no **InfluxDB** (Banco de Dados de Série Temporal) para análise avançada e visualização em tempo real.

---

## 💻 Tecnologias Utilizadas

Esta solução abrange as áreas de Hardware, Firmware e Cloud Computing.

| Área | Tecnologia | Descrição |
| :--- | :--- | :--- |
| **Microcontrolador** | `ESP32 DevKitC` | Responsável pela leitura dos sensores e conexão Wi-Fi/MQTT. |
| **IDE** | `VS Code` + `ESP-IDF` | Ambiente de desenvolvimento utilizado para programação em C/C++. |
| **Protocolo** | `MQTT` | Protocolo leve e eficiente para comunicação M2M (Machine-to-Machine). |
| **Nuvem/Broker** | `AWS IoT Core` | Serviço gerenciado da Amazon para atuar como Broker MQTT seguro. |
| **Banco de Dados** | `InfluxDB` | Banco de dados otimizado para séries temporais (dados de sensores). |
| **Linguagem** | `C` | Linguagem utilizada para o desenvolvimento do Firmware do ESP32. |

### 🧩 Sensores Integrados

| Variável | Sensor | Tipo de Leitura |
| :--- | :--- | :--- |
| **Temperatura, Umidade e Pressão** | `BME280` | Digital (I2C) |
| **Velocidade do Vento** | `Encoder` | Digital (Contagem por Interrupção) |
| **Direção do Vento** | `Sensor De Ângulo Hall Efeito Potenciômetro Transdutor` | Analógica (Divisor de Tensão) |
| **Intensidade da Chuva**| `Sensor de Chuva` | Analógica (Nível de Precipitação) |
| **Luminosidade** | `LDR` | Analógica |

---

## 💡 Funcionalidades e Objetivos

* ✅ **Monitoramento Completo:** Aquisição simultânea de 7 variáveis ambientais (T, U, P, Altitude, Luz, Vento (Velocidade e Direção) e Chuva).
* ✅ **Comunicação Segura:** Publicação de *payloads* (JSON) no Broker MQTT da AWS (IoT Core) usando certificados e TLS.
* ✅ **Persistência de Dados:** Configuração de regras na AWS IoT Core para encaminhar os dados para um *Bucket* e *Measurement* específico no InfluxDB.
* ✅ **Dashboard de Análise:** Criação de visualizações em tempo real  para gráficos e alertas.
* ✅ **Eficiência Energética:** O código em C no ESP32 foca em desempenho e no uso de modos *Deep Sleep* + controle da alimentação dos sensores a través de MOSFET para uma maior eficiência energética.

---

## 🛠️ Configuração de Desenvolvimento e Infraestrutura

Esta seção detalha como configurar e rodar o projeto.

#### 1. Pinagem do Hardware (ESP32)

| Componente | Comunicação | Pino ESP32 | Notas de Implementação |
| :--- | :--- | :--- | :--- |
| **BME280 (SDA)** | I2C | **GPIO 21** | |
| **BME280 (SCL)** | I2C | **GPIO 22** | I2C Padrão do ESP32 |
| **Sensor Chuva** | Analógica | **GPIO 34** | Mede a intensidade da precipitação. |
| **LDR** | Analógica | **GPIO 33** | |
| **Biruta (Transcoder)** | Analógica | **GPIO 32** | Converte a tensão de saída em graus (0-360º). |
| **Encoder (Saída A)** | Digital (Interrupção) | **GPIO 25** | Captura a frequência para cálculo de velocidade. |
| **Encoder (Saída B)** | Digital (Interrupção) | **GPIO 26** | Usado para lógica de rotação e interrupção. |

#### 2. Pré-requisitos de Software

1.  **IDE:** Instalar **VS Code** e a extensão **ESP-IDF**, assim como **C/C++ Extension Pack**.
2.  **Bibliotecas:** As dependências já estão inclusas no código.
3.  **Configuração de Credenciais:**
    * Configure as credenciais de Wi-Fi e os **Certificados/Endpoints da AWS IoT Core** e as **Chaves/Bucket do InfluxDB** no código-fonte.

