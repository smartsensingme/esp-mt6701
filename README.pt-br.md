# Driver de Sensor Magnético MT6701 - Componente ESP-IDF

*Leia em outros idiomas: [English](README.md)*

Este diretório contém um componente desacoplado, limpo e altamente otimizado para o sensor de encoder magnético rotativo **MagnTek MT6701** de 14 bits, desenvolvido para o **ESP-IDF v6** visando operações de alta velocidade.

O componente utiliza o driver mestre de I2C moderno do ESP-IDF (`driver/i2c_master.h`) e conta com calibrações de software, estimativa de velocidade, acumulador de voltas completas (multi-turn) e suporte opcional a thread-safety.

---

## 🛠️ Recursos

1.  **Arquitetura Baseada em Software (I2C Read-Only):** Para contornar os requisitos físicos de gravação da EEPROM do chip (que exigem alimentação VDD de 4.5V–5.5V e introduzem delays de travamento de 600ms), este driver processa o Zero Offset e a Direção de Rotação inteiramente por software. O barramento I2C opera como somente leitura em tempo de execução, garantindo máxima performance e segurança.
2.  **Resolução de 14 bits:** Suporta a precisão total de 14 bits de ângulo absoluto (`0` a `16383` passos) do MT6701.
3.  **Habilitação de Thread-Safety (Kconfig):**
    *   **`CONFIG_MT6701_THREAD_SAFE=y`** (Padrão): Protege os registradores I2C e estados internos usando Mutexes do FreeRTOS, garantindo segurança em acessos concorrentes.
    *   **`CONFIG_MT6701_THREAD_SAFE=n`**: Remove toda a lógica de semáforos em tempo de compilação, proporcionando leituras diretas lock-free de custo computacional zero para malhas de controle de alta frequência.
4.  **Acumulador de Voltas (Multi-Turn):** Monitora automaticamente as transições de fim de curso (wrap-around na metade da resolução) para registrar o número total de voltas completas e a posição angular contínua.
5.  **Estimativa de Velocidade Angular:** Calcula a velocidade angular em radianos por segundo (`rad/s`) baseado em timers de hardware de alta precisão (`esp_timer_get_time()`) acoplados a um filtro passa-baixas configurável para atenuar ruídos de quantização de passos.
6.  **Leitura Otimizada em Burst:** As rotinas de leitura buscam ambos os registradores de ângulo (`0x03` e `0x04`) em uma única transação I2C consecutiva de 2 bytes.

---

## 📈 Especificações de Hardware e Limites de Atualização do MT6701

### Performance do Hardware
*   **Resolução:** 14 bits (16.384 posições por volta de 360°, aprox. $0,022^\circ$ por LSB).
*   **Não-Linearidade Integral (INL):** $\pm 0,05^\circ$ (típico sob centralização ideal do ímã e distância de folga recomendada).
*   **Ruído de Transição (Jitter):** $0,01^\circ$ RMS (típico a $25^\circ\text{C}$).
*   **Latência de Propagação:** $< 100\,\mu\text{s}$ de atraso de processamento interno.

### Frequência Máxima de Chamada de `mt6701_update`
A taxa máxima com que você pode chamar `mt6701_update` é limitada pelas velocidades físicas do barramento I2C e pelo custo de processamento do microcontrolador:
*   **A 400 kHz (I2C Fast Mode):** Uma única transação de leitura burst de 2 bytes leva $\approx 73\,\mu\text{s}$. Incluindo o custo de processamento da stack de software do driver, a taxa de atualização máxima teórica é de **$10\text{ kHz}$** (loop de $100\,\mu\text{s}$).
*   **A 1 MHz (I2C Fast Mode Plus):** A transação física leva $\approx 29\,\mu\text{s}$. A taxa de atualização máxima teórica é de **$20\text{ kHz}$** (loop de $50\,\mu\text{s}$).
*   **Taxa de Atualização Recomendada:** Recomenda-se uma amostragem periódica de **$1\text{ kHz}$ a $5\text{ kHz}$**. Isso garante baixo consumo de CPU, deixa banda livre no barramento I2C para outros periféricos e entrega curvas de velocidade altamente dinâmicas e sem lag.

> [!TIP]
> **Limite de Anti-Aliasing (Velocidade Máxima do Motor):**
> Para que o contador de voltas baseado em software (multi-turn) consiga rastrear o sentido do motor de forma correta, o motor não pode rotacionar mais de $180^\circ$ (meia volta) entre duas chamadas consecutivas de `mt6701_update`.
> A relação matemática entre a velocidade máxima do motor $N$ (em RPM) e a frequência mínima de atualização $f_{update}$ necessária é:
> $$f_{update} > \frac{N}{30}$$
> *   Com amostragem a **1 kHz**, o driver suporta velocidades de motor de até **30.000 RPM**.
> *   Com amostragem a **5 kHz**, o driver suporta velocidades de motor de até **150,000 RPM**.

---

## ⚙️ Propriedades de Configuração

Via `menuconfig` (`Component config` -> `MT6701 Driver Configuration`):
*   **`CONFIG_MT6701_THREAD_SAFE`**: Ativa ou desativa a proteção por Mutex.

---

## 🚀 Como Adicionar ao Seu Projeto

Adicione este repositório como um submódulo Git na pasta `components` do seu projeto ESP-IDF:
```bash
git submodule add https://github.com/smartsensingme/esp-mt6701.git components/esp-mt6701
```
Em seguida, atualize o arquivo `CMakeLists.txt` do seu componente principal para declarar o requisito:
```cmake
idf_component_register(SRCS "main.c"
                       REQUIRES esp-mt6701)
```

---

## 📖 Exemplo de Uso da API

Inclua o cabeçalho do driver:
```c
#include "mt6701.h"
```

Inicialize o dispositivo:
```c
// 1. Inicialize a configuração do barramento mestre I2C
i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 8,
    .scl_io_num = 9,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_config, &bus_handle);

// 2. Registre o dispositivo MT6701 no barramento (Endereço 0x06)
i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MT6701_I2C_ADDRESS,
    .scl_speed_hz = 400000, // Suporta até 1MHz Fast Mode Plus
};
i2c_master_dev_handle_t i2c_dev;
i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);

// 3. Inicialize a estrutura de controle do driver
mt6701_dev_t mt6701_device;
ESP_ERROR_CHECK(mt6701_init(&mt6701_device, i2c_dev));

// 4. Ajuste calibrações de software se necessário (Opcional)
mt6701_set_software_direction(&mt6701_device, MT6701_DIR_CW);
```

Leitura contínua em uma Task:
```c
void controle_loop_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Atualiza a leitura física e os estados de voltas/velocidade
        if (mt6701_update(&mt6701_device) == ESP_OK) {
            float deg, velocity;
            int32_t turns;
            
            mt6701_read_angle_degrees(&mt6701_device, &deg);
            mt6701_get_total_turns(&mt6701_device, &turns);
            mt6701_get_velocity(&mt6701_device, &velocity);
            
            printf("Ângulo: %.2f graus | Voltas: %ld | Velocidade: %.2f rad/s\n", deg, turns, velocity);
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1)); // Amostragem a 1 kHz
    }
}
```

---

## 🗄️ Referência da API

### `mt6701_init`
```c
esp_err_t mt6701_init(mt6701_dev_t *dev, i2c_master_dev_handle_t i2c_dev);
```
*   **Descrição:** Inicializa a estrutura de controle do MT6701, configura os valores padrão de software (offset de zero = 0, direção = CW, alpha = 0.20), aloca estaticamente o semáforo de exclusão mútua mutex (se `CONFIG_MT6701_THREAD_SAFE` estiver ativado), e testa a comunicação física com o chip realizando uma leitura rápida I2C.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura `mt6701_dev_t` pré-alocada do dispositivo.
    *   `i2c_dev`: O descritor de dispositivo registrado `i2c_master_dev_handle_t` do driver mestre I2C.
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso (comunicação estabelecida e driver inicializado).
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `i2c_dev` for `NULL`.
    *   `ESP_ERR_NO_MEM` se falhar ao alocar o mutex estático.
    *   Erros do barramento I2C (ex: `ESP_ERR_TIMEOUT`) caso o sensor não responda no endereço `0x06`.

### `mt6701_read_raw_angle`
```c
esp_err_t mt6701_read_raw_angle(mt6701_dev_t *dev, uint16_t *raw_angle);
```
*   **Descrição:** Executa uma transação de leitura em modo burst contínuo de 2 bytes no barramento I2C, iniciando no registrador `0x03` e finalizando no `0x04`, para ler o valor de ângulo absoluto de 14 bits sem filtragem diretamente do processador CORDIC físico.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `raw_angle`: Ponteiro para a variável `uint16_t` onde o valor do ângulo bruto (`0` a `16383`) será armazenado.
*   **Valor de Retorno:**
    *   `ESP_OK` se a leitura for bem-sucedida.
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `raw_angle` for `NULL`.
    *   Códigos de erro de transmissão I2C em caso de falha física.

### `mt6701_update`
```c
esp_err_t mt6701_update(mt6701_dev_t *dev);
```
*   **Descrição:** Lê o ângulo físico do hardware, aplica o offset e sentido de rotação definidos em software, e atualiza o acumulador interno de voltas (detectando cruzamentos de limite/wraps) e o filtro de velocidade angular. Esta função deve ser executada periodicamente em uma tarefa de tempo real a uma frequência constante (ex: 1 kHz).
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` for `NULL`.
    *   Códigos de erro de transmissão I2C em caso de falha física.

### `mt6701_counts_to_degrees`
```c
float mt6701_counts_to_degrees(uint16_t angle_counts);
```
*   **Descrição:** Converte uma contagem angular bruta ou calibrada de 14 bits (`0` a `16383`) para graus, sem acessar o barramento I2C.
*   **Valor de Retorno:** Ângulo entre `0.0f` e aproximadamente `359.978f` graus.

### `mt6701_read_calibrated_angle_counts`
```c
esp_err_t mt6701_read_calibrated_angle_counts(mt6701_dev_t *dev,
                                               uint16_t *angle_counts);
```
*   **Descrição:** Realiza uma nova leitura do sensor, aplica as calibrações de software (offset de zero e direção inversa se configurada) e retorna o ângulo como contagens de 14 bits.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `angle_counts`: Ponteiro para a variável `uint16_t` onde as contagens calibradas (`0` a `16383`) serão gravadas.
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `angle_counts` for `NULL`.
    *   Códigos de erro de transmissão I2C em caso de falha física.

### `mt6701_read_angle_degrees`
```c
esp_err_t mt6701_read_angle_degrees(mt6701_dev_t *dev, float *degrees);
```
*   **Descrição:** Realiza a leitura e calibração de software do sensor e retorna a posição atual mapeada para ponto flutuante em graus.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `degrees`: Ponteiro para a variável `float` que receberá o valor em graus (`0.0f` a `360.0f`).
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `degrees` for `NULL`.
    *   Códigos de erro de transmissão I2C em caso de falha física.

### `mt6701_read_angle_radians`
```c
esp_err_t mt6701_read_angle_radians(mt6701_dev_t *dev, float *radians);
```
*   **Descrição:** Realiza a leitura e calibração de software do sensor e retorna a posição atual mapeada para ponto flutuante em radianos.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `radians`: Ponteiro para a variável `float` que receberá o valor em radianos (`0.0f` a `2*PI`).
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `radians` for `NULL`.
    *   Códigos de erro de transmissão I2C em caso de falha física.

### `mt6701_get_last_angle_degrees`
```c
esp_err_t mt6701_get_last_angle_degrees(mt6701_dev_t *dev, float *degrees);
```
*   **Descrição:** Retorna em graus a última amostra calibrada armazenada por `mt6701_update`, sem realizar uma nova leitura I2C.
*   **Valor de Retorno:** `ESP_OK` em caso de sucesso, `ESP_ERR_INVALID_ARG` para ponteiros nulos ou `ESP_ERR_TIMEOUT` se não for possível obter o mutex.

### `mt6701_get_total_angle_radians`
```c
esp_err_t mt6701_get_total_angle_radians(mt6701_dev_t *dev, float *total_radians);
```
*   **Descrição:** Calcula e retorna a posição contínua acumulada em radianos, somando todas as voltas completas contabilizadas pela função periódica `mt6701_update`.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `total_radians`: Ponteiro para a variável `float` que receberá o valor acumulado em radianos.
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `total_radians` for `NULL`.

### `mt6701_get_total_turns`
```c
esp_err_t mt6701_get_total_turns(mt6701_dev_t *dev, int32_t *turns);
```
*   **Descrição:** Retorna o contador de voltas acumuladas durante o funcionamento do sistema.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `turns`: Ponteiro para a variável `int32_t` que receberá o número total de voltas (pode ser negativo).
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `turns` for `NULL`.

### `mt6701_get_velocity`
```c
esp_err_t mt6701_get_velocity(mt6701_dev_t *dev, float *velocity);
```
*   **Descrição:** Retorna a estimativa de velocidade angular filtrada por software em radianos por segundo (`rad/s`), calculada nas chamadas a `mt6701_update`.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `velocity`: Ponteiro para a variável `float` que receberá a velocidade em `rad/s`.
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` ou `velocity` for `NULL`.

### `mt6701_set_software_zero`
```c
esp_err_t mt6701_set_software_zero(mt6701_dev_t *dev);
```
*   **Descrição:** Executa a leitura da posição mecânica corrente do sensor e define esse valor como o offset de zero (`zero_offset`), zerando os contadores de voltas e velocidade. Todas as leituras subsequentes de ângulo passarão a ser relativas a este ponto.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` for `NULL`.
    *   Códigos de erro de transmissão I2C em caso de falha física.

### `mt6701_set_software_direction`
```c
esp_err_t mt6701_set_software_direction(mt6701_dev_t *dev, mt6701_direction_t dir);
```
*   **Descrição:** Configura o comportamento do sentido de rotação lógico em software, reiniciando os estados de voltas completas e velocidade para evitar saltos ou inconsistências na mudança dinâmica de direção.
*   **Parâmetros:**
    *   `dev`: Ponteiro para a estrutura do dispositivo `mt6701_dev_t`.
    *   `dir`: Sentido lógico desejado (`MT6701_DIR_CW` para rotação padrão direta, `MT6701_DIR_CCW` para invertida).
*   **Valor de Retorno:**
    *   `ESP_OK` em caso de sucesso.
    *   `ESP_ERR_INVALID_ARG` se `dev` for `NULL`.
    *   Códigos de erro de transmissão I2C em caso de falha física.

---
![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Descrição

Este projeto é parte do ecossistema **SmartSensing.me**. Aplicamos fundamentos reais de engenharia de instrumentação e sistemas embarcados de alta performance.

Diferente de conteúdos superficiais da internet, este repositório entrega:
- **Originalidade:** Implementações únicas construídas sobre quase 30 anos de experiência acadêmica.
- **Rigor Técnico:** Uso profissional do framework ESP-IDF e sistemas de tempo real FreeRTOS.
- **Pedagogia:** Código estruturado e ricamente documentado para desenvolvedores que buscam crescimento técnico genuíno.

> "Transformamos sinais do mundo físico em inteligência digital, sem atalhos."

---

## 👤 Sobre o Autor

**José Alexandre de França** *Professor Associado no Departamento de Engenharia Elétrica da UEL*

Engenheiro Eletricista com quase três décadas de atuação no ensino de graduação e pós-graduação. Doutor em Engenharia Elétrica, pesquisador em instrumentação eletrônica e projetista de sistemas embarcados. O SmartSensing.me é meu compromisso em elevar a barra do ensino de tecnologia no Brasil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 Licença

Este projeto é licenciado sob a Licença MIT. Veja o arquivo [LICENSE](LICENSE) para detalhes.
