# `esp-mt6701`

Documentação em inglês: [README.md](README.md).

`esp-mt6701` é um componente ESP-IDF para leitura, por I2C, do ângulo absoluto
de 14 bits do MagnTek MT6701. Ele também oferece zero e direção definidos por
software, contagem de múltiplas voltas e uma estimativa filtrada da velocidade.

O driver usa a API mestre moderna do ESP-IDF, disponível em
`driver/i2c_master.h`. A aplicação é responsável pelo barramento e pelo
dispositivo I2C e entrega o handle já registrado para `mt6701_init()`.

## Responsabilidades do componente

O componente fornece:

- leitura burst de dois bytes para obter o ângulo nativo de 14 bits;
- zero e direção por software, sem gravar a EEPROM do MT6701;
- ângulo armazenado em cache, em contagens ou graus;
- novas leituras de ângulo em contagens, graus ou radianos;
- contagem assinada de múltiplas voltas;
- velocidade angular filtrada em radianos por segundo;
- mutex opcional por instância, sem alocação dinâmica.

Ele não configura os pinos ou o clock do I2C, não verifica o alinhamento do ímã,
não lê saídas de diagnóstico, não persiste a calibração de software e não
aplica uma LUT de linearização. Essas responsabilidades pertencem à aplicação
ou a outro componente.

## Fluxo dos dados

O caminho periódico normal é:

```text
registradores 0x03/0x04 do MT6701
        |
        v
mt6701_update()
        |
        +-- zero e direção por software
        +-- deslocamento cíclico e contador de voltas
        +-- velocidade instantânea e filtro passa-baixas
        |
        v
estado em cache
  |-- mt6701_get_last_angle_counts()
  |-- mt6701_get_last_angle_degrees()
  |-- mt6701_get_total_turns()
  |-- mt6701_get_total_angle_radians()
  `-- mt6701_get_velocity()
```

Chame `mt6701_update()` uma vez em cada iteração de controle e, em seguida, use
os getters do cache. Assim, ângulo, voltas e velocidade ficam associados à
mesma amostra do sensor.

As funções cujo nome começa com `mt6701_read_` realizam uma nova leitura I2C.
Por exemplo, chamar `mt6701_read_angle_degrees()` logo após `mt6701_update()`
provoca uma segunda transação e pode retornar um ângulo um pouco mais recente
que a velocidade e o contador de voltas armazenados.

## Ligação sugerida ao ESP32-S3

Esta é a ligação usada pelo projeto de referência atual:

| ESP32-S3 | MT6701 | Função |
|---|---|---|
| `3V3` | `VDD` | Alimentação do sensor e da lógica I2C |
| `GND` | `GND` / `VSS` | Referência comum |
| `GPIO8` | `SDA` | Dados I2C |
| `GPIO9` | `SCL` | Clock I2C |

GPIO8 e GPIO9 são escolhas da aplicação, não exigências do driver. O endereço
do MT6701 é fixo e está definido por `MT6701_I2C_ADDRESS` (`0x06`). A aplicação
de referência usa clock de 1 MHz. Nessa frequência, mantenha as ligações curtas
e use pull-ups externos adequados para 3,3 V; os pull-ups internos do ESP32-S3
são fracos e não devem ser a primeira escolha para um barramento robusto.

As saídas analógica, ABI e UVW do MT6701 não são usadas por este driver I2C.

## Adicionando o componente

Coloque o repositório dentro da pasta `components` da aplicação e declare a
dependência no componente consumidor:

```cmake
idf_component_register(
    SRCS "meu_controle.c"
    INCLUDE_DIRS "."
    REQUIRES esp-mt6701
)
```

Inclua o cabeçalho público:

```c
#include "mt6701.h"
```

## Kconfig

O menu `Component config -> MT6701 Driver Configuration` contém:

| Opção | Significado |
|---|---|
| `CONFIG_MT6701_THREAD_SAFE=y` | Cria um mutex estático por instância e protege as transações I2C e o estado compartilhado. |
| `CONFIG_MT6701_THREAD_SAFE=n` | Remove os mutexes na compilação para reduzir o custo no caminho de tempo real com um único proprietário. |

Quando a proteção estiver desabilitada, a mesma instância `mt6701_dev_t` não
deve ser acessada simultaneamente por tarefas diferentes ou por interrupções.
As APIs de leitura não são adequadas para ISR porque usam o driver I2C
bloqueante.

## Exemplo completo de inicialização

```c
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "mt6701.h"

static i2c_master_bus_handle_t barramento_sensor;
static i2c_master_dev_handle_t dispositivo_i2c_sensor;
static mt6701_dev_t sensor;

esp_err_t inicializar_sensor(void)
{
    const i2c_master_bus_config_t configuracao_barramento = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(&configuracao_barramento, &barramento_sensor),
        "SENSOR", "criar barramento I2C");

    const i2c_device_config_t configuracao_dispositivo = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MT6701_I2C_ADDRESS,
        .scl_speed_hz = 1000000,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(barramento_sensor, &configuracao_dispositivo,
                                  &dispositivo_i2c_sensor),
        "SENSOR", "registrar MT6701");

    ESP_RETURN_ON_ERROR(mt6701_init(&sensor, dispositivo_i2c_sensor),
                        "SENSOR", "inicializar MT6701");

    return mt6701_set_software_direction(&sensor, MT6701_DIR_CW);
}
```

A aplicação continua responsável por remover o dispositivo I2C e destruir o
barramento ao final de suas vidas úteis. O driver não possui `deinit()` porque
não aloca recursos dinâmicos próprios.

## Exemplo de amostragem periódica

```c
esp_err_t amostrar_controle(float *angulo_deg, float *velocidade_rad_s,
                            int32_t *voltas)
{
    ESP_RETURN_ON_ERROR(mt6701_update(&sensor), "SENSOR", "amostrar MT6701");

    ESP_RETURN_ON_ERROR(mt6701_get_last_angle_degrees(&sensor, angulo_deg),
                        "SENSOR", "obter ângulo do cache");
    ESP_RETURN_ON_ERROR(mt6701_get_velocity(&sensor, velocidade_rad_s),
                        "SENSOR", "obter velocidade do cache");
    return mt6701_get_total_turns(&sensor, voltas);
}
```

Esses getters não acessam o I2C. Quando a aplicação precisa apenas de um ângulo
independente e não usa rastreamento, pode chamar uma função de nova leitura,
como `mt6701_read_angle_degrees()`.

## Contagens nativas e `esp_angle_lut`

`mt6701_get_last_angle_counts()` retorna o ângulo de 14 bits armazenado depois
da aplicação do zero e da direção por software. Essa é a interface adequada
para uma linearização no domínio de contagens:

```c
uint16_t contagens;
ESP_ERROR_CHECK(mt6701_update(&sensor));
ESP_ERROR_CHECK(mt6701_get_last_angle_counts(&sensor, &contagens));

uint16_t contagens_corrigidas = esp_angle_lut_apply(contagens);
float graus_corrigidos = mt6701_counts_to_degrees(contagens_corrigidas);
```

Esse exemplo direto pressupõe
`ESP_ANGLE_LUT_FULL_SCALE_COUNTS == 16384`. Se a LUT estiver configurada para
outra resolução, a aplicação deve converter entre as escalas antes e depois da
correção.

A correção deve entrar depois do zero/direção do MT6701 e antes do unwrap, da
estimativa de velocidade ou do controle. A velocidade interna do driver MT6701
é calculada a partir do ângulo não corrigido pela LUT. Se a aplicação precisar
da velocidade corrigida, deve estimá-la externamente a partir do ângulo
corrigido.

## Estimador de velocidade

Para duas amostras calibradas consecutivas, `mt6701_update()` calcula:

```text
delta = menor diferença cíclica em contagens
velocidade_instantânea = delta * (2*pi/16384) / dt
velocidade_filtrada = alpha * velocidade_instantânea
                    + (1 - alpha) * velocidade_filtrada_anterior
```

`velocity_filter_alpha` é inicializado com `0.20f`:

- próximo de 1: acompanha mais rapidamente, mas transmite mais ruído de
  quantização e de temporização;
- próximo de 0: suaviza mais, mas adiciona atraso;
- igual a 0: conserva a velocidade filtrada anterior;
- igual a 1: desabilita a suavização e usa a velocidade instantânea.

A API atual expõe esse coeficiente como campo de `mt6701_dev_t`. Para alterá-lo,
faça isso depois de `mt6701_init()` e antes de iniciar acessos concorrentes. O
driver não limita nem valida atribuições feitas diretamente pela aplicação.

## Limite de amostragem para múltiplas voltas

O algoritmo pressupõe que o eixo se mova menos de meia volta entre duas
chamadas consecutivas bem-sucedidas de `mt6701_update()`. Caso contrário, não é
possível distinguir o movimento real do deslocamento mais curto no sentido
oposto.

Para velocidade máxima `N` em RPM, use uma frequência que satisfaça:

```text
f_update > N / 30
```

Esse é apenas o limite de aliasing, não uma margem recomendada. Jitter de
escalonamento, erros I2C e aceleração exigem margem adicional.

## Zero e direção por software

`mt6701_set_software_zero()` lê o ângulo bruto atual e o armazena como nova
origem. Também zera as voltas, a velocidade e reinicia a referência temporal.

`mt6701_set_software_direction()` escolhe entre preservar ou inverter as
contagens nativas. O zero existente é preservado, mas voltas e velocidade são
zeradas e uma nova amostra estabelece a referência, evitando interpretar a
mudança de coordenadas como movimento do eixo.

As duas configurações existem apenas na RAM e retornam a zero/CW após uma nova
inicialização ou reinicialização do sistema. O componente deliberadamente não
programa a EEPROM do MT6701.

## Resumo da API e relações de chamada

| Função | Nova leitura I2C? | Chamada internamente por | Resultado principal |
|---|:---:|---|---|
| `mt6701_init` | Sim | Ninguém | Inicialização e teste de comunicação |
| `mt6701_init_from_raw_angle` | Não | Ninguém | Inicialização com amostra adquirida pelo chamador |
| `mt6701_read_raw_angle` | Sim | `init`, `update`, leituras calibradas, zero e direção | Contagem nativa |
| `mt6701_counts_to_degrees` | Não | Funções de graus nova/cache | Conversão pura |
| `mt6701_update` | Sim | Ninguém | Atualização de todo o estado |
| `mt6701_update_from_raw_angle` | Não | `mt6701_update` | Atualização com amostra adquirida pelo chamador |
| `mt6701_read_calibrated_angle_counts` | Sim | Leituras novas em graus/radianos | Contagem processada nova |
| `mt6701_read_angle_degrees` | Sim | Ninguém | Ângulo novo em graus |
| `mt6701_read_angle_radians` | Sim | Ninguém | Ângulo novo em radianos |
| `mt6701_get_last_angle_counts` | Não | Ninguém | Contagem processada do cache |
| `mt6701_get_last_angle_degrees` | Não | Ninguém | Ângulo em graus do cache |
| `mt6701_get_total_angle_radians` | Não | Ninguém | Ângulo contínuo do cache |
| `mt6701_get_total_turns` | Não | Ninguém | Voltas completas do cache |
| `mt6701_get_velocity` | Não | Ninguém | Velocidade filtrada do cache |
| `mt6701_set_software_zero` | Sim | Ninguém | Nova origem e reinício do rastreamento |
| `mt6701_set_software_direction` | Sim | Ninguém | Nova direção e reinício do rastreamento |

Parâmetros, códigos de retorno, funções chamadas e relações entre chamadores
estão documentados detalhadamente em
[`include/mt6701.h`](include/mt6701.h). A implementação em
[`mt6701.c`](mt6701.c) está dividida em blocos funcionais comentados.

Uma aplicação que controla uma aquisição assíncrona pode usar
`mt6701_init_from_raw_angle()` e `mt6701_update_from_raw_angle()`. O chamador
mantém o buffer de recepção válido até o fim da transferência e fornece o
timestamp associado à amostra. O componente então executa a mesma calibração
por software, desempacotamento angular, contagem de voltas e atualização da
velocidade sem iniciar outra transação I2C.

## Limitações práticas

- A resolução é fixa em 14 bits: 16384 contagens por volta.
- O estado de múltiplas voltas existe apenas na RAM.
- Zero e direção por software não são gravados no sensor.
- Erros de leitura I2C deixam o estado de rastreamento armazenado inalterado.
- O driver não detecta campo magnético fraco, air gap excessivo ou
  excentricidade do ímã.
- A velocidade usa um filtro de primeira ordem, e não um filtro de Kalman.
- A linearização angular permanece deliberadamente no componente independente
  `esp_angle_lut`.
