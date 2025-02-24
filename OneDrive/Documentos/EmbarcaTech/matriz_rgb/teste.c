#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Biblioteca gerada pelo arquivo .pio durante compilação.
#include "ws2818b.pio.h"

// Definição do número de LEDs e pino.
#define LED_COUNT 25
#define LED_PIN 7

// Definição de pinos de entrada para os botões A e B
#define BUTTON_A_PIN 5   // Botão A
#define BUTTON_B_PIN 6   // Botão B

// Definição do pino do Joystick
#define JOYSTICK_X_PIN 26
#define JOYSTICK_Y_PIN 27
#define JOYSTICK_THRESHOLD 2048  // Valor médio do ADC (12 bits = 4096/2)

// Definição dos pinos I2C para o display OLED
#define I2C_SDA 14
#define I2C_SCL 15

// Definição dos pinos do BUZZER
#define BUZZER_A_PIN 21
#define BUZZER_B_PIN 10

// Definição de pixel GRB
struct pixel_t {
  uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

// Declaração do buffer de pixels que formam a matriz.
npLED_t leds[LED_COUNT];

// Buffer global para o display OLED
uint8_t display_buffer[ssd1306_buffer_length];
struct render_area display_area;

// Variáveis para uso da máquina PIO.
PIO np_pio;
uint sm;

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {

  // Cria programa PIO.
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  // Toma posse de uma máquina PIO.
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
  }

  // Inicia programa na máquina PIO obtida.
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Limpa buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}

/**
 * Atribui uma cor RGB a um LED com intensidade muito reduzida.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  leds[index].R = r / 70;  // Reduz a intensidade do vermelho para 1/8
  leds[index].G = g / 70; // Reduz a intensidade do verde para 1/16
  leds[index].B = b / 70; // Reduz a intensidade do azul para 1/16
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {
  // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

/**
 * Inicializa os botões.
 */
void init_buttons() {
  gpio_init(BUTTON_A_PIN);  // Inicializa o pino do botão A
  gpio_set_dir(BUTTON_A_PIN, GPIO_IN);  // Configura como entrada
  gpio_pull_up(BUTTON_A_PIN);  // Habilita o pull-up para o botão A

  gpio_init(BUTTON_B_PIN);  // Inicializa o pino do botão B
  gpio_set_dir(BUTTON_B_PIN, GPIO_IN);  // Configura como entrada
  gpio_pull_up(BUTTON_B_PIN);  // Habilita o pull-up para o botão B
}

/**
 * Inicializa o ADC para o joystick
 */
void init_joystick() {
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);
}

/**
 * Verifica se o joystick está pressionado (posição central)
 */
bool is_joystick_pressed() {
    adc_select_input(0);  // Y axis
    uint y_value = adc_read();
    adc_select_input(1);  // X axis
    uint x_value = adc_read();
    
    // Verifica se o joystick está próximo ao centro (pressionado)
    return (abs(x_value - JOYSTICK_THRESHOLD) < 500 && 
            abs(y_value - JOYSTICK_THRESHOLD) < 500);
}

/**
 * Função para verificar se o botão A foi pressionado.
 */
bool is_button_a_pressed() {
  return !gpio_get(BUTTON_A_PIN);  // Botão A pressionado se o valor for 0
}

/**
 * Função para verificar se o botão B foi pressionado.
 */
bool is_button_b_pressed() {
  return !gpio_get(BUTTON_B_PIN);  // Botão B pressionado se o valor for 0
}

/**
 * Inicializa o display OLED
 */
void init_oled() {
    // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o display OLED
    ssd1306_init();

    // Configura a área de renderização
    display_area.start_column = 0;
    display_area.end_column = ssd1306_width - 1;
    display_area.start_page = 0;
    display_area.end_page = ssd1306_n_pages - 1;

    calculate_render_area_buffer_length(&display_area);

    // Limpa o display
    memset(display_buffer, 0, ssd1306_buffer_length);
    render_on_display(display_buffer, &display_area);
}

/**
 * Inicializa os pinos do buzzer
 */
void init_buzzer() {
    // Configura o pino A do buzzer
    gpio_set_function(BUZZER_A_PIN, GPIO_FUNC_PWM);
    uint slice_num_a = pwm_gpio_to_slice_num(BUZZER_A_PIN);
    pwm_set_wrap(slice_num_a, 62500);  // 1kHz @ 125MHz clock
    pwm_set_enabled(slice_num_a, true);

    // Configura o pino B do buzzer
    gpio_set_function(BUZZER_B_PIN, GPIO_FUNC_PWM);
    uint slice_num_b = pwm_gpio_to_slice_num(BUZZER_B_PIN);
    pwm_set_wrap(slice_num_b, 62500);
    pwm_set_enabled(slice_num_b, true);
}

/**
 * Toca um som de erro no buzzer
 */
void play_error_sound() {
    uint slice_num_a = pwm_gpio_to_slice_num(BUZZER_A_PIN);
    uint slice_num_b = pwm_gpio_to_slice_num(BUZZER_B_PIN);
    
    // Som de erro: alterna entre duas frequências
    for(int i = 0; i < 3; i++) {
        // Frequência mais alta
        pwm_set_gpio_level(BUZZER_A_PIN, 31250);  // 50% duty cycle
        pwm_set_gpio_level(BUZZER_B_PIN, 31250);
        sleep_ms(100);
        
        // Frequência mais baixa
        pwm_set_gpio_level(BUZZER_A_PIN, 15625);  // 25% duty cycle
        pwm_set_gpio_level(BUZZER_B_PIN, 15625);
        sleep_ms(100);
    }
    
    // Desliga o buzzer
    pwm_set_gpio_level(BUZZER_A_PIN, 0);
    pwm_set_gpio_level(BUZZER_B_PIN, 0);
}

/**
 * Exibe uma mensagem no display OLED com quebra de linha
 */
void display_message(const char* message) {
    // Limpa o buffer
    memset(display_buffer, 0, ssd1306_buffer_length);
    
    // Cria uma cópia não-const da string
    char msg_buffer[64];  // Buffer maior para acomodar as quebras de linha
    strncpy(msg_buffer, message, sizeof(msg_buffer) - 1);
    msg_buffer[sizeof(msg_buffer) - 1] = '\0';
    
    // Quebra a mensagem em linhas
    char *line = strtok(msg_buffer, " ");
    int y_pos = 8;  // Posição vertical inicial
    char current_line[32] = "";  // Buffer para a linha atual
    
    while (line != NULL) {
        // Se a linha atual mais a próxima palavra não exceder o limite
        if (strlen(current_line) + strlen(line) + 1 < 16) {
            if (strlen(current_line) > 0) {
                strcat(current_line, " ");
            }
            strcat(current_line, line);
        } else {
            // Desenha a linha atual e começa uma nova
            if (strlen(current_line) > 0) {
                ssd1306_draw_string(display_buffer, 0, y_pos, current_line);
                y_pos += 12;  // Espaçamento entre linhas
            }
            strcpy(current_line, line);
        }
        line = strtok(NULL, " ");
    }
    
    // Desenha a última linha
    if (strlen(current_line) > 0) {
        ssd1306_draw_string(display_buffer, 0, y_pos, current_line);
    }
    
    // Atualiza o display
    render_on_display(display_buffer, &display_area);
}

int main() {

  // Inicializa entradas e saídas.
  stdio_init_all();

  // Inicializa matriz de LEDs NeoPixel.
  npInit(LED_PIN);

  // Inicializa os botões
  init_buttons();

  // Inicializa o joystick
  init_joystick();

  // Inicializa o display OLED
  init_oled();

  // Inicializa os pinos do buzzer
  init_buzzer();

  // Inicializa o gerador de números aleatórios
  srand(time(NULL));

  // Estado anterior do joystick
  bool last_joystick_state = false;

  // Loop principal
  while (true) {
    if (is_button_a_pressed()) {
      // Se o botão A for pressionado, acende a matriz na cor verde.
      npClear();
      for (int i = 0; i < LED_COUNT; ++i) {
        npSetLED(i, 0, 255, 0);  // Cor verde (R = 0, G = 255, B = 0)
      }
      npWrite();
      display_message("POSTES DETECTADOS");  // Exibe mensagem no OLED
    } 
    else if (is_button_b_pressed()) {
      // Se o botão B for pressionado, acende a matriz na cor vermelha.
      npClear();
      for (int i = 0; i < LED_COUNT; ++i) {
        npSetLED(i, 255, 0, 0);  // Cor vermelha (R = 255, G = 0, B = 0)
      }
      npWrite();
      display_message("LOCALIZANDO FALHAS");  // Exibe mensagem no OLED
      play_error_sound();  // Toca o som de erro
    }

    bool current_joystick_state = is_joystick_pressed();

    // Detecta quando o joystick é pressionado (mudança de estado)
    if (current_joystick_state && !last_joystick_state) {
      // Escolhe dois LEDs aleatórios diferentes
      int led1 = rand() % LED_COUNT;
      int led2;
      do {
        led2 = rand() % LED_COUNT;
      } while (led2 == led1);

      // Define a cor branca para os LEDs escolhidos
      npSetLED(led1, 255, 255, 255);  // Branco (R=255, G=255, B=255)
      npSetLED(led2, 255, 255, 255);  // Branco (R=255, G=255, B=255)

      // Atualiza os LEDs
      npWrite();
      
      display_message("FALHAS LOCALIZADAS");  // Exibe mensagem no OLED
    }

    last_joystick_state = current_joystick_state;
    sleep_ms(50);  // Pequeno delay para evitar bouncing
  }
}

