#include <stdio.h>               // Biblioteca padrão para entrada e saída
#include <string.h>              // Biblioteca manipular strings
#include <math.h>
#include <stdlib.h>              // funções para realizar várias operações, incluindo alocação de memória dinâmica (malloc)

#include "pico/stdlib.h"         // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/bootrom.h"
#include "hardware/adc.h"        // Biblioteca da Raspberry Pi Pico para manipulação do conversor ADC
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "pico/cyw43_arch.h"     // Biblioteca para arquitetura Wi-Fi da Pico com CYW43

#include "lwip/pbuf.h"           // Lightweight IP stack - manipulação de buffers de pacotes de rede
#include "lwip/tcp.h"            // Lightweight IP stack - fornece funções e estruturas para trabalhar com o protocolo TCP
#include "lwip/netif.h"          // Lightweight IP stack - fornece funções e estruturas para trabalhar com interfaces de rede (netif)

// Credenciais WIFI - Tome cuidado se publicar no github!
const char* WIFI_SSID = "JR TELECOM-LAR";
const char* WIFI_PASSWORD = "Rama2000";

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43

// Definição de macros gerais
#define ADC_PIN 28
#define BTN_B_PIN 6
#define BTN_A_PIN 5

// Definição de macros para o protocolo I2C (SSD1306)
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define SSD1306_ADDRESS 0x3C

// Inicialização de variáveis
int reference_resistor = 470; // Resistência conhecida
float unknown_resistor = 0.0f;
float closest_comercial_resistor = 0.0f;
volatile bool is_four_bands_mode = true;

// Define variáveis para debounce do botão
volatile uint32_t last_time_btn_press = 0;
const uint32_t debounce_delay_ms = 260;

// Inicializa instância do display
ssd1306_t ssd;

// Definição de tabela para valores dos resistores da série e24
const float e24_resistor_values[24] = {
  1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0,
  2.2, 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.3,
  4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1
};

// Definição de tabela para valores dos resistores da série e96
const float e96_resistor_values[96] = {
  1.00, 1.02, 1.05, 1.07, 1.10, 1.13, 1.15, 1.18,
  1.21, 1.24, 1.27, 1.30, 1.33, 1.37, 1.40, 1.43,
  1.47, 1.50, 1.54, 1.58, 1.62, 1.65, 1.69, 1.74,
  1.78, 1.82, 1.87, 1.91, 1.96, 2.00, 2.05, 2.10,
  2.15, 2.21, 2.26, 2.32, 2.37, 2.43, 2.49, 2.55,
  2.61, 2.67, 2.74, 2.80, 2.87, 2.94, 3.01, 3.09,
  3.16, 3.24, 3.32, 3.40, 3.48, 3.57, 3.65, 3.74,
  3.83, 3.92, 4.02, 4.12, 4.22, 4.32, 4.42, 4.53,
  4.64, 4.75, 4.87, 4.99, 5.11, 5.23, 5.36, 5.49,
  5.62, 5.76, 5.90, 6.04, 6.19, 6.34, 6.49, 6.65,
  6.81, 6.98, 7.15, 7.32, 7.50, 7.68, 7.87, 8.06,
  8.25, 8.45, 8.66, 8.87, 9.09, 9.31, 9.53, 9.76
};

const char *available_digit_colors[10] = {
  "preto", "marrom", "vermelho", "laranja", "amarelo",
  "verde", "azul", "violeta", "cinza", "branco"
};

const char *resistor_band_colors[4] = {0};
int resistor_band_color_indexes[4] = {
  0, // primeira banda
  0, // segunda banda
  0, // terceira banda
  0 // multiplicador
};

// Armazena o texto que será exibido no display OLED
char display_text[40] = {0};

static const char page_header[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/html\r\n"
  "\r\n"
  "<!DOCTYPE html>\n"
  "<html>\n"
  "<head>\n"
  "<meta charset='UTF-8'>\n"
  "<title>Medidor de Resist\u00eancia</title>\n"
  "  <style>\n"
  "    body{background-color:#d8d8d8;font-family:Arial,sans-serif;text-align:center;margin-top:50px;}\n"
  "    p{font-size:20px;margin:10px 0;}\n"
  "  </style>\n"
  "</head>\n"
  "<body>\n"
  "<h3>Medidor de Resist\u00eancia</h3>\n"
  ;

static const char page_footer[] ="</body>\n"
  "</html>\n";

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Leitura da temperatura interna
float resistor_measure(void);

// Obtenção do resistor da série e24 mais próximo do valor medido
float get_closest_e24_resistor(float resistor_value);

// Obtenção do resistor da série e96 mais próximo do valor medido
float get_closest_e96_resistor(float resistor_value);

// Obtenção das cores de cada uma das bandas do resistor (4 bandas) -> 5 bandas ainda será implementado
void get_band_color(float *resistor_value);

// Tratamento do request do usuário
void user_request(char **request);

// Inicialização do protocolo I2C para comunicação com o display OLED
void i2c_setup(uint baud_in_kilo);

// Inicializa o display OLED
void ssd1306_setup(ssd1306_t *ssd_ptr);

// Desenha o conteúdo do display OLED
void draw_display_layout(ssd1306_t *ssd_ptr);

// Inicializa a função que realiza o tratamento das interrupções dos botões
void gpio_irq_handler(uint gpio, uint32_t events);

int main() {
  // [INÍCIO] modo BOOTSEL associado ao botão B (apenas para desenvolvedores)
  gpio_init(BTN_B_PIN);
  gpio_set_dir(BTN_B_PIN, GPIO_IN);
  gpio_pull_up(BTN_B_PIN);
  gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  // [FIM] modo BOOTSEL associado ao botão B (apenas para desenvolvedores)

  //Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
  stdio_init_all();

  gpio_init(BTN_A_PIN);
  gpio_set_dir(BTN_A_PIN, GPIO_IN);
  gpio_pull_up(BTN_A_PIN);
  gpio_set_irq_enabled(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true);

  // Inicialização do protocolo I2C com 400Khz e inicialização do display
  i2c_setup(400);
  ssd1306_setup(&ssd);

   // Inicialização do ADC para o pino 28
  adc_init();
  adc_gpio_init(ADC_PIN);

  sleep_ms(3000);
  printf("Pico foi iniciado com sucesso.\n");

  bool color = true;
  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Inicializando", 5, 20);
  ssd1306_draw_string(&ssd, "Wi-Fi...", 5, 30);
  ssd1306_send_data(&ssd);

  sleep_ms(2000);

  //Inicializa a arquitetura do cyw43
  while (cyw43_arch_init()) {
      printf("Falha ao inicializar Wi-Fi!\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Inicializacao", 5, 20);
      ssd1306_draw_string(&ssd, "Falhou...", 5, 30);
      ssd1306_send_data(&ssd);
      sleep_ms(100);
      return -1;
  }

  // GPIO do CI CYW43 em nível baixo
  cyw43_arch_gpio_put(LED_PIN, 0);

  // Ativa o Wi-Fi no modo Station, de modo a que possam ser feitas ligações a outros pontos de acesso Wi-Fi.
  cyw43_arch_enable_sta_mode();

  // Conectar à rede WiFI - fazer um loop até que esteja conectado
  printf("Conectando ao Wi-Fi...\n");
  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Conectando em", 5, 20);
  snprintf(display_text, sizeof(display_text), "%s", WIFI_SSID);
  ssd1306_draw_string(&ssd, display_text, 5, 30);
  ssd1306_send_data(&ssd);

  while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
      printf("Falha ao conectar ao Wi-Fi\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Conexao", 5, 20);
      ssd1306_draw_string(&ssd, "Falhou...", 5, 30);
      ssd1306_send_data(&ssd);
      sleep_ms(100);
      return -1;
  }

  sleep_ms(2000);

  printf("Conectado ao Wi-Fi!\n");
  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Conectou ao", 5, 20);
  ssd1306_draw_string(&ssd, "Wi-Fi", 5, 30);
  ssd1306_send_data(&ssd);

  // Caso seja a interface de rede padrão - imprimir o IP do dispositivo.
  if (netif_default) {
      printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
  }

  sleep_ms(2000);

  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Criando", 5, 20);
  ssd1306_draw_string(&ssd, "servidor...", 5, 30);
  ssd1306_send_data(&ssd);

  sleep_ms(2000);

  // Configura o servidor TCP - cria novos PCBs TCP. É o primeiro passo para estabelecer uma conexão TCP.
  struct tcp_pcb *server = tcp_new();
  if (!server) {
      printf("Falha ao criar servidor TCP\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Falhou em", 5, 20);
      ssd1306_draw_string(&ssd, "criar servidor", 5, 30);
      ssd1306_send_data(&ssd);
      return -1;
  }

  //vincula um PCB (Protocol Control Block) TCP a um endereço IP e porta específicos.
  if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK) {
      printf("Falha ao associar servidor TCP à porta 80\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Falhou em", 5, 20);
      ssd1306_draw_string(&ssd, "criar servidor", 5, 30);
      ssd1306_send_data(&ssd);
      return -1;
  }

  // Coloca um PCB (Protocol Control Block) TCP em modo de escuta, permitindo que ele aceite conexões de entrada.
  server = tcp_listen(server);

  // Define uma função de callback para aceitar conexões TCP de entrada. É um passo importante na configuração de servidores TCP.
  tcp_accept(server, tcp_server_accept);
  printf("Servidor ouvindo na porta 80\n");
  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Servidor criado", 5, 20);
  ssd1306_draw_string(&ssd, "com sucesso", 5, 30);
  ssd1306_send_data(&ssd);

  sleep_ms(3000);

  ssd1306_fill(&ssd, !color);
  ssd1306_send_data(&ssd);

  while (true) {
    // Cálculo da resistencia em ohms e obtenção do valor comercial mais próximo
    unknown_resistor = resistor_measure();
    closest_comercial_resistor = is_four_bands_mode ? get_closest_e24_resistor(unknown_resistor) : get_closest_e96_resistor(unknown_resistor);

    get_band_color(&closest_comercial_resistor);

    // Limpeza do display
    ssd1306_fill(&ssd, false);
    draw_display_layout(&ssd);

    // Exibição do valor comercial da resistência mais próxima
    sprintf(display_text, "%.0f ohms", closest_comercial_resistor);
    ssd1306_draw_string(&ssd, display_text, 29, 5);

    // Exibição das cores de cada banda (Tolerância Multiplicador Faixa_2 Faixa_1)
    ssd1306_draw_string(&ssd, "1=", 5, 20);
    ssd1306_draw_string(&ssd, resistor_band_colors[0], 60, 20);

    ssd1306_draw_string(&ssd, "2=", 5, 31);
    ssd1306_draw_string(&ssd, resistor_band_colors[1], 60, 31);

    if (is_four_bands_mode) {
      ssd1306_draw_string(&ssd, "mult=", 5, 42);
      ssd1306_draw_string(&ssd, resistor_band_colors[3], 60, 42);
    } else {
      ssd1306_draw_string(&ssd, "3=", 5, 42);
      ssd1306_draw_string(&ssd, resistor_band_colors[2], 60, 42);

      ssd1306_draw_string(&ssd, "mult=", 5, 52);
      ssd1306_draw_string(&ssd, resistor_band_colors[3], 60, 52);
    }

    ssd1306_send_data(&ssd);

    /*
    * Efetuar o processamento exigido pelo cyw43_driver ou pela stack TCP/IP.
    * Este método deve ser chamado periodicamente a partir do ciclo principal
    * quando se utiliza um estilo de sondagem pico_cyw43_arch
    */
    cyw43_arch_poll(); // Necessário para manter o Wi-Fi ativo
    sleep_ms(100);      // Reduz o uso da CPU
  }

  //Desligar a arquitetura CYW43.
  cyw43_arch_deinit();
  return 0;
}

// -------------------------------------- Funções ---------------------------------

void i2c_setup(uint baud_in_kilo) {
  i2c_init(I2C_PORT, baud_in_kilo * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

void ssd1306_setup(ssd1306_t *ssd_ptr) {
  ssd1306_init(ssd_ptr, WIDTH, HEIGHT, false, SSD1306_ADDRESS, I2C_PORT); // Inicializa o display
  ssd1306_config(ssd_ptr);                                                // Configura o display
  ssd1306_send_data(ssd_ptr);                                             // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(ssd_ptr, false);
  ssd1306_send_data(ssd_ptr);
}

void draw_display_layout(ssd1306_t *ssd_ptr) {
  // desenho dos contornos do layout do display
  ssd1306_rect(ssd_ptr, 1, 1, 126, 62, 1, 0);
  //cima
  ssd1306_line(ssd_ptr, 5, 5, 5, 11, 1);
  ssd1306_line(ssd_ptr, 6, 4, 10, 4, 1);
  ssd1306_line(ssd_ptr, 10, 5, 20, 5, 1);
  //baixo
  ssd1306_line(ssd_ptr, 6, 12, 10, 12, 1);
  ssd1306_line(ssd_ptr, 10, 11, 20, 11, 1);
  //primeira faixa
  ssd1306_line(ssd_ptr, 8, 4, 8, 12, 1);
  ssd1306_line(ssd_ptr, 9, 4, 9, 12, 1);
  //segunda faixa
  ssd1306_line(ssd_ptr, 13, 5, 13, 11, 1);
  ssd1306_line(ssd_ptr, 14, 5, 14, 11, 1);
  //multiplicador
  ssd1306_line(ssd_ptr, 17, 5, 17, 11, 1);
  ssd1306_line(ssd_ptr, 18, 5, 18, 11, 1);
  //tolerancia
  ssd1306_line(ssd_ptr, 21, 5, 21, 11, 1);
  ssd1306_line(ssd_ptr, 22, 5, 22, 11, 1);
  //cima
  ssd1306_line(ssd_ptr, 20, 4, 24, 4, 1);
  ssd1306_line(ssd_ptr, 20, 12, 24, 12, 1);
  ssd1306_line(ssd_ptr, 25, 5, 25, 11, 1);

  // // seta para a tolerancia
  // ssd1306_line(ssd_ptr, 21, 15, 21, 23, 1);
  // ssd1306_line(ssd_ptr, 22, 15, 22, 23, 1);

  // ssd1306_line(ssd_ptr, 22, 22, 50, 22, 1);
  // ssd1306_line(ssd_ptr, 22, 23, 50, 23, 1);

  // ssd1306_line(ssd_ptr, 50, 20, 50, 25, 1);
  // ssd1306_line(ssd_ptr, 51, 21, 51, 24, 1);
  // ssd1306_line(ssd_ptr, 52, 22, 52, 23, 1);

  // // seta para o multiplicador
  // ssd1306_line(ssd_ptr, 17, 15, 17, 35, 1);
  // ssd1306_line(ssd_ptr, 18, 15, 18, 35, 1);

  // ssd1306_line(ssd_ptr, 18, 34, 50, 34, 1);
  // ssd1306_line(ssd_ptr, 18, 35, 50, 35, 1);

  // ssd1306_line(ssd_ptr, 50, 32, 50, 37, 1);
  // ssd1306_line(ssd_ptr, 51, 33, 51, 36, 1);
  // ssd1306_line(ssd_ptr, 52, 34, 52, 35, 1);

  // // seta para a segunda faixa
  // ssd1306_line(ssd_ptr, 13, 15, 13, 47, 1);
  // ssd1306_line(ssd_ptr, 14, 15, 14, 47, 1);

  // ssd1306_line(ssd_ptr, 14, 46, 50, 46, 1);
  // ssd1306_line(ssd_ptr, 14, 47, 50, 47, 1);

  // ssd1306_line(ssd_ptr, 50, 44, 50, 49, 1);
  // ssd1306_line(ssd_ptr, 51, 45, 51, 48, 1);
  // ssd1306_line(ssd_ptr, 52, 46, 52, 47, 1);

  // // seta para a primeira faixa
  // ssd1306_line(ssd_ptr, 8, 15, 8, 57, 1);
  // ssd1306_line(ssd_ptr, 9, 15, 9, 57, 1);

  // ssd1306_line(ssd_ptr, 9, 56, 50, 56, 1);
  // ssd1306_line(ssd_ptr, 9, 57, 50, 57, 1);

  // ssd1306_line(ssd_ptr, 50, 54, 50, 59, 1);
  // ssd1306_line(ssd_ptr, 51, 55, 51, 58, 1);
  // ssd1306_line(ssd_ptr, 52, 56, 52, 57, 1);
}

void gpio_irq_handler(uint gpio, uint32_t events) {
  uint32_t current_time = to_ms_since_boot(get_absolute_time()); // retorna o tempo total em ms desde o boot do rp2040

  // verifica se a diff entre o tempo atual e a ultima vez que o botão foi pressionado é maior que o tempo de debounce
  if (current_time - last_time_btn_press > debounce_delay_ms) {
    last_time_btn_press = current_time;

    if (gpio == BTN_A_PIN) {
      is_four_bands_mode = !is_four_bands_mode;

      if (is_four_bands_mode) {
        printf("modo: 4 faixas.\n");
      } else {
        printf("modo: 5 faixas.\n");
      }
    } else if (gpio == BTN_B_PIN) {
      reset_usb_boot(0, 0);
    }
  }
}

float resistor_measure(void) {
  // Seleciona o ADC para pino 28 como entrada analógica
    adc_select_input(2);

    // Obtenção de várias leituras seguidas e média
    float cumulative_adc_measure = 0.0f;

    for (int i = 0; i < 100; i++) {
      cumulative_adc_measure += adc_read();
      sleep_us(10);
    }

    float average_adc_measures = cumulative_adc_measure / 100.0f;

    // Cálculo da resistencia em ohms e obtenção do valor comercial mais próximo
    return (reference_resistor * average_adc_measures) / (4095 - average_adc_measures);
}

float get_closest_e24_resistor(float resistor_value) {
  if (resistor_value <= 0) {
     return 0.0;
  }

  float normalized_resistor = resistor_value;
  float exponent = 0.0f;

  // Normaliza o valor fornecido para a faixa [0-10]
  while (normalized_resistor >= 10) {
    normalized_resistor = normalized_resistor / 10;
    exponent = exponent + 1.0;
  }

  float closest_resistor = e24_resistor_values[0];
  float min_diff = fabs(normalized_resistor - e24_resistor_values[0]);

  for (int i = 0; i < 24; i++) {
    float curr_diff = fabs(normalized_resistor - e24_resistor_values[i]);

    if (curr_diff < min_diff) {
      min_diff = curr_diff;
      closest_resistor = e24_resistor_values[i];
    }
  }

  return closest_resistor * powf(10.0, exponent);
}

float get_closest_e96_resistor(float resistor_value) {
  if (resistor_value <= 0) {
     return 0.0;
  }

  float normalized_resistor = resistor_value;
  float exponent = 0.0f;

  // Normaliza o valor fornecido para a faixa [0-10]
  while (normalized_resistor >= 10) {
    normalized_resistor = normalized_resistor / 10;
    exponent = exponent + 1.0;
  }

  float closest_resistor = e96_resistor_values[0];
  float min_diff = fabs(normalized_resistor - e96_resistor_values[0]);

  for (int i = 0; i < 96; i++) {
    float curr_diff = fabs(normalized_resistor - e96_resistor_values[i]);

    if (curr_diff < min_diff) {
      min_diff = curr_diff;
      closest_resistor = e96_resistor_values[i];
    }
  }

  return closest_resistor * powf(10.0, exponent);
}

void get_band_color(float *resistor_value) {
  // Cálculo das cores de cada banda do resistor (4 bandas)
  float normalized_resistor = *resistor_value;
  int exponent = -1;

  // Normaliza o valor fornecido para a faixa [0-10]
  while (normalized_resistor >= 10.0) {
    normalized_resistor = normalized_resistor / 10;
    exponent = exponent + 1;
  }

  // Definição da das Bandas 1, 2, 3 e multiplicador
  resistor_band_colors[0] = available_digit_colors[(int)normalized_resistor];
  resistor_band_color_indexes[0] = (int)normalized_resistor;

  resistor_band_colors[1] = available_digit_colors[(int)((int)(normalized_resistor * 10) % 10)];
  resistor_band_color_indexes[1] = (int)((int)(normalized_resistor * 10) % 10);

  if (!is_four_bands_mode) {
    resistor_band_colors[2] = available_digit_colors[(int)(normalized_resistor * 100) % 10];
    resistor_band_color_indexes[2] = (int)(normalized_resistor * 100) % 10;
  }

  resistor_band_colors[3] = (exponent >= 0 && exponent <= 9) ? available_digit_colors[exponent] : "erro";
  resistor_band_color_indexes[3] = (exponent >= 0 && exponent <= 9) ? exponent : 0;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Tratamento do request do usuário - digite aqui
void user_request(char **request) {
    if (strstr(*request, "GET /four_bands") != NULL) {
        is_four_bands_mode = true;
    } else if (strstr(*request, "GET /five_bands") != NULL) {
        is_four_bands_mode = false;
    }
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // Transforma a requisição capturada para uma string
    char *request = malloc(p->len + 1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    printf("REQUEST: %s\n", request);

    // Se existir uma requisição para o endpoint /data executa o bloco de código abaixo
    if (strstr(request, "GET /data") != NULL) {
        // Ajusta o modo de leitura (4 bandas/ 5 bandas) conforme query string
        if (strstr(request, "mode=five") != NULL) {
            printf("CINCO\n");
            is_four_bands_mode = false;
        } else {
            printf("QUATRO\n");
            is_four_bands_mode = true;
        }

        // Monta o corpo da resposta
        char json[1024];
        int body_len = 0;

        if (is_four_bands_mode) {
            body_len = snprintf(json, sizeof(json),
                "<p>Num de faixas: 4</p>"
                "<p>Medido: %.0f &#8486;</p>"
                "<p>Comercial: %.0f &#8486;</p>"
                "<h3>Cores das Faixas</h3>"
                "<p>1 Faixa: %s</p>"
                "<p>2 Faixa: %s</p>"
                "<p>Mult.: %s</p>"
                "<p>Tole.: Au (5%%)</p>",
                unknown_resistor,
                closest_comercial_resistor,
                resistor_band_colors[0],
                resistor_band_colors[1],
                resistor_band_colors[3]
            );
        } else {
            body_len = snprintf(json, sizeof(json),
                "<p>Num de faixas: 5</p>"
                "<p>Medido: %.0f &#8486;</p>"
                "<p>Comercial: %.0f &#8486;</p>"
                "<h3>Cores das Faixas</h3>"
                "<p>1 Faixa: %s</p>"
                "<p>2 Faixa: %s</p>"
                "<p>3 Faixa: %s</p>"
                "<p>Mult.: %s</p>"
                "<p>Tole.: Au (5%%)</p>",
            unknown_resistor,
            closest_comercial_resistor,
            resistor_band_colors[0],
            resistor_band_colors[1],
            resistor_band_colors[2],
            resistor_band_colors[3]
            );
        }

        // monta o header com Content-Length
        char hdr[128];
        int hdr_len = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=UTF-8\r\n"
        "Content-Length: %d\r\n"
        "\r\n",
        body_len
        );

        // envia header + corpo
        tcp_write(tpcb, hdr,     hdr_len,  TCP_WRITE_FLAG_COPY);
        tcp_write(tpcb, json,    body_len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);

        // informa ao lwIP que já lemos os bytes do request
        tcp_recved(tpcb, p->len);

        free(request);
        pbuf_free(p);
        return ERR_OK;
    }

    // Caso não seja AJAX, envia a página normal
    tcp_write(tpcb, page_header, strlen(page_header), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // Monta o corpo com botões e script
    char body[1024];
    if (is_four_bands_mode) {
        snprintf(body, sizeof(body),
            "<button onclick=\"setMode('four')\">4 bandas</button>"
            "<button onclick=\"setMode('five')\">5 bandas</button>"
            "<div id='cont'>"
            "<p>Num de faixas: 4</p>"
            "<p>Medido: %.0f &#8486;</p>"
            "<p>Comercial: %.0f &#8486;</p>"
            "<h3>Cores das Faixas</h3>"
            "<p>1 Faixa: %s</p>"
            "<p>2 Faixa: %s</p>"
            "<p>Mult.: %s</p>"
            "<p>Tole.: Au (5%%)</p>"
            "</div>"
            "<script>"
            "let mode = 'four';"
            "function setMode(m) {"
            "  mode = m;"
            "  fetchData();"
            "}"
            "function fetchData() {"
            "  fetch('/data?mode=' + mode)"
            "    .then(function(res) { return res.text(); })"
            "    .then(function(txt) {"
            "      document.getElementById('cont').innerHTML = txt;"
            "    })"
            "    .catch(function(err) { console.error(err); });"
            "}"
            "  setInterval(function(){"
            "    window.location.reload();"
            "  }, 2000);"
            "</script>",
                unknown_resistor,
                closest_comercial_resistor,
                resistor_band_colors[0],
                resistor_band_colors[1],
                resistor_band_colors[3]);
    } else {
        snprintf(body, sizeof(body),
            "<button onclick=\"setMode('four')\">4 bandas</button>"
            "<button onclick=\"setMode('five')\">5 bandas</button>"
            "<div id='cont'>"
            "<p>Num de faixas: 5</p>"
            "<p>Medido: %.0f &#8486;</p>"
            "<p>Comercial: %.0f &#8486;</p>"
            "<h3>Cores das Faixas</h3>"
            "<p>1 Faixa: %s</p>"
            "<p>2 Faixa: %s</p>"
            "<p>3 Faixa: %s</p>"
            "<p>Mult.: %s</p>"
            "<p>Tole.: Au (5%%)</p>"
            "</div>"
            "<script>"
            "let mode = 'five';"
            "function setMode(m) {"
            "  mode = m;"
            "  fetchData();"
            "}"
            "function fetchData() {"
            "  fetch('/data?mode=' + mode)"
            "    .then(function(res) { return res.text(); })"
            "    .then(function(txt) {"
            "      document.getElementById('cont').innerHTML = txt;"
            "    })"
            "    .catch(function(err) { console.error(err); });"
            "}"
            "  setInterval(function(){"
            "    fetchData();"
            "  }, 2000);"
            "</script>",
                unknown_resistor,
                closest_comercial_resistor,
                resistor_band_colors[0],
                resistor_band_colors[1],
                resistor_band_colors[2],
                resistor_band_colors[3]);
    }

    tcp_write(tpcb, body, strlen(body), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // Envia footer da página
    tcp_write(tpcb, page_footer, strlen(page_footer), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    free(request);
    pbuf_free(p);
    return ERR_OK;
}
