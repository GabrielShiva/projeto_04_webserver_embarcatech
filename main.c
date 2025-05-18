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
#define WIFI_SSID "XXX"
#define WIFI_PASSWORD "XXX"

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43

// Definição de macros gerais
#define ADC_PIN 28
#define BTN_B_PIN 6
#define BTN_A_PIN 5
float adc_resolution = 4095;

// Definição de macros para o protocolo I2C (SSD1306)
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define SSD1306_ADDRESS 0x3C

// Inicialização de variáveis

int reference_resistor = 470; // Resistência conhecida

float cumulative_adc_measure = 0.0f;
float average_adc_measures = 0.0f;
float unknown_resistor = 0.0f;
float closest_e24_resistor = 0.0f;

// Define variáveis para debounce do botão
volatile uint32_t last_time_btn_press = 0;
bool is_matrix_enabled = true;

// Debounce delay
const uint32_t debounce_delay_ms = 260;

// Inicializa instância do display
ssd1306_t ssd;

// Definição de tabela para valores dos resistores da série e24
const float e24_resistor_values[24] = {1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1};
const int num_e24_resistor_values = sizeof(e24_resistor_values) / sizeof(e24_resistor_values[0]);

const char *available_digit_colors[10] = {"preto", "marrom", "vermelho", "laranja", "amarelo", "verde", "azul", "violeta", "cinza", "branco"};
const char *resistor_band_colors[3] = {0};
int resistor_band_color_indexes[3] = {
  0, // primeira banda
  0, // segunda banda
  0  // multiplicador
};

// definição do header do HTML
static const char page_header[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: text/html\r\n"
  "\r\n"
  "<!DOCTYPE html>\n"
  "<html>\n"
  "<head>\n"
  "  <meta charset=\"utf-8\">\n"
  "  <title>Medidor de Resistencia</title>\n"
  "  <style>\n"
  "    body { background-color:rgb(216,216,216); font-family:Arial,sans-serif; text-align:center; margin-top:50px; }\n"
  "    h1 { font-size:35px; }\n"
  "    .temperature { font-size:20px; margin:10px 0; color:#333; }\n"
  "  </style>\n"
  "</head>\n"
  "<body>\n"
  "  <h1>Medidor de Resistencia</h1>\n";

// definição do footer e script para atualizar a página do HTML
static const char page_footer[] =
  "  <script>\n"
  "    setInterval(() => { window.location.reload(); }, 1000);\n"
  "  </script>\n"
  "</body>\n"
  "</html>\n";


// Armazena o texto que será exibido no display OLED
char display_text[20] = {0};

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Leitura da temperatura interna
float resistor_measure(void);

// Obtenção do resistor da série e24 mais próximo do valor medido
float get_closest_e24_resistor(float resistor_value);

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
  ssd1306_draw_string(&ssd, "Inic. WiFi...", 5, 30);
  ssd1306_send_data(&ssd);

  sleep_ms(2000);

  //Inicializa a arquitetura do cyw43
  while (cyw43_arch_init()) {
      printf("Falha ao inicializar Wi-Fi!\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Ini Falhou", 5, 30);
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
  ssd1306_draw_string(&ssd, "Conectando WiFi", 5, 30);
  ssd1306_send_data(&ssd);

  while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
      printf("Falha ao conectar ao Wi-Fi\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Conexao Falhou", 5, 30);
      ssd1306_send_data(&ssd);
      sleep_ms(100);
      return -1;
  }

  sleep_ms(2000);

  printf("Conectado ao Wi-Fi!\n");
  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Conectou WiFi", 5, 30);
  ssd1306_send_data(&ssd);

  // Caso seja a interface de rede padrão - imprimir o IP do dispositivo.
  if (netif_default) {
      printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
  }

  sleep_ms(2000);

  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Criando Server", 5, 30);
  ssd1306_send_data(&ssd);

  sleep_ms(2000);

  // Configura o servidor TCP - cria novos PCBs TCP. É o primeiro passo para estabelecer uma conexão TCP.
  struct tcp_pcb *server = tcp_new();
  if (!server) {
      printf("Falha ao criar servidor TCP\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Falhou Server", 5, 30);
      ssd1306_send_data(&ssd);
      return -1;
  }

  //vincula um PCB (Protocol Control Block) TCP a um endereço IP e porta específicos.
  if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK) {
      printf("Falha ao associar servidor TCP à porta 80\n");
      ssd1306_fill(&ssd, !color);
      ssd1306_draw_string(&ssd, "Falhou Server", 5, 30);
      ssd1306_send_data(&ssd);
      return -1;
  }

  // Coloca um PCB (Protocol Control Block) TCP em modo de escuta, permitindo que ele aceite conexões de entrada.
  server = tcp_listen(server);

  // Define uma função de callback para aceitar conexões TCP de entrada. É um passo importante na configuração de servidores TCP.
  tcp_accept(server, tcp_server_accept);
  printf("Servidor ouvindo na porta 80\n");
  ssd1306_fill(&ssd, !color);
  ssd1306_draw_string(&ssd, "Criou Server", 5, 30);
  ssd1306_send_data(&ssd);

  sleep_ms(3000);

  ssd1306_fill(&ssd, !color);
  ssd1306_send_data(&ssd);

  while (true) {
    // Cálculo da resistencia em ohms e obtenção do valor comercial mais próximo
    unknown_resistor = resistor_measure();
    closest_e24_resistor = get_closest_e24_resistor(unknown_resistor);

    get_band_color(&closest_e24_resistor);

    // Limpeza do display
    ssd1306_fill(&ssd, false);
    draw_display_layout(&ssd);

     // Exibição do valor comercial da resistência mais próxima
    sprintf(display_text, "%.0f ohms", closest_e24_resistor);
    ssd1306_draw_string(&ssd, display_text, 29, 5);

    // Exibição das cores de cada banda (Tolerância Multiplicador Faixa_2 Faixa_1)
    ssd1306_draw_string(&ssd, "1=", 5, 20);
    ssd1306_draw_string(&ssd, resistor_band_colors[0], 60, 20);

    ssd1306_draw_string(&ssd, "2=", 5, 31);
    ssd1306_draw_string(&ssd, resistor_band_colors[1], 60, 31);

    ssd1306_draw_string(&ssd, "mult=", 5, 42);
    ssd1306_draw_string(&ssd, resistor_band_colors[2], 60, 42);

    ssd1306_draw_string(&ssd, "tol=", 5, 52);
    ssd1306_draw_string(&ssd, "Au (5%)", 60, 52);

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
}

void gpio_irq_handler(uint gpio, uint32_t events) {
  uint32_t current_time = to_ms_since_boot(get_absolute_time()); // retorna o tempo total em ms desde o boot do rp2040

  // verifica se a diff entre o tempo atual e a ultima vez que o botão foi pressionado é maior que o tempo de debounce
  if (current_time - last_time_btn_press > debounce_delay_ms) {
    last_time_btn_press = current_time;

    if (gpio == BTN_B_PIN) {
      reset_usb_boot(0, 0);
    }
  }
}

float resistor_measure(void) {
  // Seleciona o ADC para pino 28 como entrada analógica
    adc_select_input(2);

    // Obtenção de várias leituras seguidas e média
    cumulative_adc_measure = 0.0f;

    for (int i = 0; i < 100; i++) {
      cumulative_adc_measure += adc_read();
      sleep_us(10);
    }

    average_adc_measures = cumulative_adc_measure / 100.0f;

    // Cálculo da resistencia em ohms e obtenção do valor comercial mais próximo
    return (reference_resistor * average_adc_measures) / (adc_resolution - average_adc_measures);
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

  for (int i = 0; i < num_e24_resistor_values; i++) {
    float curr_diff = fabs(normalized_resistor - e24_resistor_values[i]);

    if (curr_diff < min_diff) {
      min_diff = curr_diff;
      closest_resistor = e24_resistor_values[i];
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

  // Obtenção do valor da primeira banda
  // EX.: 3.7 => (int)(3.7) => 3
  int first_band_value = (int)normalized_resistor;

  // Obtenção do valor da segunda banda
  // EX.: 3.7 => 3.7 * 10 => 37 => 37 % 10 => 7.0 => (int)(7.0) => 7
  int second_band_value = (int)(normalized_resistor * 10) % 10;

  // Definição da das Bandas 1, 2 e multiplicador
  resistor_band_colors[0] = available_digit_colors[first_band_value % 10];
  resistor_band_colors[1] = available_digit_colors[second_band_value % 10];
  resistor_band_colors[2] = (exponent >= 0 && exponent <= 9) ? available_digit_colors[exponent] : "erro";

  resistor_band_color_indexes[0] = first_band_value % 10;
  resistor_band_color_indexes[1] = second_band_value % 10;
  resistor_band_color_indexes[2] = (exponent >= 0 && exponent <= 9) ? exponent : 0;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // copia a requisição
    char *request = malloc(p->len+1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    // Envia o header da página
    tcp_write(tpcb, page_header, strlen(page_header), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // Cria o corpo da página e envia com os valores de resistência atualizados
    char body[1024];
    int body_len = snprintf(body, sizeof(body),
        "  <p class=\"temperature\">Numero de faixas: <span>4</span></p>\n"
        "  <p class=\"temperature\">Valor Medido: <span id=\"measuredValue\">%.0f</span> &#8486;</p>\n"
        "  <p class=\"temperature\">Valor Comercial: <span id=\"commercialValue\">%.0f</span> &#8486;</p>\n"
        "  <h1 style='font-size:25px;'>Cores das Faixas</h1>\n"
        "  <p class=\"temperature\">1 Faixa: <span>%s</span></p>\n"
        "  <p class=\"temperature\">2 Faixa: <span>%s</span></p>\n"
        "  <p class=\"temperature\">Multiplicador: <span>%s</span></p>\n"
        "  <p class=\"temperature\">Tolerancia: <span>Au (5%%)</span></p>\n",
        unknown_resistor,
        closest_e24_resistor,
        resistor_band_colors[0],
        resistor_band_colors[1],
        resistor_band_colors[2]
    );
    tcp_write(tpcb, body, body_len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // Envia o footer da págian HTML
    tcp_write(tpcb, page_footer, strlen(page_footer), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // Faz a limpeza do request
    free(request);
    pbuf_free(p);
    return ERR_OK;
}
