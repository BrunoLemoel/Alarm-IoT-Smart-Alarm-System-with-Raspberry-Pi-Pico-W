#include <stdio.h>
#include <math.h>
#include <time.h>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define RELAY_PIN 21  // Pino GPIO conectado ao módulo relé
#define DS3231_ADDR 0x68 // Endereço I2C do DS1307
#define LED_PIN 15 // Pino GPIO conectado ao led
#define BUZZER_PIN 2 // Pino GPIO conectado ao buzzer
#define BUTTON_PIN 4 // Pino GPIO conectado ao botão
#define WIFI_SSID "Seu_SSID" // Nome da rede do seu wi-fi
#define WIFI_PASSWORD "Sua_Senha" // Senha da rede do seu wi-fi

#define NTC_PIN 28 // GPIO conectado ao sensor de temperatura (ADC 2)
#define BETA 3950 // Coeficiente beta do NTC
#define R_FIXED 10000 // Resistor fixo de 10kΩ
#define ADC_MAX 4095.0 // Resolução máxima do ADC (12 bits)
#define V_REF 3.3 // Tensão de referência

// I2C configuração
#define I2C_PORT i2c1 // Porta I2C
#define I2C_SDA_PIN 26 // GPIO26 para SDA
#define I2C_SCL_PIN 27 // GPIO27 para SCL
#define LCD_I2C_ADDRESS 0x27 // LCD I2C endereço (0x3F se necessário)
#define RTC_I2C_ADDRESS 0x68 // DS3231 RTC I2C endereço

// Comandos LCD
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_DDRAM_ADDR 0x80

// Bandeiras para configuração de LCD
#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_OFF 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_BACKLIGHT 0x08
#define ENABLE_BIT 0x04


uint8_t alarm_hour, alarm_minute, alarm_second; // Variáveis globais para o alarme
bool alarm_set = false;
bool alarm_active = false; // Variável para controlar o ciclo do alarme


void lcd_write_byte(uint8_t value) { // Função que envia byte ao LCD
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDRESS, &value, 1, false);
    sleep_us(50);
}


void connectToWiFi() { //Função que conecta ao Wi-Fi
    printf("Conectando ao Wi-Fi: %s\n", WIFI_SSID);
    if (cyw43_arch_init()) {
        printf("Falha na inicialização do módulo Wi-Fi.\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Falha ao conectar ao Wi-Fi.\n");
    } else {
        printf("Conexão Wi-Fi estabelecida!\n");
    }
}


void lcd_send_command(uint8_t cmd) { // Envia o comando ao LCD
    uint8_t high_nibble = (cmd & 0xF0) | LCD_BACKLIGHT;
    uint8_t low_nibble = ((cmd << 4) & 0xF0) | LCD_BACKLIGHT;

    lcd_write_byte(high_nibble | ENABLE_BIT);
    lcd_write_byte(high_nibble);
    lcd_write_byte(low_nibble | ENABLE_BIT);
    lcd_write_byte(low_nibble);
    sleep_ms(2);
}


void lcd_send_data(uint8_t data) { // Envia a data ao LCD
    uint8_t high_nibble = (data & 0xF0) | LCD_BACKLIGHT | 0x01;
    uint8_t low_nibble = ((data << 4) & 0xF0) | LCD_BACKLIGHT | 0x01;

    lcd_write_byte(high_nibble | ENABLE_BIT);
    lcd_write_byte(high_nibble);
    lcd_write_byte(low_nibble | ENABLE_BIT);
    lcd_write_byte(low_nibble);
    sleep_us(50);
}


void lcd_init() { // Inicializa o LCD
    sleep_ms(50); // Aguarda o LCD

    // inicializa em modo 4 bit 
    lcd_send_command(0x03);
    lcd_send_command(0x03);
    lcd_send_command(0x03);
    lcd_send_command(0x02);

    // Configura o LCD
    lcd_send_command(LCD_FUNCTION_SET | 0x08); // 4-bit modo, 2 linhas
    lcd_send_command(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    lcd_send_command(LCD_CLEAR_DISPLAY);
    lcd_send_command(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT);
}

// Define o curso positivo
void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = col + (row * 0x40);
    lcd_send_command(LCD_SET_DDRAM_ADDR | address);
}

// Impriome a String para LCD
void lcd_print(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Le o tempo para RTC
void rtc_read_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) {
    uint8_t buffer[3];
    i2c_write_blocking(I2C_PORT, RTC_I2C_ADDRESS, (uint8_t[]){0x00}, 1, true);
    i2c_read_blocking(I2C_PORT, RTC_I2C_ADDRESS, buffer, 3, false);

    *seconds = ((buffer[0] >> 4) * 10) + (buffer[0] & 0x0F);
    *minutes = ((buffer[1] >> 4) * 10) + (buffer[1] & 0x0F);
    *hours = ((buffer[2] >> 4) * 10) + (buffer[2] & 0x0F);
}

// Le a data para o RTC
void rtc_read_date(uint8_t *day, uint8_t *month, uint8_t *year) {
    uint8_t buffer[3];
    i2c_write_blocking(I2C_PORT, RTC_I2C_ADDRESS, (uint8_t[]){0x04}, 1, true);
    i2c_read_blocking(I2C_PORT, RTC_I2C_ADDRESS, buffer, 3, false);

    *day = ((buffer[0] >> 4) * 10) + (buffer[0] & 0x0F);
    *month = ((buffer[1] >> 4) * 10) + (buffer[1] & 0x0F);
    *year = ((buffer[2] >> 4) * 10) + (buffer[2] & 0x0F);
}

// Le temperatura para NTC
float calculate_temperature(uint16_t adc_value) {
    float voltage = adc_value * (V_REF / ADC_MAX); // Converte o valor ADC em tensão
    if (voltage == 0) return -273.15;             // Evita divisão por zero
    float resistance = (R_FIXED * voltage) / (V_REF - voltage); // Calcula a resistência do NTC
    float temperature = 1 / (log(resistance / R_FIXED) / BETA + 1 / 298.15) - 273.15; // Fórmula para Celsius
    return temperature;
}

// Configura PWM para o buzzer
void buzzer_play(uint slice_num, uint channel, float frequency) {
    uint32_t clock_div = 125000000 / (frequency * 100); // Calcula o divisor
    pwm_set_clkdiv(slice_num, clock_div);
    pwm_set_wrap(slice_num, 100);
    pwm_set_chan_level(slice_num, channel, 50); // Define duty cycle de 50%
    pwm_set_enabled(slice_num, true);          // Ativa o PWM
}

// Desliga o buzzer
void buzzer_stop(uint slice_num, uint channel) {
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, false);
}

// Função de debounce para leitura confiável do botão
bool is_button_pressed(uint button_pin) {
    if (!gpio_get(button_pin)) { // Botão pressionado (nível lógico baixo)
        sleep_ms(20); // Aguarda 20ms para confirmar o estado
        if (!gpio_get(button_pin)) { // Verifica novamente o estado
            return true; // Confirma que o botão está pressionado
        }
    }
    return false; // Botão não pressionado
}

void setup_relay() {
    gpio_init(RELAY_PIN); // Inicia o relé
    gpio_set_dir(RELAY_PIN, GPIO_OUT); // Define o pino como saída
    gpio_put(RELAY_PIN, 0); // Inicialmente o relé estará desligado
}



// Função para acionar o LED e o buzzer com diferentes frequências (simulando um som de alarme)
void activate_alarm() {
    alarm_active = true;
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PIN);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM); // Define função PWM para o buzzer
    // Aciona o relé
    gpio_put(RELAY_PIN, 1);  // Liga o relé

    // Sequência de frequências para o buzzer (imita um alarme)
    float frequencies[] = {1000, 1200, 800, 1500, 1000, 1200, 900};
    int num_frequencies = sizeof(frequencies) / sizeof(frequencies[0]);

    for (int cycle = 0; cycle < 6; cycle++) {
        if (!alarm_active) { // Sai do loop se o botão for pressionado
            printf("Alarme interrompido pelo botão.\n");
            break;
        }

        // Liga LED e inicia o ciclo do alarme
        gpio_put(LED_PIN, 1);
        for (int i = 0; i < num_frequencies; i++) {
            if (!alarm_active) { // Verifica novamente o botão durante o ciclo
                printf("Alarme interrompido pelo botão.\n");
                break;
            }
            buzzer_play(slice_num, channel, frequencies[i]);
            sleep_ms(1000); // Toca cada frequência por 1000ms

            // Verifica se o botão foi pressionado
            if (is_button_pressed(BUTTON_PIN)) {
                alarm_active = false;
                printf("Alarme desativado. \n");
                break;
            }

        }
        if (!alarm_active) break; // Verifica antes de desligar o LED

        // Desliga LED e buzzer
        gpio_put(LED_PIN, 0);
        buzzer_stop(slice_num, channel);
        sleep_ms(3000); // Pausa de 3 segundos entre os ciclos
    }

    // Garante que o LED e o buzzer estejam desligados no final
    gpio_put(LED_PIN, 0); //Desliga o led
    gpio_put(RELAY_PIN, 0); // Desliga o relé
    buzzer_stop(slice_num, channel); // Desliga o buzzer
    alarm_active = false; // Reseta o estado do alarme
}

// No loop principal
void check_alarm(uint8_t current_hour, uint8_t current_minute, uint8_t current_second) {
    if (alarm_set &&
        current_hour == alarm_hour &&
        current_minute == alarm_minute &&
        current_second == alarm_second) {
        printf("Alarme acionado!\n");
        activate_alarm();
        alarm_set = false; // Desativa o alarme após acioná-lo
    }
}

int main() {
    // Inicializa stdio
    stdio_init_all();

    // Inicializa ADC
    adc_init(); //Inicia o adc
    adc_gpio_init(NTC_PIN);  // Configura o GPIO26 como entrada do ADC
    adc_select_input(2); // Seleciona o canal 2 (GPIO28)

    // Initialize I2C
    i2c_init(I2C_PORT, 100 * 1000); // 100kHz I2C frequência
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); // Define função SDA
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C); // Define função SCL
    gpio_pull_up(I2C_SDA_PIN); // Ativa o resistor de pull-up no SDA
    gpio_pull_up(I2C_SCL_PIN); // Ativa o resistor de pull-up no SCL

    gpio_init(LED_PIN); // Inicia o led
    gpio_set_dir(LED_PIN, GPIO_OUT); // Configura o pino do led como entrada
    gpio_init(BUZZER_PIN); // Inicia o buzzer
    gpio_set_dir(BUZZER_PIN, GPIO_OUT); // Configura o pino do buzzer como entrada

    gpio_init(BUTTON_PIN); // Inicia o botão
    gpio_set_dir(BUTTON_PIN, GPIO_IN);  // Configura o pino do botão como entrada
    gpio_pull_up(BUTTON_PIN);  // Ativa o resistor de pull-up no botão

    lcd_init(); // Inicializar o LCD
    connectToWiFi(); // Conecta ao Wi-Fi
    setup_relay(); // Inicializa o módulo relé
    uint8_t hours, minutes, seconds;
    uint8_t day, month, year;
    char buffer[17];

    bool printed = false; // Bandeira para controlar a impressão no console

   // Loop principal
    while (true) {

        // Leia a hora e a data no RTC
        rtc_read_time(&hours, &minutes, &seconds);
        rtc_read_date(&day, &month, &year);

        // Exibir a hora na primeira linha
        lcd_set_cursor(0, 0);
        snprintf(buffer, sizeof(buffer), "Time: %02d:%02d:%02d", hours, minutes, seconds);
        lcd_print(buffer);

        // Exibir a data na segunda linha
        lcd_set_cursor(1, 0);
        snprintf(buffer, sizeof(buffer), "Date: %02d/%02d/20%02d", day, month, year);
        lcd_print(buffer);    

        // Verifica alarme
        check_alarm(hours, minutes, seconds);

        // Imprima a hora e a data no console apenas uma vez
        if (!printed) {
            printf("Time: %02d:%02d:%02d | Date: %02d/%02d/20%02d\n", hours, minutes, seconds, day, month, year);
            printed = true; // Defina o sinalizador como verdadeiro para que ele não seja impresso novamente

            uint16_t adc_value = adc_read(); // Lê o valor bruto do ADC
            float temperature = calculate_temperature(adc_value); // Calcula a temperatura
            printf("Temperatura: %.2f°C\n", temperature); // Exibe os resultados no console
            sleep_ms(1000); // Aguarda 1 segundo antes de repetir
        }

        // Pergunta ao usuário a data e hora do alarme
        if (!alarm_set) {
            printf("Qual data e hora deseja programar o alarme? (Formato: HH:MM:SS)\n");
            char input[9];
            scanf("%8s", input); // Lê no formato HH:MM:SS
            sscanf(input, "%hhu:%hhu:%hhu", &alarm_hour, &alarm_minute, &alarm_second);
            alarm_set = true;
            printf("Alarme programado para %02d:%02d:%02d\n", alarm_hour, alarm_minute, alarm_second);
        }

        // Obtém a data e hora atual do RTC
        uint8_t current_hour, current_minute, current_second;
        rtc_read_time(&current_hour, &current_minute, &current_second);
        printf("Hora atual: %02d:%02d:%02d\n", current_hour, current_minute, current_second);
        
        // Verifica se o alarme deve ser acionado
        if (alarm_active) {
            activate_alarm();
        } else {
            check_alarm(current_hour, current_minute, current_second);
        }


        // Aguarde 1 segundo
        sleep_ms(1000);
    
    }
    return 0;

}