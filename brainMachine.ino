#include "esp_timer.h"

#define SINAL1_ADC 34  // GPIO do primeiro sinal (ADC1_CH6)
#define SINAL2_ADC 35  // GPIO do segundo sinal (ADC1_CH7)
#define POT_INTENSITY 32
#define POT_RGB 33

//#define COMMON_CATHODE

#define COMMON_ANODE



class LedRGB {
private:
    int pinRed, pinGreen, pinBlue;
    int redValue, greenValue, blueValue;
    int redValueCurrent, greenValueCurrent, blueValueCurrent;    //cores com variação de intensidade das funções
    int redValueWithIntensity, greenValueWithIntensity, blueValueWithIntensity;    //cores com variação de intensidade das funções
    unsigned long previousMillis; // Para controlar o tempo com millis()
    float frequency; // Frequência de piscagem em hertz
    float period; // Período da onda (tempo de um ciclo completo)
    unsigned long interval; // Intervalo entre as trocas de estado
    bool ledState; // Para controlar se o LED está ligado ou desligado

public:
    // Construtor: inicializa os pinos
    LedRGB(int r, int g, int b) {
        pinRed = r;
        pinGreen = g;
        pinBlue = b;
        
        pinMode(pinRed, OUTPUT);
        pinMode(pinGreen, OUTPUT);
        pinMode(pinBlue, OUTPUT);
        
        redValue = 0;
        greenValue = 0;
        blueValue = 0;

        redValueCurrent = 255;
        greenValueCurrent = 255;
        blueValueCurrent = 255;

        redValueWithIntensity = 255;
        greenValueWithIntensity = 255;
        blueValueWithIntensity = 255;

        previousMillis = 0;
        ledState = false;
    }

    // Função para definir a cor (valores de 0 a 255)
    void setColor(int r, int g, int b) {
      #ifdef COMMON_ANODE
        redValue =  (int)r;  // Subtrai de 255 para inverter os valores de cor
        greenValue =  (int)g;
        blueValue =  (int)b;
        #endif
        #ifdef COMMON_CATHODE
        redValue = (int)255 - r ;  // Subtrai de 255 para inverter os valores de cor
       greenValue = (int)255 - g ;
        blueValue = (int)255 - b ;
        #endif
    }

    //Configura intensidade dos leds 
    void setIntensity(float intensity)
    {
        #ifdef COMMON_ANODE
        redValueWithIntensity =  (int)redValue*intensity;  // Subtrai de 255 para inverter os valores de cor
        greenValueWithIntensity =  (int)greenValue*intensity;
        blueValueWithIntensity =  (int) blueValue*intensity;
        #endif
        #ifdef COMMON_CATHODE
        redValueWithIntensity = (int)255 - redValue + 255*(1-intensity);  // Subtrai de 255 para inverter os valores de cor
       greenValueWithIntensity = (int)255 - greenValue + 255*(1-intensity);
        blueValueWithIntensity = (int)255 - blueValue+255*(1-intensity);
        #endif
    }

    //altera a intensidade dos leds sem mudar cor RGB
    void setIntensityCurrent(float intensity)
    {
      #ifdef COMMON_ANODE
        redValueCurrent =  (int)redValueWithIntensity*intensity;  // Subtrai de 255 para inverter os valores de cor
        greenValueCurrent =  (int)greenValueWithIntensity*intensity;
        blueValueCurrent =  (int) blueValueWithIntensity*intensity;
        #endif
        #ifdef COMMON_CATHODE
        redValueCurrent = (int)255 - redValueWithIntensity + 255*(1-intensity);  // Subtrai de 255 para inverter os valores de cor
       greenValueCurrent = (int)255 - greenValueWithIntensity + 255*(1-intensity);
        blueValueCurrent = (int)255 - blueValueWithIntensity+255*(1-intensity);
        #endif
    }

    // Função para ligar o LED com a cor escolhida
    void turnOn() {
      
        // Ligar as cores com base nos valores definidos
        analogWrite(pinRed, redValueCurrent);   // Valor invertido para o hardware
        analogWrite(pinGreen, greenValueCurrent); // Valor invertido para o hardware
        analogWrite(pinBlue, blueValueCurrent);  // Valor invertido para o hardware
    }

    // Função para desligar o LED
    void turnOff() {
        // No caso do hardware invertido, nível alto apaga o LED
        #ifdef COMMON_ANODE
        analogWrite(pinRed, LOW);
        analogWrite(pinGreen, LOW);
        analogWrite(pinBlue, LOW);
        #endif
        #ifdef COMMON_CATHODE
        digitalWrite(pinRed, HIGH);
        digitalWrite(pinGreen, HIGH);
        digitalWrite(pinBlue, HIGH);
        #endif
    }
	void blinkSquareWaveWithDutyCycle(float freq, float dutyCycle) {
    frequency = freq;
    period = 1000.0 / frequency; // Período da onda em milissegundos
    unsigned long intervalOn = period * dutyCycle; // O tempo "on" é controlado pelo duty cycle
    unsigned long intervalOff = period - intervalOn; // O tempo "off" é o restante do período
    unsigned long currentMillis = millis();

    if (ledState) {  // Se o LED estiver ligado, verificar o tempo "on"
        if (currentMillis - previousMillis >= intervalOn) {
            previousMillis = currentMillis;
            ledState = !ledState; // Inverte o estado do LED
            turnOff(); // Desliga o LED (nível alto)
        }
    } else {  // Se o LED estiver desligado, verificar o tempo "off"
        if (currentMillis - previousMillis >= intervalOff) {
            previousMillis = currentMillis;
            ledState = !ledState; // Inverte o estado do LED
            turnOn(); // Liga o LED (nível baixo)
        }
    }
}
    // Função para piscar o LED com a onda quadrada
    void blinkSquareWave(float freq) {
        frequency = freq;
        period = 1000.0 / frequency; // Período da onda em milissegundos
        interval = period / 2; // O tempo de "on" e "off" é metade do período
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
          ledState = !ledState; // Inverte o estado do LED
            if (ledState) {
                turnOn();
            } else {
                turnOff(); // Desliga o LED (nível alto)
            }
        }
    }


void blinkSineWave(float freq) {
    period = 1000.0 / frequency; // Período da onda quadrada em milissegundos

    unsigned long currentMillis = millis();
    
    // Calcula o valor da função seno, que vai de -1 a 1
    // A variável 'freq' é a frequência da onda senoidal em Hz, usada para calcular a posição do ciclo
    float sineValue = sin(2 * PI * freq * (currentMillis / 1000.0)); 
    
    // Mapeia a função seno de [-1, 1] para [0, 1], para ser usado como duty cycle
    float dutyCycle = (sineValue + 1.0) / 2.0; 

    // Controla o brilho do LED usando o PWM com base no duty cycle gerado pela onda senoidal
    setIntensityCurrent(dutyCycle);
    turnOn();

}


void blinkTriangleWave(float freq) {
    period = 1000.0 / frequency; // Período da onda quadrada em milissegundos

    unsigned long currentMillis = millis();

    // Calcula a fase do ciclo em função da frequência e do tempo decorrido
    float cycleTime = 1000.0 / freq; // Tempo total de um ciclo da onda triangular (em ms)
    float elapsedTime = (currentMillis % (unsigned long)cycleTime); // Tempo dentro do ciclo atual
    float dutyCycle;

    // Se a fase estiver na subida (0 a 1)
    if (elapsedTime < cycleTime / 2) {
        dutyCycle = elapsedTime / (cycleTime / 2); // Aumento linear do duty cycle
    } else {
        // Se a fase estiver na descida (1 a 0)
        dutyCycle = 1 - ((elapsedTime - cycleTime / 2) / (cycleTime / 2)); // Diminuição linear do duty cycle
    }

    // Controla o brilho do LED usando o PWM com base no duty cycle gerado pela onda triangular
    setIntensityCurrent(dutyCycle);
    turnOn();
}


  
};

// Instancia o LED com os pinos 2, 3 e 4 fora da classe
LedRGB led(12,13,14);

void lerPotenciometroIntensidade() {
    int leitura = analogRead(POT_INTENSITY);  
    led.setIntensity(map(leitura, 0, 4095, 0, 255));
}

// Função que lê um potenciômetro e gera valores RGB baseados na leitura
void calcularRGB() {
    int leitura = analogRead(POT_RGB);
    int r, b, g = 0;
    int valor = map(leitura, 0, 4095, 0, 765); // Mapeia de 0-4095 para 0-765 (total de 3 cores somadas)

    if (valor <= 255) {
        r = 255 - valor; g = valor; b = 0; // De vermelho para verde
    } else if (valor <= 510) {
        r = 0; g = 255 - (valor - 255); b = valor - 255; // De verde para azul
    } else {
        r = valor - 510; g = 0; b = 255 - (valor - 510); // De azul para vermelho
    }
    led.setColor(r,g,b);
}

void delayMicrosecondsESP(uint32_t us) {
    esp_rom_delay_us(us);
}

void delayMillisecondsESP(uint32_t ms) {
    esp_rom_delay_us(ms * 1000);  // Converte ms para µs
}


volatile float freqSinal1 = 0;
volatile float freqSinal2 = 0;

void medirFrequencia(void *pvParameters) {
    static int ultimaLeitura1 = 0, ultimaLeitura2 = 0;
    static unsigned long ultimoTempo1 = 0, ultimoTempo2 = 0;
    int leitura1, leitura2;
    unsigned long tempoAtual;
    int contador = 0;
    while (true) {
        leitura1 = analogRead(SINAL1_ADC);
        leitura2 = analogRead(SINAL2_ADC);
        tempoAtual = micros();

        // Normalizando o sinal (ponto médio do ADC como referência)
        int referenciaADC = 2048;  // 3.3V / 2 ≈ 2048 no ADC de 12 bits
        bool cruzouZero1 = (ultimaLeitura1 < referenciaADC && leitura1 >= referenciaADC);
        bool cruzouZero2 = (ultimaLeitura2 < referenciaADC && leitura2 >= referenciaADC);

        // Calcula frequência do Sinal 1
        if (cruzouZero1) {
            if (ultimoTempo1 > 0) {
                freqSinal1 = 1e6 / (tempoAtual - ultimoTempo1);
            }
            ultimoTempo1 = tempoAtual;
        }

        // Calcula frequência do Sinal 2
        if (cruzouZero2) {
            if (ultimoTempo2 > 0) {
                freqSinal2 = 1e6 / (tempoAtual - ultimoTempo2);
            }
            ultimoTempo2 = tempoAtual;
        }

        ultimaLeitura1 = leitura1;
        ultimaLeitura2 = leitura2;

        esp_rom_delay_us(100); // Delay de 100 µs

        contador++;
        if (contador >= 10000) { // A cada 10ms (~10000 iterações)
            vTaskDelay(pdMS_TO_TICKS(1)); // Libera a CPU por 1ms
            contador = 0;
        }

    }
}


// Máquina de estados controlada por botão
#define BOTAO_PIN 25
volatile int estado = 0;

void IRAM_ATTR mudarEstado() {
    static unsigned long ultimaPressao = 0;
    unsigned long tempoAtual = millis();
    
    if (tempoAtual - ultimaPressao > 200) {  // Debounce
        estado = (estado + 1) % 5;
        ultimaPressao = tempoAtual;
    }
}

void maquinaEstados(void *pvParameters) {
    pinMode(BOTAO_PIN, INPUT_PULLUP);
    attachInterrupt(BOTAO_PIN, mudarEstado, FALLING);

    while (true) {
      calcularRGB();
      lerPotenciometroIntensidade();

        switch (estado) {
            case 0:
                Serial.println("Leds desligados.");
                  led.turnOff();
                break;
            case 1:
                Serial.println("Leds ligados.");
                led.turnOn();
                break;

            case 2:
                Serial.println("Onda senoidal.");
                led.blinkSineWave(abs(freqSinal1 - freqSinal2)); // Frequência de 1 Hz para onda senoidal
                break;
            case 3:
                Serial.println("Onda quadrada.");
                led.blinkSquareWave(abs(freqSinal1 - freqSinal2)); // Frequência de 1 Hz para onda quadrada
                break;
            case 4:
                Serial.println("Onda triangular.");
                led.blinkTriangleWave(abs(freqSinal1 - freqSinal2)); // Frequência de 1 Hz para onda triangular
                break;
        }

        Serial.printf("Frequências: Sinal1 = %.2f Hz, Sinal2 = %.2f Hz\n", freqSinal1, freqSinal2);
        vTaskDelay(pdMS_TO_TICKS(10)); // Libera a CPU por 1ms
    }
}






void setup() {
  Serial.begin(115200);
  led.setColor(255,255,255); // Define a cor como verde
  led.setIntensity(1);
  xTaskCreatePinnedToCore(medirFrequencia, "LeituraFreq", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(maquinaEstados, "MaquinaEstados", 2048, NULL, 1, NULL, 1);
}

void loop() {
    // Escolha o tipo de onda desejada e a frequência de piscagem
    }
