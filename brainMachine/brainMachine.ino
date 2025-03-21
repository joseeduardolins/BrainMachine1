#include "esp_timer.h"
#include "driver/timer.h"


#define SINAL1_ADC 5  // GPIO do primeiro sinal (ADC1_CH6)
#define SINAL2_ADC 18  // GPIO do segundo sinal (ADC1_CH7)
#define POT_INTENSITY 32
#define POT_RGB 33
#define BOTAO_PIN 25


#define COMMON_CATHODE

//#define COMMON_ANODE



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

        redValueCurrent = 0;
        greenValueCurrent = 0;
        blueValueCurrent = 0;

        redValueWithIntensity = 0;
        greenValueWithIntensity = 0;
        blueValueWithIntensity = 0;

        previousMillis = 0;
        ledState = false;
    }

    // Função para definir a cor (valores de 0 a 255)
    void setColor(int r, int g, int b) {
      /*
      #ifdef COMMON_ANODE
        redValue =  (int)r;  //  valores de cor
        greenValue =  (int)g;
        blueValue =  (int)b;
        #endif
        #ifdef COMMON_CATHODE
        redValue = (int)255 - r ;  // Subtrai de 255 para inverter os valores de cor
        greenValue = (int)255 - g ;
        blueValue = (int)255 - b ;
        #endif
        */
        redValue =  (int)r;  //  valores de cor
        greenValue =  (int)g;
        blueValue =  (int)b;

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
        redValueWithIntensity = (int)(255-redValue*intensity);  // Subtrai de 255 para inverter os valores de cor
        greenValueWithIntensity = (int)( 255-greenValue*intensity);
        blueValueWithIntensity = (int)(255-blueValue*intensity);
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
        redValueCurrent = (int)( 255-(redValueWithIntensity-255)*(-1)*intensity);  // Subtrai de 255 para inverter os valores de cor
       greenValueCurrent = (int)(255- (greenValueWithIntensity-255)*(-1)*intensity);
        blueValueCurrent = (int)( 255- (blueValueWithIntensity-255)*(-1)*intensity);
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
        analogWrite(pinRed, 255);
        analogWrite(pinGreen, 255);
        analogWrite(pinBlue, 255);
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
        setIntensityCurrent(1);

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
LedRGB led(14,12,13);
/*
void lerPotenciometroIntensidade() {
    delayMicroseconds(100);  // Pequeno atraso antes da leitura
    int leitura = analogRead(POT_INTENSITY); 
    Serial.printf("PotInt = %f\n",round(leitura / 4095.0 * 10) / 10.0);
    led.setIntensity(round(leitura / 4095.0 * 10) / 10.0);

}*/

void lerPotenciometroIntensidade() {
    static float ultimoValor = 0.0;  // Mantém o último valor válido
    
    float leitura = analogRead(POT_INTENSITY);
    float intensidade = round(leitura / 4095.0 * 100) / 100.0;

    if (intensidade < 0.01 ) { // Se a leitura real for >10, não deveria ser 0
        intensidade = ultimoValor;  // Mantém o último valor válido
    } else {
        ultimoValor = intensidade;  // Atualiza o último valor
    }

    Serial.printf("PotInt = %.1f\n", intensidade);
    led.setIntensity(intensidade);
}

// Função que lê um potenciômetro e gera valores RGB baseados na leitura
void calcularRGB() {
    static int ultimoValor = 0;  // Mantém o último valor válido
    int leitura = analogRead(POT_RGB);

    Serial.printf("PotRGB = %d\n", leitura);

    // Mapeia de 0-4095 para 0-765 (total de transição RGB)
    int valor = map(leitura, 0, 4095, 0, 765);

    // Se a leitura for muito baixa, mantém o último valor válido (evita glitches)
    if (leitura < 1) { 
        valor = ultimoValor;
    } else {
        ultimoValor = valor; // Atualiza o último valor
    }

    int r = 0, g = 0, b = 0;

    if (valor <= 255) {
        r = 255 - valor;  
        g = valor;  
        b = 0;  // Vermelho → Verde
    } else if (valor <= 510) {
        r = 0;  
        g = 255 - (valor - 255);  
        b = (valor - 255);  // Verde → Azul
    } else {
        r = (valor - 510);  
        g = 0;  
        b = 255 - (valor - 510);  // Azul → Vermelho
    }

    // Garantia de que os valores de cor não ultrapassem 0-255
    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);

    Serial.printf("RGB: R=%d, G=%d, B=%d\n", r, g, b);
    led.setColor(r, g, b);
}

volatile unsigned long ultimoTempo1 = 0;
volatile unsigned long ultimoTempo2 = 0;
volatile unsigned long periodo1 = 0;
volatile unsigned long periodo2 = 0;
volatile float freqSinal1 = 0;
volatile float freqSinal2 = 0;

// Interrupção para o primeiro sinal
void IRAM_ATTR medirFrequencia1() {
    unsigned long tempoAgora = micros();
    if (ultimoTempo1 != 0) {
        periodo1 = tempoAgora - ultimoTempo1;
        freqSinal1 = 1e6 / (float)periodo1; // Frequência em Hz
        Serial.println(freqSinal1);
    }
    ultimoTempo1 = tempoAgora;
}

// Interrupção para o segundo sinal
void IRAM_ATTR medirFrequencia2() {
    unsigned long tempoAgora = micros();
    if (ultimoTempo2 != 0) {
        periodo2 = tempoAgora - ultimoTempo2;
        freqSinal2 = 1e6 / (float)periodo2; // Frequência em Hz
        Serial.println(freqSinal2);
    }
    ultimoTempo2 = tempoAgora;
}



// Máquina de estados controlada por botão
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

    int tempoParaMedirPot = 0;
    float freqPulse = 0.0;
    attachInterrupt(BOTAO_PIN, mudarEstado, FALLING);
   // attachInterrupt(SINAL1_ADC, medirFrequencia1, RISING);
   // attachInterrupt(SINAL2_ADC, medirFrequencia2, RISING);
    while (true) {
      
      if((millis() - tempoParaMedirPot)>50)
      {
      calcularRGB();
      lerPotenciometroIntensidade();
      tempoParaMedirPot = millis();
      }


      freqPulse = 1.0;


        switch (estado) {
            case 0:
                Serial.println("Leds desligados.");
                  led.turnOff();
                break;
            case 1:

                Serial.println("Leds ligados.");
                led.setIntensityCurrent(1);
                led.turnOn();
                break;

            case 2:
                Serial.println("Onda senoidal.");
                led.blinkSineWave(abs(freqPulse)); // Frequência de 1 Hz para onda senoidal
                break;
            case 3:
                Serial.println("Onda quadrada.");
                led.blinkSquareWave(abs(freqPulse)); // Frequência de 1 Hz para onda quadrada
                break;
            case 4:
                Serial.println("Onda triangular.");
                led.blinkTriangleWave(abs(freqPulse)); // Frequência de 1 Hz para onda triangular
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Libera a CPU por 1ms
    }
}






void setup() {
  Serial.begin(115200);
  led.setColor(255,255,255); // Define a cor
  led.setIntensity(1);
  pinMode(SINAL1_ADC,  INPUT);
  pinMode(SINAL2_ADC,  INPUT);
  pinMode(BOTAO_PIN, INPUT_PULLUP);

  // Configura interrupções para borda de subida                                 

  //xTaskCreatePinnedToCore(medirFrequencia, "LeituraFreq", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(maquinaEstados, "MaquinaEstados", 4096, NULL, 1, NULL, 0);
}

void loop() {

    }
