
//#define COMMON_CATHODE

#define COMMON_ANODE

class LedRGB {
private:
    int pinRed, pinGreen, pinBlue;
    int redValue, greenValue, blueValue;
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
        previousMillis = 0;
        ledState = false;
    }

    // Função para definir a cor (valores de 0 a 255)
    void setColor(int r, int g, int b, float intensity) {
      #ifdef COMMON_ANODE
        redValue =  (int)r*intensity;  // Subtrai de 255 para inverter os valores de cor
        greenValue =  (int)g*intensity;
        blueValue =  (int)b*intensity;
        #endif
        #ifdef COMMON_CATHODE
        redValue = (int)255 - r + 255*(1-intensity);  // Subtrai de 255 para inverter os valores de cor
       greenValue = (int)255 - g + 255*(1-intensity);
        blueValue = (int)255 - b+255*(1-intensity);
        #endif
    }

    // Função para ligar o LED com a cor escolhida
    void turnOn() {
      
        // Ligar as cores com base nos valores definidos
        analogWrite(pinRed, redValue);   // Valor invertido para o hardware
        analogWrite(pinGreen, greenValue); // Valor invertido para o hardware
        analogWrite(pinBlue, blueValue);  // Valor invertido para o hardware
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
    frequency = 100;  // Frequência de 500Hz para o PWM
    period = 1000.0 / frequency; // Período da onda quadrada em milissegundos

    unsigned long currentMillis = millis();
    
    // Calcula o valor da função seno, que vai de -1 a 1
    // A variável 'freq' é a frequência da onda senoidal em Hz, usada para calcular a posição do ciclo
    float sineValue = sin(2 * PI * freq * (currentMillis / 1000.0)); 
    
    // Mapeia a função seno de [-1, 1] para [0, 1], para ser usado como duty cycle
    float dutyCycle = (sineValue + 1.0) / 2.0; 

    // Controla o brilho do LED usando o PWM com base no duty cycle gerado pela onda senoidal
    blinkSquareWaveWithDutyCycle(frequency, dutyCycle);  // Usa o PWM com o duty cycle senoidal
}


void blinkTriangleWave(float freq) {
    frequency = 100;  // Frequência do PWM
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
    blinkSquareWaveWithDutyCycle(frequency, dutyCycle);  // Usa o PWM com o duty cycle da onda triangular
}


  
};

// Instancia o LED com os pinos 2, 3 e 4 fora da classe
LedRGB led(4,2,35);

void setup() {
    led.setColor(0,0, 255,0.01); // Define a cor como verde
}

void loop() {
    // Escolha o tipo de onda desejada e a frequência de piscagem
    
    //led.turnOn();
    //delay(1000);
    //led.turnOff();
    //delay(1000);
   led.blinkSquareWave(2); // Frequência de 1 Hz para onda quadrada
   //led.blinkTriangleWave(1); // Frequência de 1 Hz para onda triangular
  //led.blinkSineWave(1); // Frequência de 1 Hz para onda senoidal


}
