#include <Arduino_FreeRTOS.h>
// ============================================================================
// DEFINIÇÕES E PARÂMETROS TÉCNICOS DA EVSE SHIELD
// ============================================================================

// Fator de Escala do Control Pilot (2.40):
// Justificativa/Cálculo: O sinal do CP oscila de +12V a -12V (variação de 24Vpp). 
// O circuito atenua e desloca a excursão para a faixa de 0V a 5V do ADC. 
// Multiplicar a leitura do ADC por 2.40 reverte essa proporção e recupera os 
// valores reais de tensão estipulados pela norma IEC 61851-1 (Estados A, B, C, D).
const float FATOR_ESCALA_CP          = 2.40;     

const float FATOR_DIVISOR_14V        = 11.0;     // Divisor de Tensão da Fonte Interna: Fator multiplicador do pino A3 (ADC_+14V) para monitorizar o barramento de +14V, baseado no divisor R7 (100k) e R8 (10k).
const float FONTE_14V_MINIMA         = 11.5;     // Limite mínimo operacional da fonte interna de +14V.
const float FONTE_14V_MAXIMA         = 15.5;     // Limite máximo operacional da fonte interna de +14V.

// --- Configurações da Rede Elétrica (VGRID) ---
const float TENSAO_NOMINAL_REDE      = 220.0;    // Tensão nominal de referência da rede elétrica.
const float PERCENTAGEM_TOLERANCIA   = 15.0;     // Margem de tolerância percentual aceitável para subtensão e sobretensão.

// Fator do Transformador de Tensão (8.89):
// Justificativa/Cálculo: Coeficiente de proporção fixado pelo projeto do circuito 
// para projetar a amostragem de baixa tensão da malha secundária de volta aos 
// valores reais da rede alternada de alta tensão (220V RMS / pico de ~311.13V).
const float FATOR_TRANSFORMADOR      = 8.89; 

const float RELACAO_TC_IFASE         = 1000.0;   // Relação do TC de Fase: Proporção de espiras (1000:1) indicando quantos amperes primários correspondem a 1 ampère induzido no secundário.
const float RELACAO_TC_IDIFF         = 1000.0;   // Relação do TC Diferencial: Proporção de espiras (1000:1) para conversão da corrente de fuga.

// Resistor Burden Equivalente (448.9 Ohms):
// Justificativa/Cálculo: Calculado através da associação em paralelo do resistor 
// fixo de 470 Ohms com o trimpot de ajuste de 10k Ohms (R_burden = (470 * 10000) / (470 + 10000)).
const float R_BURDEN                 = 448.9;  

const float LIMITE_FUGA_MA           = 30.0;     // Limite de Segurança RCD (Fuga à Terra): Limiar máximo de 30mA estipulado por normas internacionais (IEC 62752 / IEC 61851) para proteção contra choques.

// Limites calculados de Subtensão e Sobretensão da rede
const float TENSAO_MINIMA_ACEITAVEL  = TENSAO_NOMINAL_REDE * (1.0 - (PERCENTAGEM_TOLERANCIA / 100.0));
const float TENSAO_MAXIMA_ACEITAVEL  = TENSAO_NOMINAL_REDE * (1.0 + (PERCENTAGEM_TOLERANCIA / 100.0));

// ============================================================================
// DEFINIÇÃO OFICIAL DOS PINOS ANALÓGICOS (CONECTOR DA SHIELD)
// ============================================================================
const uint8_t PINO_ADC_VGRID        = A0; 
const uint8_t PINO_ADC_IFASE        = A1; 
const uint8_t PINO_ADC_IDIFF        = A2; 
const uint8_t PINO_ADC_14V          = A3; 
const uint8_t PINO_ADC_CP           = A5; 
const uint8_t PINO_ADC_PP           = A6; 
const uint8_t PINO_ADC_OFFSET_VGRID = A7; 
const uint8_t PINO_ADC_OFFSET_IFASE = A8; 
const uint8_t PINO_ADC_REF_2V495    = A9; 

const float TENSAO_REFERENCIA   = 5.00;
const float PASSOS_ADC          = 1023.0;
const uint8_t NUMERO_AMOSTRAS   = 8; 

// ============================================================================
// ESTRUTURAS DE DADOS DAS LEITURAS ANALÓGICAS
// ============================================================================
struct Dados_EVSE {
  float pp_tensao;
  uint8_t pp_corrente_max;
  String pp_estado;

  float cp_tensao_adc;
  float cp_tensao_simulada;
  char cp_letra;
  String cp_estado;

  float vgrid_offset;
  float vgrid_adc;
  float vgrid_vrms;
  String vgrid_estado;

  float ifase_offset;
  float ifase_adc;
  float ifase_irms;
  String ifase_estado;

  float idiff_offset_2v495;
  float idiff_adc;
  float idiff_fuga_ma;
  String idiff_estado;

  float fonte_adc;
  float fonte_real;
  String fonte_estado;
};

Dados_EVSE evse;

// Declaração da Task "Leitura"
void vTaskLeitura(void *pvParameters);

// ----------------------------------------------------------------------------
// Função Auxiliar: Leitura com Filtro de Média Móvel (Anti-Ruído)
// ----------------------------------------------------------------------------
float lerAnalogicoFiltrado(uint8_t pino) {
  uint32_t soma = 0;
  for (uint8_t i = 0; i < NUMERO_AMOSTRAS; i++) {
    soma += analogRead(pino);
    delayMicroseconds(100);
  }
  float media_adc = (float)soma / NUMERO_AMOSTRAS;
  return (media_adc * TENSAO_REFERENCIA) / PASSOS_ADC;
}

// ----------------------------------------------------------------------------
// IMPLEMENTAÇÃO DA TASK "Leitura" NO FREERTOS
// ----------------------------------------------------------------------------
void vTaskLeitura(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // Executa a cada 100ms fixos

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // 1. Leitura do Proximity Pilot (PP)
    evse.pp_tensao = lerAnalogicoFiltrado(PINO_ADC_PP);
    if (evse.pp_tensao > 4.50) {
      evse.pp_corrente_max = 0;
      evse.pp_estado = "Sem Cabo";
    } else if (evse.pp_tensao > 3.00) {
      evse.pp_corrente_max = 13;
      evse.pp_estado = "Cabo 13A";
    } else if (evse.pp_tensao > 2.00) {
      evse.pp_corrente_max = 20;
      evse.pp_estado = "Cabo 20A";
    } else if (evse.pp_tensao > 1.00) {
      evse.pp_corrente_max = 32;
      evse.pp_estado = "Cabo 32A";
    } else {
      evse.pp_corrente_max = 63;
      evse.pp_estado = "Cabo 63A";
    }

    // 2. Leitura do Control Pilot (CP / ADC_VA)
    evse.cp_tensao_adc = lerAnalogicoFiltrado(PINO_ADC_CP);
    evse.cp_tensao_simulada = evse.cp_tensao_adc * FATOR_ESCALA_CP;
    if (evse.cp_tensao_simulada >= 10.50) {
      evse.cp_letra = 'A';
      evse.cp_estado = "Estado A";
    } else if (evse.cp_tensao_simulada >= 7.50) {
      evse.cp_letra = 'B';
      evse.cp_estado = "Estado B";
    } else if (evse.cp_tensao_simulada >= 4.50) {
      evse.cp_letra = 'C';
      evse.cp_estado = "Estado C";
    } else if (evse.cp_tensao_simulada >= 1.50) {
      evse.cp_letra = 'D';
      evse.cp_estado = "Estado D";
    } else {
      evse.cp_letra = 'F';
      evse.cp_estado = "Estado F";
    }

    // 3. Leitura da Tensão da Rede (VGRID)
    evse.vgrid_offset = lerAnalogicoFiltrado(PINO_ADC_OFFSET_VGRID);
    evse.vgrid_adc = lerAnalogicoFiltrado(PINO_ADC_VGRID);
    float tensao_deslocada_v = evse.vgrid_adc - evse.vgrid_offset; 
    float vtr_ac_inst = tensao_deslocada_v * 14.0;
    float tensao_pico_rede = abs(vtr_ac_inst) * FATOR_TRANSFORMADOR;
    evse.vgrid_vrms = tensao_pico_rede / 1.4142;

    if (evse.vgrid_vrms < TENSAO_MINIMA_ACEITAVEL) {
      evse.vgrid_estado = "SUBTENSÃO";
    } else if (evse.vgrid_vrms > TENSAO_MAXIMA_ACEITAVEL) {
      evse.vgrid_estado = "SOBRETENSÃO";
    } else {
      evse.vgrid_estado = "REDE OK";
    }

    // 4. Leitura da Corrente de Fase (IFASE)
    evse.ifase_offset = lerAnalogicoFiltrado(PINO_ADC_OFFSET_IFASE);
    evse.ifase_adc = lerAnalogicoFiltrado(PINO_ADC_IFASE);
    float tensao_deslocada_i = evse.ifase_adc - evse.ifase_offset; 
    float corrente_pico = (abs(tensao_deslocada_i) / R_BURDEN) * RELACAO_TC_IFASE;
    evse.ifase_irms = corrente_pico / 1.4142;
    float limite_corrente = (evse.pp_corrente_max > 0) ? evse.pp_corrente_max : 32.0;

    if (evse.ifase_irms > limite_corrente) {
      evse.ifase_estado = "SOBRECORRENTE";
    } else {
      evse.ifase_estado = "CORRENTE OK";
    }

    // 5. Leitura de Fuga à Terra (IDIFF - com A9)
    evse.idiff_offset_2v495 = lerAnalogicoFiltrado(PINO_ADC_REF_2V495);
    evse.idiff_adc = lerAnalogicoFiltrado(PINO_ADC_IDIFF);
    float tensao_deslocada_diff = evse.idiff_adc - evse.idiff_offset_2v495; 
    float corrente_pico_a = (abs(tensao_deslocada_diff) / R_BURDEN) * RELACAO_TC_IDIFF;
    evse.idiff_fuga_ma = (corrente_pico_a / 1.4142) * 1000.0;

    if (evse.idiff_fuga_ma > LIMITE_FUGA_MA) {
      evse.idiff_estado = "FALHA: FUGA TERRA";
    } else {
      evse.idiff_estado = "RCD OK";
    }

    // 6. Diagnóstico da Fonte Interna +14V
    evse.fonte_adc = lerAnalogicoFiltrado(PINO_ADC_14V);
    evse.fonte_real = evse.fonte_adc * FATOR_DIVISOR_14V;
    if (evse.fonte_real < FONTE_14V_MINIMA) {
      evse.fonte_estado = "FONTE BAIXA";
    } else if (evse.fonte_real > FONTE_14V_MAXIMA) {
      evse.fonte_estado = "FONTE ALTA";
    } else {
      evse.fonte_estado = "FONTE OK";
    }

    // 7. Impressão Consolidada no Monitor Serial
    Serial.print("[TASK Leitura] [PP]: "); Serial.print(evse.pp_estado);
    Serial.print(" | [CP]: "); Serial.print(evse.cp_tensao_simulada, 1); Serial.print("V ("); Serial.print(evse.cp_letra);
    Serial.print(") | [REDE]: "); Serial.print(evse.vgrid_vrms, 1); Serial.print("V ["); Serial.print(evse.vgrid_estado);
    Serial.print("] | [FUGA]: "); Serial.print(evse.idiff_fuga_ma, 1); Serial.print("mA ["); Serial.print(evse.idiff_estado);
    Serial.print("] | [+14V]: "); Serial.print(evse.fonte_real, 1); Serial.print("V ["); Serial.print(evse.fonte_estado);
    Serial.println("]");

    // Aguarda o próximo ciclo exato da Task Leitura
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("==================================================================");
  Serial.println("     EVSE SHIELD - TASK 'Leitura' CONFIGURADA NO FREERTOS          ");
  Serial.println("==================================================================");

  // Instanciação da Task "Leitura"
  xTaskCreate(
    vTaskLeitura,   // Ponteiro para a função da Task
    "Task_Leitura", // Nome descritivo da Task
    256,            // Tamanho da pilha (stack size)
    NULL,           // Parâmetro
    1,              // Prioridade (a definir posteriormente)
    NULL            // Handle
  );
}

void loop() {
  // Vazio, gerido pelo FreeRTOS.
}
