/*
 *       Main: TMS320F28335
 *             (c) Alex Reis
 *
 *###########################################################################
 *
 * FILE:    Main.c
 *
 * TITLE:   DSP28335ControlCARD;
 *          Sistema de controle para inversor
 *
 *#################################################################################
 *  Ver | dd mmm yyyy | Who  | Description of changes
 * =====|=============|======|=====================================================
 *  0.1 | 01 Out 2016 | A.R. | Verão inicial do algoritmo: Testes das placas de
 *      |             |      | aquisição de sinal
 *---------------------------------------------------------------------------------
 *#################################################################################
*/

#include "DSP2833x_Device.h"

#include "DSP28x_Project.h"

// If not installed, install the "Control Suite".
// Navigate to "C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\include" and include the file "DSP28x_Project.h".

#include "Math.h"

// Diretivas de definição
#define ADC_CKPS 0x01
#define AMOSTRAS 16                      // Numero de amostras de tensão armazenadas internamente               //// -----> NO NOSSO CASO, SERÃO TAMBÉM 16? DOS DOIS LADOS?
#define PERI_PWM 1000000    //0.0004    // Periodo de chaveamento do inversor, considerando uma frequencia de 2500 Hz (0,4 ms)
#define PERI_AMOSTRAGEM 0.0001          // Periodo de amostragem do controle, considerando uma frequencia de 10 kHz (0,1 ms)
#define V_REF_CC 600.0

// Funções preexistentes nas bibliotecas da TI para configuração da CPU
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void InitAdc(void);
//extern void ConfigCpuTimer(struct CPUTIMER_VARS*, float, float);


// Prototype statements for functions found within this file.                                                                         //// -----> AQUI PROVAVELMENTE CRIAREMOS UMA "void smo" EM RET.
//-----------------------------------------------------------
void Gpio_select(void);                             // Configuração das saídas digitais
interrupt void cpu_timer0_isr(void);                // Função associada à interrupção vinculada ao Timer 0
interrupt void adc_isr(void);                       // Função associada à interrupção vinculada ao ADC
//void SpaceVectorPWM(void);                        // Função para determinação dos pulsos PWM
void DSOGI_FLL(float Va, float Vb, float Vc);       // Função para o DSOGI-FLL
void Data_Estimation(void); // Função que determina o valor RMS de tensão e correntes lidos
void Setup_ePWM(void); // Função que configura os canais ePWM1, ePWM2 e ePWM3 para gerar os pulsos de chaveamento
void Change_ePWM(void); // Função que altera o duty cycle dos pulsos de chaveamento conforme as ondas modulantes
void Malha_Controle(void); // Função que realiza as transformações ABC para DQO e também calcula os valores de Ev e Eq baseados nos controladores de malha interna e externa


// Variaveis globais utilizadas no programa
// #############################################################################

// Váriáveis associadas à placa de aquisição de sinais
// ---------------------------------------------------

float PI = 3.1415926535;  // Definição do valor de pi
float V_base = 380.0;      // Valor da tensão de linha utilizada como base para os cálculos
float S_base =  10000.0; //5000;        // Valor da potência base (trifásica)
float I_base = 15.193;//  Fórmula utilizada -> I_bse = S_base/(1.7320508*V_base) //15.193428;
float Z_base =  14.44;// Fórmula utilizada -> Z_base = (V_base*V_base)/S_base // 14.44;

/*
 * As variáveis abaixo referem-se ao ganho que deve ser atribuído as placas de condionamento de sinais.
 * KV_PCI: Ganho da placa de condicionamento de sinal, considerando as alterações provocadas pelo transdutor de
 * tensão e pelo filtro anti-aliasing. A utilização deste ganho permite transformar o sinal de saída da
 * PCI (Vsaida) no sinal de entrada (Ventrada ou Ientrada). As transformações seguem as seguintes proporções:
 * 1. Transdutor de tensão: para uma entrada de 220 V, tem-se 2,5 V na sua saída
 * 2. Transdutor de corrente: para uma entrada de 50 A, tem-se 5,0 V na sua saída
 * 3. Filtro anti-aliasing: para evitar a saturação do sinal, o filtro atenua os sinais em 0.303030303030
 *
 * Logo:            2.5 * 0,3030303030 * Ventrada                               220
 *        Vsaida =  -----------------------------       ->  KV_PCI = -------------------- = 167,64
 *                              220                         0,3030303030 * 2,5
 *                  5,0 * 0,3030303030 * Ientrada                              50
 *        Vsaida =  -----------------------------       ->  KI_PCI = -------------------- = 33,0
 *                              50                          0,3030303030 * 5,0
 */
float KV_PCI = 264.75;//290.40;//290.40; //13.2; // 290.40; // 167.64; --> Ganho da placa de condicionamento, referente ao sensor de tensão
float KV_PCI_A = 252.27;
float KV_PCI_B = 257.33;
float KV_PCI_C = 261.96;
float KV_PCI_CC = 289.51;//304.92; --> Ganho da placa de condionamento, referente ao sensor de tensão do barramento CC
float KI_PCI = 0.737*(33.0/4.0); // --> Ganho do sensor de corrente. É realizado uma divisão por "4.0" pois o fio que passa pelo sensor de corrente está dando 4 voltas.
                               // Logo, é como se estivessemos calculando uma corrente 4x maior que a verdadeira.

/*
 * Constante utilizada para transformar o valor de saida do conversor A/D no valor de entrada.  A mesma segue
 * a resolução do conversor A/D do DSP (3,3 V; 12 bits - 4095)
 * Kad = 3,3/4095 = 0.000805860806                                                            //// -----> OK, OS VALORES SÃO OS MESMOS?
 */
float Kad = 0.000805860806;

// Variáveis para armazenamento da saída do conversor AD
// -----------------------------------------------------
int AD_output1 = 0, AD_output2 = 0, AD_output3 = 0, AD_output4 = 0, AD_output5 = 0, AD_output6 = 0, AD_output7 = 0;           //// -----> OK, CONTINUAREMOS COM 7 SAÍDAS? E DO LADO DO RET.?

// Vetores para armazenamento dos dados amostrados - valore reais                                     //// -----> OK PARA INV., MAS E RET.?
// --------------------------------------------------------------
float Corrente_A[AMOSTRAS]  = {0};
float Corrente_B[AMOSTRAS]  = {0};
float Corrente_C[AMOSTRAS]  = {0};
float Tensao_A[AMOSTRAS]    = {0};
float Tensao_B[AMOSTRAS]    = {0};
float Tensao_C[AMOSTRAS]    = {0};
float Vcc[AMOSTRAS]     = {0};

// Variaveis para as tensoes de entrada filtradas

float Tensao_Af[2] = {0,0};
float Tensao_Bf[2] = {0,0};
float Tensao_Cf[2] = {0,0};

// Variaveis para as correntes de entrada filtradas

float Corrente_Af[2] = {0,0};
float Corrente_Bf[2] = {0,0};
float Corrente_Cf[2] = {0,0};

// Variaveis para verificação dos valores RMS das tensões e correntes de entrada

float Tensao_A_RMS = 0, Tensao_B_RMS = 0, Tensao_C_RMS = 0;
float Soma_Tensao_A = 0, Soma_Tensao_B = 0, Soma_Tensao_C = 0;
float Corrente_A_RMS = 0, Corrente_B_RMS = 0, Corrente_C_RMS = 0;
float Soma_Corrente_A = 0, Soma_Corrente_B = 0, Soma_Corrente_C = 0;
float Contador_Amostras = 0;

// Variaveis para implementação do DSOGI-FLL
const float k = 1.41424356, Tau = 266.579;// 50.0;
float V_alfa_inter[2] = {0}, V_alfa_out[2] = {0}, V_alfa_int[2] = {0};
float V_beta_inter[2] = {0}, V_beta_out[2] = {0}, V_beta_int[2] = {0};
float V_alfa_P = 0, V_alfa_N = 0, V_beta_P = 0, V_beta_N = 0;
float FLL_int[2] = {0}, FLL_Freq[2] = {0}, Freq_in = 0, Freq_Rede = 376.99111841;
float Teta = 0;

// Variaveis e funcao para verificacao da PLL

float Vpll = 0;
void Teste_PLL(void);

// Variavel utilizada para verificar a primeira passagem pelo codigo do algoritmo da PLL
unsigned int first_time = 0;

// Contador utilizado para aguardar o sincronismo da PLL com a rede

unsigned int counter_PLL = 0;

// Variáveis para implementação do controle vetorial (sistema síncrono (DQ0) com as tensões no PAC)
float Vd_med[2] = {0, 0}, Vq_med[2] = {0, 0}, Vd_medf[2] = {0, 0}, Vq_medf[2] = {0, 0};
float Id_med[2] = {0, 0}, Iq_med[2] = {0, 0}, Id_medf[2] = {0, 0}, Iq_medf[2] = {0, 0};

// Indutância do filtro de saída, em H
float L_Filt = 0.003; // O filtro que está sendo utilizado é de 3 mH

// Indutância do filtro de saída, em pu, para cálculo dos termos de acoplamento entre malhas de controle
float L_pu = 0.07832225;//(376.99111841*L_Filt)/(Z_base) //0.078624099722992; //+ 0.0003; // O valor 0.0003 refere-se a impedância do transformador em pu

// Valores de tensão no capacitor

float V_CAP[2] = {0,0}, V_CAPF[2] = {0,0};

// Valores em pu das tensões medidas

float Vd_pu = 0, Vq_pu = 0;

// Valores em pu das correntes medidas

float Id_pu = 0, Iq_pu = 0;

// Valores das potências trifásicas no sistemas dq0, em pu (utilizando dados de nova transformação vetorial)
float P_med[2] = {0,0}, Q_med[2] = {0,0};
float P_medf[2] = {0,0}, Q_medf[2] = {0,0};

// Valores de erro de potência
float Erro_P[2] = {0,0}, Erro_Q[2] = {0,0};

// Valores de referência para os controladores de corrente de eixo direito e em quadratura
float Id_ref[2] = {0, 0}, Iq_ref[2] = {0, 0};

// Ganhos dos controladores PI de Potência Ativa
/*
 * Ganhos para o capacitor de 6 mF
 * K1 = 0.7523, K2 = 0.7477
 * K3 = 0.739, K4 = 0.7269
 * K5 = 0.7358, K6 = 0.7242
 *
 * Ganhos para o capacitor de 940 uF
 * K1 = 0.1184, K2 = 0.1176
 * K3 = 0.5081, K4 = 0.4939
 * K5 = 0.8104, K6 = 0.7957
 *
 * Ganhos utilizados na tese do Leonardo
 * K1 = 0.8895, K2 = 0.8812;
 * K3 = 0.7693, K4 = 0.7611;
 * K5 = 0.8104, K6 = 0.7957;
 * */

float K1 = 0.8895, K2 = 0.8812;

// Ganhos dos controladores PI de Potência Reativa
float K3 = 0.7693, K4 = 0.7611;

// Ganhos dos controladores PI de Corrente
float K5 = 0.8104, K6 = 0.7611;

// Erro entre valores de referência e medidos, para correntes de eixo direto e em quadratura
float Erro_id[2] = {0, 0}, Erro_iq[2] = {0, 0};

// Ganhos dos controladores PI da malha de corrente de eixo direto e em quadratura
//float Kp_CC = 1, Ki_CC = 1;

// Valores de saída dos controladores PI das malhas de corrente de eixo direto e em quadratura
float Pi_id[2] = {0, 0}, Pi_iq[2] = {0, 0};

// Valores das tensões a serem sintetizadas nos terminais do inversor
float Ed_inv = 0, Eq_inv = 0, InvFaseA = 0, InvFaseB = 0, InvFaseC = 0;

// int Chaves[6] = {1, 1, 1, 0, 0, 0}; // Matriz para armazenamento das posições das chaves
                       // A mesma é utilizada para alterar o estado das saidas digitais.

//int Status = 0;                     // Habilita o novo cálculo do estado do inversor

unsigned long int counter_iref = 0;
unsigned long int counter_iref_sequence = 0;

unsigned int setup_EPWM_first_time = 0; // Variável que permite o setup do PWM somente uma primeira vez.



// Fim das variaveis globais utilizadas no programa
// #############################################################################





//###########################################################################
//                      main code
//###########################################################################
void main(void)
{
    // Basic Core Initialization
    InitSysCtrl();

    EALLOW;
    #if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
        #define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
    #endif
    #if (CPU_FRQ_100MHZ)
        #define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
    #endif
    EDIS;

    // Define ADCCLK clock frequency ( less than or equal to 25 MHz )
    // Assuming InitSysCtrl() has set SYSCLKOUT to 150 MHz
    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;

    //WD activation
    EALLOW;
    SysCtrlRegs.WDCR = 0x006F;
    SysCtrlRegs.SCSR = 0x00;
    EDIS;

    // Disable all interrupts
    DINT;

    // GPIO Configuration
    // ------------------
    Gpio_select();

    // -------------------

    // Interruption service initialization
    InitPieCtrl();
    InitPieVectTable();


    // ADC Initialization
    // #######################################

    // Inicialização básica do conversor A/D
    InitAdc();

    //AdcRegs.ADCTRL1.all = 0;

    //Sequencer mode: cascade mode
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;

    //Autosequencer para no final da sequencia de amostragem
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;

    //Tempo de aquisição
    //AdcRegs.ADCTRL1.bit.ACQ_PS = 3;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 5;

    //Janela de aquisição
    AdcRegs.ADCTRL1.bit.CPS = 0;

    // Definição do clock FCLK em 12.5 MHz
    AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;

    // Definição do numero de canais que o conversor A/D deve ler (n+1)
    AdcRegs.ADCMAXCONV.all = 0x6;

    //Canais de leitura
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;  //ADCINA0 -> Fase A - Tensão
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;  //ADCINA1 -> Fase B - Tensão ---> A PLL está em fase com este canal do A/D
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;  //ADCINA2 -> Fase C - Tensão
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;  //ADCINA3 -> Fase A - Corrente
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;  //ADCINA4 -> Fase B - Corrente
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;  //ADCINA5 -> Fase C - Corrente
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;  //ADCINA6 -> Vcc - Tensão

    // Conversor A/D é ativado por pulso PWM
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    // Ativação da interrupção associada ao termino da amostragem
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
    // Interrupções são executadas a cada final de sequencia do Autosequencer
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;

    // ##############################################

    // ePWM4 initialization
    // ##################################
    EPwm4Regs.TBCTL.all = 0xC030;       // Configure timer control register
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;     // CLKDIV = 1
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 1;  // HSPCLKDIV = 2
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;    // Contagem up
    EPwm4Regs.TBPRD = 7500;         // Frequencia do PWM = 10 kHz
    EPwm4Regs.ETPS.all = 0x0100;        // Configure ADC start by ePWM4
    EPwm4Regs.ETSEL.all = 0x0A00;       // Enable SOCA to ADC
    //#########################################


    // ########################################

    // Configuração das interrupções
    // #############################

    EALLOW;

    // Ativação da interrupção associada ao conversor A/D
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

    // Associação da função "adc_isr" à interrupção do conversor A/D
    PieVectTable.ADCINT = &adc_isr;

    // Ativação da interrupção associada  ao Timer 0
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Associação da função "cpu_timer0_isr" à interrupção do Timer0
    //PieVectTable.TINT0 = &cpu_timer0_isr;

    EDIS;

    // Inicialização dos timers
    //InitCpuTimers();

    // Configuração inicial do Timer 0: o mesmo é utilizado para a contagem do periodo de chaveamento
    //ConfigCpuTimer(&CpuTimer0, 150, PERI_PWM);

    // Ativação de interrupções
    IER |= 1;
    EINT;
    ERTM;

    //Início da contagem do tempo - Timer 0
    //CpuTimer0Regs.TCR.bit.TSS = 0;

    //#########################################

    while(1)
    {
        EALLOW;
        SysCtrlRegs.WDKEY = 0x55;
        SysCtrlRegs.WDKEY = 0xAA;
        EDIS;
    }

}

//###########################################################################
//                      End of main code
//###########################################################################

void Gpio_select(void)
{
    EALLOW;
    // Set all Mux to use GPIO
    GpioCtrlRegs.GPAMUX1.all = 0;   // GPIO15 ... GPIO0 = General Puropse I/O
    GpioCtrlRegs.GPAMUX2.all = 0;   // GPIO31 ... GPIO16 = General Purpose I/O
    GpioCtrlRegs.GPBMUX1.all = 0;   // GPIO47 ... GPIO32 = General Purpose I/O
    GpioCtrlRegs.GPBMUX2.all = 0;   // GPIO63 ... GPIO48 = General Purpose I/O
    GpioCtrlRegs.GPCMUX1.all = 0;   // GPIO79 ... GPIO64 = General Purpose I/O
    GpioCtrlRegs.GPCMUX2.all = 0;   // GPIO87 ... GPIO80 = General Purpose I/O

    // All GPIO are defined as input ports
    GpioCtrlRegs.GPADIR.all = 0;
    GpioCtrlRegs.GPBDIR.all = 0;
    GpioCtrlRegs.GPCDIR.all = 0;

    // peripheral explorer: LED LD3 at GPIO34
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

    // GPIO33 = GPIO para recebimento do status do disjuntor
    //GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;

    // GPIO de envio de sinal de fechamento das chaves superiores
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;     // GPIO32 = Fase A
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;     // GPIO30 = Fase B
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;     // GPIO28 = Fase C

    // GPIO de envio de sinal de fechamento das chaves inferiores
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     // GPIO32 = Fase A
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;     // GPIO30 = Fase B
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;     // GPIO28 = Fase C

    // GPIO para verificação do status da PLL
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; // GPIO10 = Teste da PLL

    EDIS;

    //Inicialização do estado das chaves - Todas estão inicialmente abertas
    GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;

    //Inicialização da GPIO da PLL - Pino inicialmente em 0

    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;

}

/*interrupt void cpu_timer0_isr(void)
{
     Esta função é utlizada para atualizar o estado das saidas digitais, as
     * quais se destinam a comandar as chaves do inversor.


    //Contador de execução da interrupção
    CpuTimer0.InterruptCount++;

    // Acendimento do LED: indicativo de passagem pelo código
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // Habilita o novo calculo do estado do inversor
    Status = 0;

    // Atualização do estado das chaves
    if (Chaves[0] == 1){
        GpioDataRegs.GPBSET.bit.GPIO32 = 1; // Fase A - Chave superior
        GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   // Fase A - Chave inferior
    }
    else{
        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;   // Fase A - Chave superior
        GpioDataRegs.GPASET.bit.GPIO31 = 1; // Fase A - Chave inferior
    }

    if (Chaves[1] == 1){
        GpioDataRegs.GPASET.bit.GPIO30 = 1; // Fase B - Chave superior
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;   // Fase B - Chave inferior
    }
    else{
        GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;   // Fase B - Chave superior
        GpioDataRegs.GPASET.bit.GPIO29 = 1; // Fase B - Chave inferior
    }

    if (Chaves[2] == 1){
        GpioDataRegs.GPASET.bit.GPIO28 = 1;     // Fase C - Chave superior
        GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;   // Fase C - Chave inferior
    }
    else{
        GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;   // Fase C - Chave superior
        GpioDataRegs.GPASET.bit.GPIO23 = 1; // Fase C - Chave inferior
    }


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    EALLOW;
    SysCtrlRegs.WDKEY = 0x55;
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;
}*/

interrupt void adc_isr(void)
{
    int jj;     // Variável temporária para contagem

    //Deslocamento dos valores do array para a esquerda
    for (jj=0;jj<(AMOSTRAS-1);jj++){
        Corrente_A[jj] = Corrente_A[jj+1];
        Corrente_B[jj] = Corrente_B[jj+1];
        Corrente_C[jj] = Corrente_C[jj+1];
        Tensao_A[jj] = Tensao_A[jj+1];
        Tensao_B[jj] = Tensao_B[jj+1];
        Tensao_C[jj] = Tensao_C[jj+1];
        Vcc[jj] = Vcc[jj + 1];
    }

    // Amostragem das tensões e correntes. O offset está de acordo com
    // o ajuste da placa de aquisição de dados
    AD_output1 = (AdcRegs.ADCRESULT0>>4) - 1862;
    AD_output2 = (AdcMirror.ADCRESULT1) - 1860; // <------ A PLL está em fase com este canal do A/D.
    AD_output3 = (AdcMirror.ADCRESULT2) - 1859;
    AD_output4 = (AdcMirror.ADCRESULT3) - 1860;
    AD_output5 = (AdcMirror.ADCRESULT4) - 1860;
    AD_output6 = (AdcMirror.ADCRESULT5) - 1860;
    AD_output7 = (AdcMirror.ADCRESULT6) - 1750; // 621


    // Conversão dos valores de saída do conversor A/D
    Tensao_A[AMOSTRAS-1]    = KV_PCI_A*Kad*(float)AD_output1;
    Tensao_B[AMOSTRAS-1]    = KV_PCI_B*Kad*(float)AD_output2;
    Tensao_C[AMOSTRAS-1]    = KV_PCI_C*Kad*(float)AD_output3;
    Corrente_A[AMOSTRAS-1]  = KI_PCI*Kad*(float)AD_output4;
    Corrente_B[AMOSTRAS-1]  = KI_PCI*Kad*(float)AD_output5;
    Corrente_C[AMOSTRAS-1]  = KI_PCI*Kad*(float)AD_output6;
    Vcc[AMOSTRAS-1] = KV_PCI_CC*Kad*(float)AD_output7;

    // Filtragem das tensões e correntes de entrada amostradas
    // O Valor da frequência de corte do filtro passa-baixas é 500 rad/s.

    Tensao_Af[1] = Tensao_Af[0];
    Tensao_Af[0] = 0.02439*(Tensao_A[AMOSTRAS-1] + Tensao_A[AMOSTRAS-2]) + 0.9512*Tensao_Af[1];

    Tensao_Bf[1] = Tensao_Bf[0];
    Tensao_Bf[0] = 0.02439*(Tensao_B[AMOSTRAS-1] + Tensao_B[AMOSTRAS-2]) + 0.9512*Tensao_Bf[1];

    Tensao_Cf[1] = Tensao_Cf[0];
    Tensao_Cf[0] = 0.02439*(Tensao_C[AMOSTRAS-1] + Tensao_C[AMOSTRAS-2]) + 0.9512*Tensao_Cf[1];

    Corrente_Af[1] = Corrente_Af[0];
    Corrente_Af[0] = 0.07024*(Corrente_A[AMOSTRAS-1] + Corrente_A[AMOSTRAS-2]) + 0.85953*Corrente_Af[1];

    Corrente_Bf[1] = Corrente_Bf[0];
    Corrente_Bf[0] = 0.07024*(Corrente_B[AMOSTRAS-1] + Corrente_B[AMOSTRAS-2]) + 0.85953*Corrente_Bf[1];

    Corrente_Cf[1] = Corrente_Cf[0];
    Corrente_Cf[0] = 0.07024*(Corrente_C[AMOSTRAS-1] + Corrente_C[AMOSTRAS-2]) + 0.85953*Corrente_Cf[1];


    // Calculo de tensão no capacitor

    V_CAP[1] = V_CAP[0];
    V_CAP[0] = Vcc[AMOSTRAS-1];

    V_CAPF[1] = V_CAPF[0];
    //V_CAPF[0] = 0.02439*(V_CAP[0] + V_CAP[1]) + 0.9512*V_CAPF[1];
    V_CAPF[0] = 0.00624*(V_CAP[0] + V_CAP[1]) + 0.98751*V_CAPF[1];

    // Testes do sistema de aquisição: Detecção de amplitudes e frequencia dos sinais do lado da linha

    Data_Estimation();

    // Execução do DSOGI-FLL

    DSOGI_FLL(Tensao_A[AMOSTRAS-1], Tensao_B[AMOSTRAS-1], Tensao_C[AMOSTRAS-1]);

    // Verificação do sincronismo da PLL utilizando um pino de GPIO
    Teste_PLL();

    // Implementação de um contador para esperar o a sincronização do algoritmo da PLL com as tensões da rede.

    if (counter_PLL > 100*167-1){

        // Inicialização do ePWM1, ePWM2 e ePWM3 (ocorre somente uma vez após o sincronismo da PLL)
        if(setup_EPWM_first_time == 0){
            Setup_ePWM();
            setup_EPWM_first_time++;
        }

        //Código do controle
        Malha_Controle();

        //InvFaseA = 0.98*cos(Teta);
        //InvFaseB = 0.98*cos(Teta - 2.094395102);
        //InvFaseC = 0.98*cos(Teta + 2.094395102);

        // Determinação dos pulsos de chaveamento
        Change_ePWM();
    }
    else{
        counter_PLL++;
    }

    // Reset ADC Sequencer1 (Register ADCCTRL2)
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;

    // Clear Interrupt Flag ADC Sequencer 1 (Register ADCST)
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

    // Acknowledge PIE Interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

/*void SpaceVectorPWM(void)
{


}
*/

void DSOGI_FLL(float Va, float Vb, float Vc)
{
/*
 * Esta função implementa o método de sincronização conhecido por DSOGI-FLL
 */

    float V_alfa = 0, V_beta = 0;
    float Ev_alfa = 0, kEv_alfa = 0, qV_alfa = 0;
    float Ev_beta = 0, kEv_beta = 0, qV_beta = 0;

    // Transformação vetorial (abc -> ab0)
    V_alfa = 0.66666667*(Va - 0.5*Vb - 0.5*Vc);
    V_beta = 0.66666667*(0.86602540*Vb - 0.86602540*Vc);

    // Adaptative filter (SOGI) - Componente alfa
    // ------------------------------------------
    V_alfa_inter[1]= V_alfa_inter[0];
    V_alfa_int[1] = V_alfa_int[0];
    V_alfa_out[1] = V_alfa_out[0];

    Ev_alfa = V_alfa - V_alfa_out[0];
    kEv_alfa = k*Ev_alfa;
    qV_alfa = V_alfa_int[0]*Freq_in;
    V_alfa_inter[0] = Freq_in*(kEv_alfa - qV_alfa);

    // Integração: V_alfa_inter -> V_alfa_out
    V_alfa_out[0] = (PERI_AMOSTRAGEM/2)*V_alfa_inter[0] + (PERI_AMOSTRAGEM/2)*V_alfa_inter[1] + V_alfa_out[1];

    // Integração: V_alfa_out -> V_alfa_int
    V_alfa_int[0] = (PERI_AMOSTRAGEM/2)*V_alfa_out[0] + (PERI_AMOSTRAGEM/2)*V_alfa_out[1] + V_alfa_int[1];


    // Adaptative filter (SOGI) - Componente beta
    // ------------------------------------------
    V_beta_inter[1]= V_beta_inter[0];
    V_beta_int[1] = V_beta_int[0];
    V_beta_out[1] = V_beta_out[0];

    Ev_beta = V_beta - V_beta_out[0];
    kEv_beta = k*Ev_beta;
    qV_beta = V_beta_int[0]*Freq_in;
    V_beta_inter[0] = Freq_in*(kEv_beta - qV_beta);

    // Integração: V_alfa_inter -> V_alfa_out
    V_beta_out[0] = (PERI_AMOSTRAGEM/2)*V_beta_inter[0] + (PERI_AMOSTRAGEM/2)*V_beta_inter[1] + V_beta_out[1];

    // Integração: V_alfa_out -> V_alfa_int
    V_beta_int[0] = (PERI_AMOSTRAGEM/2)*V_beta_out[0] + (PERI_AMOSTRAGEM/2)*V_beta_out[1] + V_beta_int[1];


    // Positive and negative sequence voltage calculation
    V_alfa_P = 0.5*V_alfa_out[0] - 0.5*qV_beta;
    V_alfa_N = 0.5*V_alfa_out[0] + 0.5*qV_beta;
    V_beta_P = 0.5*V_beta_out[0] + 0.5*qV_alfa;
    V_beta_N = 0.5*V_beta_out[0] - 0.5*qV_alfa;

    // FLL Normalization
    FLL_int[1] = FLL_int[0];
    if(first_time == 0)
    {
        FLL_int[0] = 0.0000000001;
        first_time++;
    }
    else{
        FLL_int[0] = ( k*Freq_in/( V_alfa_P*V_alfa_P + V_beta_P*V_beta_P ) )*( -Tau*(Ev_alfa*qV_alfa + Ev_beta*qV_beta) );
    }


    // Integração: FLL_int -> FLL_Freq
    FLL_Freq[1] = FLL_Freq[0];
    FLL_Freq[0] = (PERI_AMOSTRAGEM/2)*FLL_int[0] + (PERI_AMOSTRAGEM/2)*FLL_int[1] + FLL_Freq[1];

    Freq_in = FLL_Freq[0] + Freq_Rede;

    // Phase angle calculation: atan retorna valores situados entre -pi/2 e pi/2
    Teta = atan(V_beta_P/V_alfa_P);

    // Ajuste do ângulo TETA, para situar a variável entre 0 e 2*PI
    if (V_alfa_P < 0)
        Teta = Teta + PI;
    else if (V_alfa_P > 0 && V_beta_P <0)
        Teta = Teta + 2*PI;
}

//void Transformation_ABC_2_DQ0(float FaseA, float FaseB, float FaseC, float Teta)
//{
    /*
     * Esta função implementa a transformação de Park, realizando a transformação do sistema
     *  ABC para o sistema DQO. As equações que modelam esta transformação são:
     *      Vd = (2/3)*(Va*cos(Teta) + Vb*cos(Teta - 2*pi/3) + Vc*cos(Teta + 2*pi/3)
     *      Vq = (2/3)*(Va*sin(Teta) + Vb*sin(Teta - 2*pi/3) + Vc*sin(Teta + 2*pi/3)
     *      V0 = (2/3)*(0.5*Va + 0.5*Vb + 0.5*Vc)
     *
     * Variáveis de entrada da função:
     *          float FaseA: sinal instantâneo da Fase A;
     *          float FaseB: sinal instantâneo da Fase B;
     *          float FaseC: sinal instantâneo da Fase C;
     *          float Teta: ângulo de referência para a transformação.
     *
     *  Variáveis de saída da função:
     *
     */
//  float Daxis = 0, Qaxis = 0, Zeroaxis = 0;

//  Daxis = 0.66666667*( FaseA*cos(Teta) + FaseB*cos(Teta - 2.094395102) + FaseC*cos(Teta + 2.094395102) );
//  Qaxis = -0.66666667*( FaseA*sin(Teta) + FaseB*sin(Teta - 2.094395102) + FaseC*sin(Teta + 2.094395102) );
//  Zeroaxis = 0.66666667*( 0.5*FaseA + 0.5*FaseB + 0.5*FaseC );

//}

//void Transformation_DQ0_2_ABC(float Daxis, float Qaxis, float Zeroaxis, float Teta)
//{
    /*
     * Esta função implementa a transformação inversa de Park, realizando a transformação do sistema
     *  DQ0 para o sistema ABC. As equações que modelam esta transformação são:
     *      Va = (2/3)*(Vd*cos(Teta) + Vq*sin(Teta) + V0)
     *      Vb = (2/3)*(Vd*cos(Teta - 2*pi/3) + Vq*sin(Teta - 2*pi/3) + V0)
     *      Vc = (2/3)*(Vd*cos(Teta + 2*pi/3) + Vq*sin(Teta + 2*pi/3) + V0)
     *
     * Variáveis de entrada da função:
     *          float Daxis: componente de eixo direto
     *          float Qaxis: componente de eixo em quadratura
     *          float Zeroaxis: componente de sequência zero
     *          float Teta: ângulo de referência para a transformação.
     *
     *  Variáveis de saída da função:
     *
     */
//  float FaseA = 0, FaseB = 0, FaseC = 0;

//  FaseA = 0.66666667*(Daxis*cos(Teta) + Qaxis*sin(Teta) + Zeroaxis);
//  FaseB = 0.66666667*(Daxis*cos(Teta - 2.094395102) + Qaxis*sin(Teta - 2.094395102) + Zeroaxis);
//  FaseC = 0.66666667*(Daxis*cos(Teta + 2.094395102) + Qaxis*sin(Teta + 2.094395102) + Zeroaxis);

//}

void Teste_PLL(void){
    /*
     * Esta função verifica o status da PLL com relação a fase A da rede.
     * A verificação é realizada através da GPIO12 do DSP.
     * Quando o ângulo é menor que PI, ou a tensão Vpll é maior que zero, a GPIO12 é 1.
     * No contrário, a GPIO12 é 0.
     * */
    Vpll = cos(Teta);

    if (Vpll >= 0.0){
        // Colocar a GPIO em 3.3 V
        GpioDataRegs.GPASET.bit.GPIO10 = 1;
    }
    else{
        // Colocar a GPIO em 0 V
        GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    }
}

void Data_Estimation(void){

    Soma_Tensao_A += Tensao_A[AMOSTRAS-1]*Tensao_A[AMOSTRAS-1];
    Soma_Tensao_B += Tensao_B[AMOSTRAS-1]*Tensao_B[AMOSTRAS-1];
    Soma_Tensao_C += Tensao_C[AMOSTRAS-1]*Tensao_C[AMOSTRAS-1];
    //Soma_Corrente_A += Corrente_A[AMOSTRAS-1]*Corrente_A[AMOSTRAS-1];
    //Soma_Corrente_B += Corrente_B[AMOSTRAS-1]*Corrente_B[AMOSTRAS-1];
    //Soma_Corrente_C += Corrente_C[AMOSTRAS-1]*Corrente_C[AMOSTRAS-1];

    Soma_Corrente_A += Corrente_Af[0]*Corrente_Af[0];
    Soma_Corrente_B += Corrente_Bf[0]*Corrente_Bf[0];
    Soma_Corrente_C += Corrente_Cf[0]*Corrente_Cf[0];

    Contador_Amostras++;

    if(Contador_Amostras == 167 - 1){

        Tensao_A_RMS = sqrt(Soma_Tensao_A/167);
        Tensao_B_RMS = sqrt(Soma_Tensao_B/167);
        Tensao_C_RMS = sqrt(Soma_Tensao_C/167);
        Corrente_A_RMS = sqrt(Soma_Corrente_A/167);
        Corrente_B_RMS = sqrt(Soma_Corrente_B/167);
        Corrente_C_RMS = sqrt(Soma_Corrente_C/167);

        Contador_Amostras = 0;
        Soma_Tensao_A = 0;
        Soma_Tensao_B = 0;
        Soma_Tensao_C = 0;
        Soma_Corrente_A = 0;
        Soma_Corrente_B = 0;
        Soma_Corrente_C = 0;
    }

}

void Setup_ePWM(void){
        // Configures three ePWMs for use as a SPWM

        // Configuration of EPwm1
        EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // Set multiplexer register line GPIO0 to enable ePWM1A as output signal.
        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // Set multiplexer register line GPIO1 to enable ePWM1B
        EDIS;

        EPwm1Regs.TBCTL.bit.CTRMODE = 2; // TBCTL - Time Base Control Register
                                         // Field CTRMODE - Counter Mode
                                         // 00 = count up, 01 = count down, 10 = count
                                         // up and down and 11 = stop - freeze (default)
        EPwm1Regs.TBCTL.bit.CLKDIV = 1; // Divides by 1
        EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // Divides by 1

        // The equation to define the PWM period is
        // TBPRD = 0.5*((f_sysclkout)/(f_pwm * CLKDIV * HSPCLKDIV))
        // The system has f_sysclkout = 150 MHz. We want the f_pwm frequency of 5 kHz.
        // Using CLKDIV = 1 and HSPCLKDIV = 1, we can obtain
        // TBPRD = 0.5*(150*10^6)/(5*10^3*2) = 15000

        EPwm1Regs.TBPRD = 7500-1;//30000-1;

        // We would like to set ePWM1A to 1 on "CMPA - up match" and to
        // clear ePWM1A on event "CMPA - down math".
        // Register AQCTLA
        // CAD (bits 7-6): Action when CTR = CMPA on down count
        // CAU (bits 5-4): Action when CTR = CMPA on up count
        // 00 = do nothing (action disabled), 01 = clear (low), 10 = set (high), 11 = toogle
        // CAD = 1; CAU = 2;
        EPwm1Regs.AQCTLA.bit.CAD = 1;
        EPwm1Regs.AQCTLA.bit.CAU = 2;

        // Set register CMPA and CMPB to initially generate a
        // pulse width of 50%
        EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2;

        // Setup a delay time between outA and outB of 10 microseconds

        EPwm1Regs.DBRED = 750;
        EPwm1Regs.DBFED = 750;
        EPwm1Regs.DBCTL.bit.IN_MODE = 0;
        EPwm1Regs.DBCTL.bit.POLSEL = 2;
        EPwm1Regs.DBCTL.bit.OUT_MODE = 3;

        // Configuration of EPwm2
        EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // Set multiplexer register line GPIO2 to enable ePWM2A as output signal.
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // Set multiplexer register line GPIO3 to enable ePWM2B as output signal
        EDIS;
        EPwm2Regs.TBCTL.bit.CTRMODE = 2;
        EPwm2Regs.TBCTL.bit.CLKDIV = 1;
        EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
        EPwm2Regs.TBPRD = 7500-1;//30000-1;
        EPwm2Regs.AQCTLA.bit.CAD = 1;
        EPwm2Regs.AQCTLA.bit.CAU = 2;
        EPwm2Regs.CMPA.half.CMPA = EPwm2Regs.TBPRD/2;
        EPwm2Regs.DBRED = 750;
        EPwm2Regs.DBFED = 750;
        EPwm2Regs.DBCTL.bit.IN_MODE = 0;
        EPwm2Regs.DBCTL.bit.POLSEL = 2;
        EPwm2Regs.DBCTL.bit.OUT_MODE = 3;

        // Configuration of EPwm2
        EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // Set multiplexer register line GPIO4 to enable ePWM3A as output signal.
        GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1; // Set multiplexer register line GPIO5 to enable ePWM3B as output signal
        EDIS;
        EPwm3Regs.TBCTL.bit.CTRMODE = 2;
        EPwm3Regs.TBCTL.bit.CLKDIV = 1;
        EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
        EPwm3Regs.TBPRD = 7500-1;//30000-1;
        EPwm3Regs.AQCTLA.bit.CAD = 1;
        EPwm3Regs.AQCTLA.bit.CAU = 2;
        EPwm3Regs.CMPA.half.CMPA = EPwm3Regs.TBPRD/2;
        EPwm3Regs.DBRED = 750;
        EPwm3Regs.DBFED = 750;
        EPwm3Regs.DBCTL.bit.IN_MODE = 0;
        EPwm3Regs.DBCTL.bit.POLSEL = 2;
        EPwm3Regs.DBCTL.bit.OUT_MODE = 3;

}

void Malha_Controle(void){

    // Transformação vetorial das tensões: ABC para DQO
    Vd_med[1] = Vd_med[0];
    Vq_med[1] = Vq_med[0];
    Vd_med[0] = 0.66666667*( Tensao_A[AMOSTRAS-1]*cos(Teta) + Tensao_B[AMOSTRAS-1]*cos(Teta - 2.094395102) + Tensao_C[AMOSTRAS-1]*cos(Teta + 2.094395102) );
    Vq_med[0] = -0.66666667*( Tensao_A[AMOSTRAS-1]*sin(Teta) + Tensao_B[AMOSTRAS-1]*sin(Teta - 2.094395102) + Tensao_C[AMOSTRAS-1]*sin(Teta + 2.094395102) );

    // Transformação vetorial das correntes: ABC para DQO
    Id_med[1] = Id_med[0];
    Iq_med[1] = Iq_med[0];
    Id_med[0] = 0.66666667*( Corrente_A[AMOSTRAS-1]*cos(Teta) + Corrente_B[AMOSTRAS-1]*cos(Teta - 2.094395102) + Corrente_C[AMOSTRAS-1]*cos(Teta + 2.094395102) );
    Iq_med[0] = -0.66666667*( Corrente_A[AMOSTRAS-1]*sin(Teta) + Corrente_B[AMOSTRAS-1]*sin(Teta - 2.094395102) + Corrente_C[AMOSTRAS-1]*sin(Teta + 2.094395102) );

    // Filtragem e normalização das tensões
    // --------------------------------------
    // Filtragem e normalização das tensões: utiliza-se um filtro digital passa-baixas
    // com frequência de corte de 7.95 Hz (50 rad/s)

    Vd_medf[1] = Vd_medf[0];
    Vq_medf[1] = Vq_medf[0];
    Vd_medf[0] = 0.002494*Vd_med[0] + 0.002494*Vd_med[1] + 0.995*Vd_medf[1];
    Vq_medf[0] = 0.002494*Vq_med[0] + 0.002494*Vq_med[1] + 0.995*Vq_medf[1];

    // * Filtragem e normalização das correntes
    // * --------------------------------------
    // * Filtragem e normalização das correntes: utiliza-se um filtro digital passa-baixas
    // * com frequência de corte de 79,57 Hz (500 rad/s)

    Id_medf[1] = Id_medf[0];
    Iq_medf[1] = Iq_medf[0];
    Id_medf[0] = 0.02439*(Id_med[0] + Id_med[1]) + 0.9512*Id_med[1];
    Iq_medf[0] = 0.02439*(Iq_med[0] + Iq_med[1]) + 0.9512*Iq_med[1];

    // Conversão das tensões medidas para pu

    Vd_pu = Vd_medf[0]/(V_base*sqrt(2)/sqrt(3));
    Vq_pu = Vq_medf[0]/(V_base*sqrt(2)/sqrt(3));

    // Conversão das correntes medidas para pu

    Id_pu = Id_medf[0]/I_base;
    Iq_pu = Iq_medf[0]/I_base;

    // Cálculo de potência

    P_med[1] = P_med[0];
    Q_med[1] = Q_med[0];

    P_med[0] = 1.5*(Vd_med[0]*Id_med[0] + Vq_med[0]*Iq_med[0])/S_base;
    Q_med[0] = 1.5*(Vq_med[0]*Id_med[0] - Vd_med[0]*Iq_med[0])/S_base;

    // * Filtragem das potências trifásicas. O filtro passa-baixas têm
    // * frequência de corte em 100 rad/s.

    P_medf[1] = P_medf[0];
    Q_medf[1] = Q_medf[0];

    P_medf[0] = 0.004975*(P_med[0] + P_med[1]) + 0.99*P_medf[1];
    Q_medf[0] = 0.004975*(Q_med[0] + Q_med[1]) + 0.99*Q_medf[1];

    // Controladores de potência

    Erro_P[1] = Erro_P[0];
    Erro_Q[1] = Erro_Q[0];

    Erro_P[0] = -1*(1 - V_CAPF[0]/V_REF_CC);
    Erro_Q[0] = 0 - Q_medf[0];

    Id_ref[1] = Id_ref[0];

    Id_ref[0] = K1*Erro_P[0] - K2*Erro_P[1] + Id_ref[1];

    // Teste da malha interna de corrente
    //Id_ref[0] = 0.0;// 0.01 * 15.2 A = 0.152 A // <--- Valor inserido de corrente

/* Código utilizado para verificar em passos a malha interna de corrente, setando valores
 * de Id_ref[0] a cada 5 segundos.
 */

    if(counter_iref < 50000){
        Id_ref[0] = 0.02; // <-- 0.3 A
    }
    else if(counter_iref >= 50000 && counter_iref < 100000){
        Id_ref[0] = 2*0.02; // <-- 0.6 A
    }
    else if(counter_iref >= 100000 && counter_iref < 150000){
        Id_ref[0] = 3*0.02; // <-- 0.9 A
    }
    else if(counter_iref >= 150000 && counter_iref < 200000){
        Id_ref[0] = 4*0.02; // <-- 1.2 A
    }
    else if(counter_iref >= 200000 && counter_iref < 250000){
        Id_ref[0] = 5*0.02; // <-- 1.5 A
    }

    if (counter_iref_sequence == 0){
        counter_iref++;
    }
    else{
        counter_iref--;
    }

    if(counter_iref == 0 && counter_iref_sequence == 1){
        counter_iref_sequence = 0;
        counter_iref = 50000;
    }
    else if(counter_iref == 250000-1 && counter_iref_sequence == 0){
        counter_iref_sequence = 1;
        counter_iref = 200000-1;
    }

   /* if(V_CAPF[0] > 530.0){
        Id_ref[0] = 4*0.01974;
    }
    else if(V_CAPF[0] < 500.0)
    {
        Id_ref[0] = 0;
    }*/


    Iq_ref[1] = Iq_ref[0];

    Iq_ref[0] = K3*Erro_Q[0] - K4*Erro_Q[1] + Iq_ref[1];
    Iq_ref[0] = 0.00; // <-- Valor inserido de corrente

    // Controladores de corrente de eixo direto: calculo do erro e execução do controle PI

    Erro_id[1] = Erro_id[0];
    Erro_id[0] = Id_ref[0] - Id_pu;
    Pi_id[1] = Pi_id[0];
    Pi_id[0] = K5*Erro_id[0] - K6*Erro_id[1] + Pi_id[1];

    Ed_inv = Pi_id[0] +  Vd_pu - (L_pu*Iq_pu); // Tensão de saída do inversor

    // Controladores de corrente de eixo em quadratura: calculo do erro e execução do controle PI
    Erro_iq[1] = Erro_iq[0];
    Erro_iq[0] = -Iq_ref[0] - Iq_pu;
    Pi_iq[1] = Pi_iq[0];
    Pi_iq[0] = K5*Erro_iq[0] - K6*Erro_iq[1] + Pi_iq[1];

    Eq_inv = Pi_iq[0] +  Vq_pu + (L_pu*Id_pu); // Tensão de saída do inversor

    //Ed_inv = 1.2;
    //Eq_inv = 0.0;

    // Transformação vetorial inversa
    InvFaseA = 0.66666667*( Ed_inv*cos(Teta) - Eq_inv*sin(Teta) );
    InvFaseB = 0.66666667*( Ed_inv*cos(Teta - 2.094395102) - Eq_inv*sin(Teta - 2.094395102) );
    InvFaseC = 0.66666667*( Ed_inv*cos(Teta + 2.094395102) - Eq_inv*sin(Teta + 2.094395102) );

}

void Change_ePWM(void){
    // Esta função converte a onda modulante que está entre -1 e 1
    // em um valor inteiro e insere em cada registrador CMPA para que
    // estes possam ser comparados com os registradores TBPRD de cada ePWM

    unsigned int value1, value2, value3;
    value1 = (InvFaseA + 1.0)*(EPwm1Regs.TBPRD/2);
    value2 = (InvFaseB + 1.0)*(EPwm1Regs.TBPRD/2);
    value3 = (InvFaseC + 1.0)*(EPwm1Regs.TBPRD/2);

    value1 = (unsigned int)value1;
    value2 = (unsigned int)value2;
    value3 = (unsigned int)value3;

    //EPwm1Regs.CMPA.half.CMPA = (EPwm1Regs.TBPRD/2);
    //EPwm2Regs.CMPA.half.CMPA = (EPwm2Regs.TBPRD/2);
    //EPwm3Regs.CMPA.half.CMPA = (EPwm3Regs.TBPRD/2);
    EPwm1Regs.CMPA.half.CMPA = value1;
    EPwm2Regs.CMPA.half.CMPA = value2;
    EPwm3Regs.CMPA.half.CMPA = value3;

}


//===========================================================================
// End of SourceCode.
//===========================================================================
