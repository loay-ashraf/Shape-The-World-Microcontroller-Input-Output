// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define LIGHT_CARS              (*((volatile unsigned long *)0x400050FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))

#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002401C)) // bits 2-0
#define SENSOR                  (*((volatile unsigned long *)0x4002401C))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

#define LIGHT_PED								(*((volatile unsigned long *)0x40025028))	// bits 3,1
#define GPIO_PORTF_OUT       		(*((volatile unsigned long *)0x40025028))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))	
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOF      0x00000020	// port F Clock Gating Control

#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))

#define goN   0
#define waitN 1
#define goE   2
#define waitE 3
#define goWalk 4	
#define waitWalk_1 5
#define waitWalk_2 6
#define waitWalk_3 7
#define waitWalk_4 8	
#define waitWalk_5 9
#define waitWalk_6 10


// ***** 2. Global Declarations Section *****

struct State {
  unsigned long Out_cars; // 6-bit output for west/south lights
	unsigned long Out_ped;	// 3-bit output for walk/don't walk lights 
  unsigned long Time;  // delay in 10ms units
  unsigned long Next[8];}; // 8 possible next states for 3-bit possible input combinations
typedef const struct State STyp;
	
STyp FSM[11]={
 {0x21,0x02,50,{goN,waitN,goN,waitN,waitN,waitN,waitN,waitN}}, 
 {0x22,0x02,50,{goE,goE,goE,goE,goE,goE,goE,goE}},
 {0x0C,0x02,50,{goE,goE,waitE,waitE,waitE,waitE,waitE,waitE}},
 {0x14,0x02,50,{goN,goN,goN,goN,goWalk,goWalk,goWalk,goWalk}},
 {0x24,0x08,50,{goWalk,waitWalk_1,waitWalk_1,waitWalk_1,goWalk,waitWalk_1,waitWalk_1,waitWalk_1}},
 {0x24,0x02,50,{goWalk,waitWalk_2,waitWalk_2,waitWalk_2,goWalk,waitWalk_2,waitWalk_2,waitWalk_2}},
 {0x24,0,50,{goWalk,waitWalk_3,waitWalk_3,waitWalk_3,goWalk,waitWalk_3,waitWalk_3,waitWalk_3}},
 {0x24,0x02,50,{goWalk,waitWalk_4,waitWalk_4,waitWalk_4,goWalk,waitWalk_4,waitWalk_4,waitWalk_4}},
 {0x24,0,50,{goWalk,waitWalk_5,waitWalk_5,waitWalk_5,goWalk,waitWalk_5,waitWalk_5,waitWalk_5}},
 {0x24,0x02,50,{goWalk,waitWalk_6,waitWalk_6,waitWalk_6,goWalk,waitWalk_6,waitWalk_6,waitWalk_6}},
 {0x24,0,50,{goWalk,goE,goN,goE,goWalk,goE,goN,goN}}};	

unsigned long curr_S;  // index to the current state 
unsigned long Input; 

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void SysTick_Init(void);
void SysTick_Wait(unsigned long delay);
void SysTick_Wait10ms(unsigned long delay); 

// ***** 3. Subroutines Section *****

int main(void){ 
	
	volatile unsigned long delay;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
 
  
  EnableInterrupts();
	
	SysTick_Init();   // initializing SysTick timer
  SYSCTL_RCGC2_R |= 0x32;      // 1) B E F
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock PORTE, PORTB 
  GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2-0
	
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R |= 0x0A;           // allow changes to PF3,PF1       
  GPIO_PORTF_AMSEL_R &= ~0x0A;        // 3) disable analog function
  GPIO_PORTF_PCTL_R &= ~0x00FFFFFF;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0A;          // 5) PF3,PF1 output   
  GPIO_PORTF_AFSEL_R &= ~0x0A;        // 6) no alternate function      
  GPIO_PORTF_DEN_R |= 0x0A;          // 7) enable digital pins PF3,PF1       
  curr_S = goN;  
  while(1){
     
		LIGHT_CARS = FSM[curr_S].Out_cars;  // set lights for south/west roads first
		LIGHT_PED = FSM[curr_S].Out_ped;	// set lights for pedestrians second
    SysTick_Wait10ms(FSM[curr_S].Time);	// wait for the set time
    Input = SENSOR;     // read sensors
    curr_S = FSM[curr_S].Next[Input];  // transition to new state
		
  }
}

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 10000us equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

