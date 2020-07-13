#include "include/stm32f103xb.h"

// manual timing calculation at comile time
    /*
     * assumptions (these are configured in the startup.S and explained in startup-Clock-setup-info.txt):
     *   (not repeated here)
     * specifications:
     *  1.  FSM update Interval = 0.5 ms (i.e. IRQ period)
     *  2.  allowed time elapsed from being seated to finish buckling the seatbeat 
     *        WITHOUT triggering the warning LED or buzzer = 6 sec 
     * 
     * the corresponding constant literals to evaluate:
     *  1.  IRQ period in unit time of SysTick Input Clock
     *  2.  maximum delay before triggering warning (i.e. in unit time of ISR Period!!!)
     * 
     * make sure they are ALL integers AND within their respective expected range!!!
     */
    #define SysTimerInterval_ticks 2000 // see item 1 above, range: 1... 2^24
    #define delayBeforeWarning 12000 //12000    // see item 2 above, expect 1... 2^32
        
// MCU Digital I/O usage (in pin number, all in GPIO Port A)
    // FSM inputs (all in GPIO Port A)
    #define INPUT_SEAT_OCCUPIED 6
    #define INPUT_BELT_ON  7
    
   // FSM outputs (also in GPIO Port A)
    #define OUTPUT_WARNING_SIGNAL 4 // could be a warning LED or buzzer or ...

// some helpful macros concerning state transition actions 
// (this facilitates examining when such action is executed during code analysis)
    #define ACTION_WHEN_ENTERING_WARNING  \
            GPIOA->BSRR = (1<<OUTPUT_WARNING_SIGNAL)
        // turn on the warning output!!!  
    #define ACTION_WHEN_LEAVING_WARNING  \
            GPIOA->BSRR = (1<<(OUTPUT_WARNING_SIGNAL+16))
        // turn off the warning signal
    #define ACTION_WHEN_ENTERING_SEATED_NO_BELT \
            elapsedTime_until_BucklingUp = 1
// more macros
    #define getState(GPIOx,pinNum) (((GPIOx->IDR)>>pinNum)& 1U)==1U
        
    enum SeatBeltReminderFsmStates{IDLE,SEAT_NO_BELT, SEAT_WITH_BELT, WARNING};    
    
// global variable (assuming only modified by the SysTick Handler)
    static enum SeatBeltReminderFsmStates stateSeat1 = IDLE; // i.e. initial state
    	// note: benefit of initial state being numerically 0: save (some) flash space
    static uint32_t elapsedTime_until_BucklingUp;  //actually 
        /* time in state SEAT_NO_BELT  [in unit time of SysTick IRQ period]
         * (for visualization, we could keep counting it in State 3*/

// prototypes
    int main(void); // only for startup configuration, empty while-loop
    void configIO(void);
    void SysTick_Handler(void);

// definitions
int main(void){
    configIO();
    (void) SysTick_Config(SysTimerInterval_ticks); // macro defined in "core_cm3.h"
    while (1);/*better alternative: inline with wfi */
    return 0;
}

void SysTick_Handler (void){  //ISR, see STM32F1_vecTable.S
// exit the ISR if the IRQ was not REALLY caused by the SysTick
    if (((SysTick->CTRL)&SysTick_CTRL_COUNTFLAG_Msk) != SysTick_CTRL_COUNTFLAG_Msk) {                   
        return ;
    }
 
    
// the Time-triggered Seat Belt Reminder System FSM

	// fetching physical (push button) inputs
	volatile uint32_t isSeated = getState(GPIOA,INPUT_SEAT_OCCUPIED);
	volatile uint32_t isBelted = getState(GPIOA,INPUT_BELT_ON);
    
	// performing the FSM logic
	switch (stateSeat1){
		case IDLE: // aka state 0
            if (isSeated){
                ACTION_WHEN_ENTERING_SEATED_NO_BELT;
                stateSeat1 = SEAT_NO_BELT;      // state transition 0-->1 (even if belted)
            }
            // omitting the direct state transition: 0-->2 
            //  if seated && belted && currently in IDLE state: 0-->1-->2 instead
            break;             
        case SEAT_NO_BELT: // aka state 1
            if (elapsedTime_until_BucklingUp > delayBeforeWarning) {
                ACTION_WHEN_ENTERING_WARNING;    // note: this is the only way to state 3
                stateSeat1 = WARNING;           // state transition 1-->3
                break;
            }
            if (isBelted){  
                stateSeat1 = SEAT_WITH_BELT;    // state transition 1-->2  
                break;
            }
            if (!isSeated){ 
                stateSeat1 = IDLE;              // state transition 1-->0
            }
            elapsedTime_until_BucklingUp++;            
            break;   
        case SEAT_WITH_BELT: // aka state 2
            if (!isBelted){
                ACTION_WHEN_ENTERING_SEATED_NO_BELT;
                stateSeat1 = SEAT_NO_BELT;      // state transition 2-->1
                break;
            }
            if (!isSeated){ // Just in case
                stateSeat1 = IDLE;              // state transition 2-->0
                break;
            }
            break;    
        case WARNING: // aka state 3
            if (isBelted){
                ACTION_WHEN_LEAVING_WARNING; 
                stateSeat1 = SEAT_WITH_BELT;    // state transition 3-->2
                break;
            }
            if (!isSeated){ // and still belted...!?
                ACTION_WHEN_LEAVING_WARNING;
                stateSeat1 = IDLE;              // state transition 3-->0
                break;
            }
            break;
    } 
}

void configIO(void){ // change me with HAL if you want to!
    // RCC control
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // enable GPIO Port A
    
    // configure the pins for the seat and belt sensor inputs AND the OutputSensor
        // clear the value for the 3 pins involved
        GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7|  
                        GPIO_CRL_CNF6 | GPIO_CRL_MODE6| 
                        GPIO_CRL_CNF4 | GPIO_CRL_MODE4); 
        // set the corresponding bits according to our Pin Assignment
         GPIOA->CRL |= (
         	GPIO_CRL_MODE4_1 |  // PA4 output
            GPIO_CRL_CNF6_0|  // PA6 (seat detector input as floating input, externally pulled down)
            GPIO_CRL_CNF7_0  // PA7 (belt detector input as floating input, externally pulled down)
         );
}
