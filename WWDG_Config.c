#include "Global.h"

void WWDG_Config(void)
{
    RCC->APB1ENR|=RCC_APB1ENR_WWDGEN ;
    //使能看门狗外设时钟
    
    /* Configure WWDG */
    /* (1) set prescaler to have a rollover each about 5.5ms, set window value (about 2.25ms) */
    /* (2) Refresh WWDG before activate it */
    /* (3) Activate WWDG */
    WWDG->CFR=0x7F ;
    /* (1) */
    WWDG->CR=0x4B ;
    /* (2) */
    WWDG->CR|=WWDG_CR_WDGA ;
    /* (3) */
}
