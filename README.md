# MS51_PWD_WAKEUP
 MS51_PWD_WAKEUP

update @ 2019/12/11

Power down then wake up sample code

1. Enable self wake up timer , check init_WakeupTimer , set_WakeupTimer , WakeUp_Timer_ISR

2. Since power down will disable timer/ADC pheripheral , re-init after power on

3. Use GPIO to monitor power on wake up status

- P30 : monitor power down (GPIO = 0) and power on (GPIO = 1)
![image](https://github.com/released/MS51_PWD_WAKEUP/blob/master/00_Wake_up.png)

- P04 : GPIO = 0 , TIMER0 DELAY 100 ms , GPIO = 1
![image](https://github.com/released/MS51_PWD_WAKEUP/blob/master/01_Timer0_Delay.png)

- P05 : GPIO = 0 , ADC init , convert , GPIO = 1
![image](https://github.com/released/MS51_PWD_WAKEUP/blob/master/02_ADC_Convert.png)
