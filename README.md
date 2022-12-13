## STM32F103C8T6_AmpereVoltageReader
485 Communication
stm32F103C8T6 CPU 64Kbyte Flash 

### 예)  4개의 데이타를 요구함.
**01 04 00 00 00 04 F1 C9**  
address 0 : BusVoltage  
address 1 : Current   
address 2 : ReadShuntVoltage  
address 3 : address  

### 예) 485 ID를 2번으로 아이디를 바꾸고 싶을경우
**01 05 00 00 00 02 4c 0b**
