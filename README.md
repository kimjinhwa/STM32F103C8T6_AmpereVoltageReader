## STM32F103C8T6_AmpereVoltageReader
485 Communication
stm32F103C8T6 CPU 64Kbyte Flash 

### 예)  4개의 데이타를 요구함.
**01 04 00 00 00 04 F1 C9**  
address 0 : BusVoltage  
 읽은 값이 07 80 이면 Decimal로 1,920가 되며 1000으로 나누면 1.920V가 된다.
address 1 : Current   
 읽은 값이 01 3B 이면 Decimal로 315가 되며 10으로 나누면 31.5mA가 된다.
  
address 2 : ReadShuntVoltage  
 예비용으로 사용되면 항상 0x1234를 리턴한다.
address 3 : address  
 장비의 Address를 리턴하며 만일 장비의 Address를 모를경우 0x80(128)의 Address로 문의 하면 응답하여
 장비의 주소를 알수 있다.


### 예) 485 ID를 2번으로 아이디를 바꾸고 싶을경우
**01 05 00 00 00 02 4c 0b**


[check sum calulater](https://crccalc.com/)
