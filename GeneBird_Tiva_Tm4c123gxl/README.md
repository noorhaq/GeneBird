 Tiva C Quadcopter Controller


|           |                                                   |       
| --------- | ------------------------------------------------- |  
| Props:    | 10x4.5        200*4 =800RS                              |
| Motors:   | 950kV    : 1390 *4  = 5560RS  :1000kv 750rs  =3000                                  |
| Battery:  | 5200mAh  : 4400 RS :2200mah : 2100Rs                                |
| ESC:      | Afro ESC or any ESC that has 1000Hz refresh rate 800*4 =3200 |
| Frame:    | 550X from Hobbyking          1600Rs                     |
| TX/RX:    | flysky TGY-i6 and iA6 6 channel reciever    9000rs    receiver 1706rs |
| Acc/Gyro: | MPU6050         170rs                                  |
| Sonar:    | HC-SR04          170rs                               |
| ESP 8266: | 275rs
Total : 8200rs(ESC,Motor,Frame,Propeller) + 4400rs + 170+ 170rs + 9000rs +275rs = 22,215

Quad flies in x-config.

## OUTPUT PINS:
|              |                                                    |
| ------------ | -------------------------------------------------- |
| I2C:         | PB2 - scl , PB3 - sda                              |
| UART:        | PA0 - rx , PA1 - tx                                |
| MPU6050:     | same as I2C , INT pin from mpu6050 to PE2 |
| ESC(motors): | PB4, PB5, PB6, PB7                                 |
| RX:          | PA2, PA3, PA4, PA5, PA6, PA8                       |
| LIGHTS       | PC4(F), PC5(B), PC6(R), PC7(L)                     |
| ESP8266      | PE3(int)			                    |

