# Balance with BLDC 
This project was developed in 2022 in a lecture from my BSc. The objective was to design a balance which, with 2 BLDC motors, could find the desired angle. 

The next image shows the block diagram of the system:

![image](https://github.com/user-attachments/assets/9ebaade4-612b-4dfe-baa9-0b50d49b84cc)

It was composed by 2 BLDC controllers controlled by a MSP430 programmed in bare metal and a central controller programmed with TI-RTOS. 

## Block diagram of each BLDC controller PCB
![image](https://github.com/user-attachments/assets/0e59a497-1595-4fd0-be35-c5ec9411361d)

## Schematic of the BLDC controller
![image](https://github.com/user-attachments/assets/751235b9-0593-4fd5-a551-075cee9f3c7f)

## RTOS architecture: 
![image](https://github.com/user-attachments/assets/d4cd81b0-a341-4ce9-88af-673c4594db42)

## Final built of the system
![image](https://github.com/user-attachments/assets/06aaa2a7-ce82-4ea9-b542-1953a68dc476)




