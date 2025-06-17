# Simple-DIPC
A simpel controls project consisting of first a Single Inverted Pendulum Cart (SIPC) later upgraded to a Double Inverted Pendulum Cart (DIPC).

The aim of this project is to explore many different ways to robustly control the chaotic systems and gain experience on aspects of control systems I may not be too familiar with while also gaining more hands on experience

## Results

### SIPC with PID + LPF in Simulink
![image](https://github.com/user-attachments/assets/fce2b765-b667-4f62-8b31-5aa1211aa39a)

A Simulink simulation with the pendulum starting at 10° to the right of the vertical with a reference point set to 0°. Both input disturbance and sensor noise are included using the Simulink blocks 'Band Limited White-Noise'
