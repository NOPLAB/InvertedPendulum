clear
clc
clc

list = blelist

b = ble("UART Service")
c_tx = characteristic(b, "ff94ca6f-91a5-4048-b805-755bca71075e", "a3308588-5d66-4643-b0ef-0df52539d950");
write(c_tx, "Hello, World!!")
