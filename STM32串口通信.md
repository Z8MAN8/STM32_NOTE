## STM32串口通信



#### HAL库transmit

###### HAL_UART_Transmit的区别：

1. HAL_UART_Transmit 阻塞模式。初始化后可直接使用，无需中断。在实时性要求较高的RTOS中不宜使用。
2. HAL_UART_Transmit_IT 非阻塞模式，通过中断方式发送。必要准备工作：

1）使能USARTx_IRQn中断

2）启用中断服务函数USARTx_IRQHandler，在里面清除标志位。

每发送一个字符会进一次USARTx_IRQHandler。

3. HAL_UART_Transmit_DMA 非阻塞模式，先发送到DMA，再通过中断方式发送。必要准备工作：

1）使能USARTx_IRQn，DMAx_Streamy_IRQn中断。CubeMX里面可以选择不使能串口中断，实际测试是必须要使能的，否则第二次使用该函数会一直返回HAL_UART_STATE_BUSY_TX。

2）启用中断服务函数USARTx_IRQHandler、DMAx_Streamy_IRQHandler，在里面清除标志位。

整个过程只产生两次中断，第一次是进入DMAx_Streamy_IRQHandler；第二次进入USARTx_IRQHandler。

##### 