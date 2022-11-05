# PID基本优化

## 1.梯形积分

将常规的PID算法中通过矩形面积来近似计算积分的方法用**梯形面积**进行替换，从而达到**控制频率低**的场合**提高精度**的效果

```
static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);
}
```



## 2.微分先行

在常规PID中，微分项是微分系数乘误差的微分，而误差的微分又可以化成目标信号的微分减去输入（测量值）的微分，即：
$$
Dout = Kd *\frac{d \text { Err }}{d t}=Kd*(\frac{d \text { Target }}{d t}-\frac{d \text { Input }}{d t})
$$
通过**去除目标信号的微分**，只**保留输入（测量值）的微分**，从而解决**微分冲击**现象，即：
$$
Dout =-Kd*\frac{d \text { Input }}{d t}
$$


```c
static void f_Derivative_On_Measurement(PID_TypeDef *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure);
}
```



## 3.积分分离

误差瞬间增大，误差的积分会迅速累积，从而导致积分项输出过大，进而产生较大的超调，严重时甚至会引起系统振荡。

为解决上述问题，在误差超过一定**阈值**时停止积分过程，当误差小于这个**阈值**，降至合理范围的时候，再继续积分过程，这样就避免了由于**目标信号变化引起的巨大误差**累积到积分项中。

```c
static void f_Integral_Separation(PID_TypeDef *pid)
{
    //首先判断该周期内积分项是否为积累趋势
    //只有本次积分为积累趋势才会取消积分项
    if (pid->Err * pid->Iout > 0)
    {
        if (ABS(pid->Err) <= pid->MaxErr)
            return; //完整积分
        else
            pid->ITerm = 0;//取消积分环节
    }
}

```



## 4.变速积分

积分分离的进一步优化。
$$
ITerm=\left\{\begin{array}{lr}Ki * Err & |Err| \leq B \\Ki*Err*\frac{A-|Err|+B}{A} & B<|Err| \leq A+B \\0 & |Err|>\operatorname{A} + B\end{array}\right.
$$

```c
static void f_Changing_Integral_Rate(PID_TypeDef *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        if (ABS(pid->Err) <= pid->ScalarB)
            return; //完整积分
        if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
            //使用线性函数过渡
            pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
        else
            pid->ITerm = 0;//取消积分环节
    }
}
```



## 5.积分抗饱和

在某些情况下，输出限幅的存在或者执行器本身的限制导致了被控对象无法达到我们的期望值，这就意味着误差将会持续存在，持续存在的误差会使积分项过分积累，从而导致我们在下调期望值使误差反向时，积分项需要一段时间来下降至最大输出一下，这段过程中PID的输出将会持续保持最大，从而导致响应的严重滞后，我们将这种情况称之为**积分饱和**（integral windup）。

一种思路便是在PID输出达到输出限幅时停止积分过程。也可以设定一个合适的积分阈值，来通过这个阈值对积分进行限幅。

```c
static void f_Integral_Limit(PID_TypeDef *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (ABS(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
			//在取消积分作用前，同样需要先判断当前周期内积分是否积累
            //如果积分为减小趋势则不需要限制其减小
            //原因与（三）中相同。
            pid->ITerm = 0;
        }
    }
    
    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}
```



## 6.电机堵转保护

```c
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    //排除PID输出本身很小的情况
    if (pid->Output < pid->MaxOut * 0.01)
        return;
	
    //考虑到该判断策略的灵活性，0.9这个常数的选取是很灵活的
    if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
    {
        //电机堵转计数
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        //上述现象持续一段时间则被认定为电机堵转
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

```





详细资料来源：[PID库与PID基本优化（一） - WangHongxi - 博客园 (cnblogs.com)](https://www.cnblogs.com/WangHongxi/p/12404424.html)