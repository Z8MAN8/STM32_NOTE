# OpenMV学习笔记

2022年5月16日

学习能让我得到真正属于我的东西

[TOC]



## 基本模块

[大致介绍]: https://book.openmv.cc/image/
[详细资料]: https://docs.singtown.com/micropython/zh/latest/openmvcam/library/index.html#openmv-cam



### 感光元件sensor模块

[粗略函数]: https://book.openmv.cc/image/sensor.html
[详细函数]: https://docs.singtown.com/micropython/zh/latest/openmvcam/library/omv.sensor.html



### Statustucs模块：

```python
img.get_statistics(roi=(0,0,10,20))
```

其中roi是目标区域。注意，这里的roi，bins之类的参数，一定要显式地标明。（roi感兴趣区域，roi的格式是(x, y, w, h)的tupple.）



### April Tag：

无特殊需求，推荐使用TAG36H11
