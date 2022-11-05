# 海南大学Mecha战队电控代码规范

## 1.代码书写规范

### 1.1命名规范

* 缩进使用4个空格，禁止使用tab

* 结构体名和美剧名遵循大驼峰式命名法，例如：VpiHwCtx

* 函数命名采用sb_do_sth的规则，例如Sensor_Read_samples

* 变量名称使用小写字符，字符中间用‘_’分割，如reg_value

* 枚举元素定义使用全大写的命名规则，如

  ~~~c
  type enum VpeCmdu{
      VPE_CMD_BASE,
      VPE_CMD_DEC_PIC_CONSUME,
      VPE_CMD_DEC_STRM_BUF_COUNT,
  }VpeCmd;
  ~~~

* 在cubemx定义引脚时需要加上标签，命名用全大写+下划线的方式 如：IO_I2C_SCL …

### 1.2注释规范

* 代码块注释用 /*  */  ，行注释使用//

  结构体、枚举注释：

  ~~~
  /**
    * @brief  UART handle Structure definition
    */
  typedef struct __UART_HandleTypeDef
  {
    USART_TypeDef  *Instance;       /*!< UART registers base address        */
    UART_InitTypeDef Init;          /*!< UART communication parameters      */
    uint8_t         *pTxBuffPtr;    /*!< Pointer to UART Tx transfer Buffer */
    uint16_t        TxXferSize;     /*!< UART Tx Transfer size              */
    __IO uint16_t   TxXferCount;    /*!< UART Tx Transfer Counter           */
    uint8_t         *pRxBuffPtr;              /*!< Pointer to UART Rx transfer Buffer */
    uint16_t        RxXferSize;     /*!< UART Rx Transfer size              */
  } UART_HandleTypeDef;
  ~~~

  

* 是函数就需要注释，注释格式如下

  ~~~
  /**
    * @brief Send an amount of data in blocking mode.
    * @param huart   UART handle.
    * @param pData   Pointer to data buffer (u8 or u16 data elements).
    * @param Size    Amount of data elements (u8 or u16) to be sent.
    * @param Timeout Timeout duration.
    * @retval HAL status
    */
  HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  {
  	/*
      *    如果程序过于复杂，这里可以写明，具体算法和思路。
      */
  }
  ~~~

  

### 1.3 文件规范

* 宏定义、结构体必须写在对应头文件中



## 2. 代码合作规范

### 2.1 流程规范

#### 2.1.1 合作——分工

合作之前需要协商好各自的分工，分工时需要将整台车根据功能模块进行拆分，尽量保证每个人负责相对独立的一部分功能，各个功能模块之间通过相对应的标志位进行联动、合作。

#### 2.1.2 合作——代码托管平台

##### 2.1.2.1 流程

使用git合作流程大体遵守如下规范（gitee代码托管平台为例）：

1. 首先创建gitee仓库，将仓库分享给参与合作的组内成员

2. 合作成员在本地clone对应仓库代码

3. 以本地的master分支为基础，重新创建新的分支来写对应的功能模块，并且将分支推送到远端，在远端仓库创建对应的分支

   ~~~
   git branch feature //创建feature分支
   git push origin feature:feature //在远端创建feature分支
   ~~~

4. 跳转到对应的分支进行对应功能代码的编写

   ~~~
   git switch feature
   ~~~

5. 在功能代码编写的时候，如果进行测试为了保护正在编写的代码，需要在feature分支的基础上另开辟新的分支

   ~~~
   //当前在feature分支下
   git branch dev //以feature分支为基础创建dev分支
   git switch dev //跳转到dev分支进行测试
   ~~~

6. 功能代码编写完毕测试成功以后需要和本地的master分支进行合并，合并之后将本地的master分支推送到远端的master分支上

   ~~~
   git switch master //跳转到本地master分支
   git merge feature //将feature分支合并到master分支
   git push origin master:master //将本地的master推送到远端仓库
   ~~~

##### 2.1.2.2 注意事项

* 在stm32工程推送到远端仓库时需要将CMakeLists生成出来的cmake-buildxxx文件夹加入到.gitignore文件中，避免其他人在pull的时候构建工程出错

  ![image-20220703152244379](../img/海南大学Mecha战队电控代码规范/image-20220703152244379.png)

* 开始写代码的时候要先将远程仓库的代码拉下来以后再写，保证手中的代码是最新的。

* master分支只能合并其他分支来进行修改，而不能直接在master分支上进行修改

* feature完成的功能代码测试完后及时合并并推送，避免合作者在旧代码上开发，导致功能的缺少，并且后续推送出现冲突

### 2.2 推送信息规范

在合作的时候，没法面对面进行沟通，或者是对方在睡觉联系不上的时候，这时就需要通过提交的commit信息来了解代码的最新的变动，提升开发效率。因此，规范好推送的信息是有必要的。 之前大家提交都是使用命令 git commit -m 简介，但是这种方法不能很好的让读者了解这次改动。因此之后的代码提交时要写详细信息，方法有两种：

1. 用git命令行：git commit （如果不加-m参数就会自动打开vim编辑器）在vim编辑器中进行详细明细的编辑

2. 使用clion的commit file功能

   ![image-20220703165456732](../img/海南大学Mecha战队电控代码规范/image-20220703165456732.png)

![image-20220703165542059](../img/海南大学Mecha战队电控代码规范/image-20220703165542059.png)

规范Commit Message的提交格式如下：分为页眉（header）、正文（body）两部分

#### 2.2.1 页眉

页眉的内容是本次提交的类型和简述。

类型大概有如下几种：

1. 代码格式的修改 （style）
2. 代码的重构（refactor）
3. 版本的回滚（revert）
4. 优化（perf）
5. 修改bug（fix）
6. 增加新功能（feat）
7. 文档修改（docs）

简述便是简略的描述一下本次修改

总体来说页眉的内容就是 平常自己随便写代码时的 git commit -m 后的内容

#### 2.2.2 正文

正文便是详细地描述一下修改的地方。例如，本次修改影响到了哪个函数、增添了功能能实现什么样的目的、新增了的功能如何操作….包括代码上可能需要解释的一些变动的细节。

#### 2.2.3 格式

* header和正文之间空一格分隔开
* 正文每一点以星号“*”+空格 开头

## 3. 文档规范

1. 文档要包含传感器等设备的引脚来方便接线
2. 需要写明程序的模式、以及模式切换的方法
3. 

