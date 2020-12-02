# DLLStrategy

DLL策略的模板项目

## 说明

该模板适用于Simuro5v5平台的[DLLAdapter](https://github.com/npuv5pp/V5DLLAdapter)项目。

新的接口函数包括：

- `void GetTeamInfo(TeamInfo* teaminfo)`

  用于指定策略信息，目前包含队名字段。

  参数`TeamInfo* teaminfo`**需要策略填充自身的信息**，会返回给平台。

- `void GetInstruction(Field* field)`

  比赛中的每拍被调用，**需要策略指定轮速**，相当于旧接口的Strategy。

  参数`Field* field`为`In/Out`参数，存储当前赛场信息，并允许策略修改己方轮速。

- `void GetPlacement(Field* field)`

  每次自动摆位时被调用，**需要策略指定摆位信息**。

  参数`Field* field`为`In/Out`参数，存储当前赛场信息，并允许策略修改己方位置（和球的位置）。

- `void OnEvent(EventType type, void* argument)`

  事件发生时被调用。

  参数`EventType type`表示事件类型；

  参数`void* argument`表示该事件的参数，如果不含参数，则为NULL。

## 其他

新的比赛接口函数所接受的参数类型为`Field`，里面包含了赛场信息，类似旧接口中的`Environment`，但是`field`中并不保存`gamestate`和`whosball`，因为这些变量只在回合开始时改变。

## Credits

版权所有(C) 西北工业大学V5++团队。保留所有权利。