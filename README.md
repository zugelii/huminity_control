# huminity_control
1. nrfnetwork
该网络的拓扑结构中，必须有node:0，做为master,通过master实现个节点之间的通讯（master 转发功能）。
而且node的地址按照八进制进行书写：
每个数字代标一层网络：
In RF24Network, the master is just `00`

- Children of master are `01`, `02`, `03`, `04`, `05`
- Children of `01` are `011`, `021`, `031`, `041`, `051`

2. 子节点发送数据给master时，需要延时一段时间，才能恢复