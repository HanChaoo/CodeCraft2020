# 2020华为软件精英挑战赛(Code Craft)

## 前言
* 团队名：小金鱼拉着一小摩托的小火车
* 成绩：上合赛区初赛rank3，复赛a榜rank4，b榜rank1
* 成员：[syzhao](github.com/SongyuanZhao)，[hanchaoo](github.com/HanChaoo)，[washboard](github.com/washboard)


## 赛题分析
给定包含资金流水的文本文件，每一行代表一次资金交易记录，输出满足限制条件下的循环转账个数和循环转账路径详情。
具体题目在[这里](https://github.com/HanChaoo/CodeCraft2020/tree/master/%E8%B5%9B%E9%A2%98)。

可将题目抽象为**有向图找环**，确立解题步骤：读文件->建图->dfs找环->结果输出到文件。


## 具体实现
* **读文件**<br>
多线程mmap和排序：将每条交易的`dst`、`src`、`weight`存入结构体，并根据`src`(main key)和`dst`(secondary keyword)将结构体排为升序。<br>
按`dst`、`src`（均为32位）顺序存放是因为使用的机器是小端序，这样可以直接按64位排序，效率比较高。<br>

* **建图**<br>
（1）id映射，只对src中的点进行hash映射，这样可以排除掉出度为0的点<br>
（2）前向星算法存图，并统计每个节点的入度<br>
（3）拓扑排序，标记入度为0的点及其对应的边<br>
（4）更新图，删除拓扑排序的标记点和边，并更新结点的出度和入度，同时存储映射后结点所对应的原字符串<br>

* **dfs找环**<br>
（1）采用4+3，对每一个结点，先反向搜索3层并使用`path3`存储中间2层结点<br>
（2）在反向搜索中，对反向3层能搜到的点进行染色，用以正向搜索时避免对path3不必要的访问，减少cache miss<br>
（3）正向搜索4层，第1、2、3、4层分别利用path3找长度为4、5、6、7的环，同时第3层还负责寻找长度为3的环<br>
（4）每搜索到一个环，将路径上各个结点转为字符串存入结果变量<br>

* **输出结果文件**<br>
多线程mmap<br>


## 优化细节
* **数据结构**<br>
（1）避免使用STL，自定义hashmap、vector、string<br>
（2）结构体对齐，并使用pad进行填充，使其长度>=cacheline，避免false sharing<br>
（3）尽量使用较小的数据结构，能用uint16绝不用uint32<br>

* **反向搜索**<br>
（1）找环时使用快慢速组合策略，提升算法效率的同时，增强代码鲁棒性。先使用`数组存储path3`（快速）进行找环，若path3数组大小不够，则落回到`vector存储path3`（慢速）进行找环，可以有效提升程序效率<br>
（2）`数组存储path3`
时，按需分配path3存储空间，可以有效减小path3数组大小，并适当增大path3分配给每一结点的宽度。这样可以避免因为数据集增大导致的静态数组过度膨胀<br>
（3）path3大部分访问都集中在每一行的前边，拒绝将其宽度设置为cacheline的整数倍，避免魔鬼步长，可以降低cache mis。<br>

* **负载均衡**<br>
多线程找环，使用抢占式任务调度，每次任务分配256个结点。<br>



