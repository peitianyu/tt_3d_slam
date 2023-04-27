## platform

### 设计思路

```txt
实现类似于ros的多进程通信方案
根据想法设计模块:
1. 多进程通信
	a. 通过共享内存通信
	b. 通信时同步设计(读写锁)
	c. 数据传输(数据的序列化与反序列化, 校验(crc16))
	d. 实现订阅者与发布者
2. pin 与 pinOnce
	a. 使用定时器设计
3. 应用于整个平台
	a. 单例模式

因此设计模块有:
0. platform
1. rwlock
2. write read
3. serialize deserialize
4. subscrib publish
5. timer
6. crc16
7. timer
```
