## "To learn, read; To know, write; To master, teach"

```txt
学slam已有几年, 总不清楚自己几斤几两, 故寄此巩固.   --- 2023-02-12

有兴趣了解或者参与项目的朋友可以联系: pty21821
```

## 前言

* 编译测试文件

```
mkdir -vp install/test
mkdir build && cd build
cmake .. -DCOMPILE_MODE=TEST
make
```

* 编译库文件

```
mkdir -vp install/lib install/include
mkdir build && cd build
cmake ..
make && make install
```
