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