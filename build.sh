# 删除所有build与install
rm -rf */build */install

# 安装lv_math viz 与 lv_platform
cd lv_math && mkdir build && cd build && cmake .. && make -j &&  make install && cd ../..
cd lv_platform && mkdir build && cd build && cmake .. && make -j &&  make install && cd ../..
cd viz && mkdir build && cd build && cmake .. && make -j &&  make install && cd ../..

# 测试lv_math viz 与 lv_platform
cd lv_math/build && cmake .. -DCOMPILE_MODE=TEST && make -j && cd ../..
cd lv_platform/build && cmake .. -DCOMPILE_MODE=TEST && make -j && cd ../..
cd viz/build && cmake .. -DCOMPILE_MODE=TEST && make -j && cd ../..