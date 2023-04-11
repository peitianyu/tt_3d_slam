## 该程序实现了类似于ros的多进程发布订阅模式[共享内存]

```c++
// publish.cc


void Test()
{
    Point point(1, 2, 3, 4);
    std::string topic = "/test";
    platform::Platform::getInstance().CreatePublisher(topic, sizeof(Point));

    platform::Serialize serialize;
    serialize << point;

    platform::Platform::getInstance().Publish(topic, serialize.str());

    LOG_DEBUG("Publish: ", point) << std::endl;

    common::RateSleep(2.0);
}


// subscribe.cc

void Callback(const std::string &data)
{
    platform::Deserialize deserialize(data);
    Point point;
    deserialize >> point;
    
    LOG_INFO("Subscribe: ", point) << std::endl;
    LOG_WARN("Subscribe: ", point) << std::endl;
    LOG_DEBUG("Subscribe: ", point) << std::endl;
    LOG_ERROR("Subscribe: ", point) << std::endl;

    ASSERT_EQ(point.x, 1);
    ASSERT_EQ(point.y, 2);
    ASSERT_EQ(point.z, 3);
    ASSERT_EQ(point.intensity, 2); // fail
}

TEST(Platform, Deserialize)
{
    std::string topic = "/test";
    platform::Platform::getInstance().CreateSubscriber(topic, sizeof(Point), Callback, 1.0);

    // platform::Platform::getInstance().Spin();
    platform::Platform::getInstance().SpinOnce();
}
```


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
