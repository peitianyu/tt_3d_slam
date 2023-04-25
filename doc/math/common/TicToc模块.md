## 统计程序运行时间

### 设计思路

```txt
1. tic时初始化时间
2. toc时统计程序运行时间
```

### 代码设计

```c++
class TicToc
{
public:
    TicToc() {Tic();}

    // tic时初始化时间
    void Tic() {gettimeofday(&last_time_, NULL);}

    // toc时统计程序运行时间
    float Toc(){
    	struct timeval now;
    	gettimeofday(&now, NULL);
    	return (now.tv_sec - last_time_.tv_sec) + float(now.tv_usec - last_time_.tv_usec) / 1e6;
    }
private:
    struct timeval last_time_;
};
```

### 使用方法

```c++
#include "common/tt_tic_toc.h"

TEST(common, sleep_tictoc)
{
    common::TicToc tic_toc;
    // tic时初始化时间
    tic_toc.Tic();
    common::Sleep(1.0); // sleep 1s
    // toc时统计程序运行时间
    ASSERT_TRUE(tic_toc.Toc() < 1.0 + 1e-3);
}
```
