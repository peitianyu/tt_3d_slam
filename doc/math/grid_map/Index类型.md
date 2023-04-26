## index类型

### 设计思路

```txt
1. 设计index的目的是用于grid_map的id使用的, 因此有几种方案
	a. 可以通过(int)x, y, z形式来保存id信息
	b. 可以通过哈希值实现(Cantor)int id(然后转换为3维数据的形式), 实现过于复杂放弃
2. 同样希望index中可以保存resolution的信息, 不过这就会导致储存信息变多, 但这种方案的好处是
	a. 降采样非常方便
	b. index转point非常方便
3. 因此实现步骤
	a. Vector3i 与 resolution (实际上可以通过Vector3i实现, 不过这里自己实现了一遍)
	b. 通过point 与 resolution构造
	c. 便于单独使用x, y, z
	d. 可转换成point
```

### 代码设计

```c++
struct Index3D
{
    double resolution;
    Eigen::Vector3i index;

    Index3D(const Eigen::Vector3i& index = Eigen::Vector3i::Zero(), const double& res = 0.05);
    Index3D(const types::Point3D& point, const double& res = 0.05);
    types::Point3D Index2Point() const;

    const int& x() const;
    const int& y() const;
    const int& z() const;

    bool operator==(const Index3D &other) const;
    void operator+=(const Index3D &other);
    Index3D operator+(const Index3D &other) const;
};

```

### 哈希值方案(放弃)

```c++
// 由于实现复杂, 放弃
int cantor(int a, int b) {
  return (a + b + 1) * (a + b) / 2 + b;
}

int hash(int a, int b, int c) {
  return cantor(a, cantor(b, c));
}

void inverse_cantor(int z, int *x, int *y) {
  int w = floor((sqrt(8 * z + 1) - 1) / 2);
  int t = (w * w + w) / 2;
  *y = z - t;
  *x = w - *y;
}

void unhash(int z, int *a, int *b, int *c) {
  int N;
  inverse_cantor(z, a, &N);
  inverse_cantor(N, b, c);
}
```
