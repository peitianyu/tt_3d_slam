#ifndef __COMMON_TEST_H__
#define __COMMON_TEST_H__

#include <iostream>
#include <vector>

class Tester
{
public:
	Tester(const std::string &name, uint line) : m_name(name), m_line(line), m_ok(true) {}
	~Tester(){ if(!m_ok) exit(1); }
	Tester &Is(bool x, const std::string &msg = ""){
		if (!x){
			std::cout << "[      !OK ] " << m_name << ":" << m_line << " " << msg << std::endl;
			m_ok = false;
		}
		return *this;
	}

private:
	std::string m_name;
	uint m_line;
	bool m_ok;
};

#define ASSERT_TRUE(x) Tester(__FILE__, __LINE__).Is((x), #x)
#define ASSERT_FALSE(x) Tester(__FILE__, __LINE__).Is(!(x), #x)
#define ASSERT_EQ(a, b) Tester(__FILE__, __LINE__).Is((a) == (b), #a " == " #b)
#define ASSERT_NE(a, b) Tester(__FILE__, __LINE__).Is((a) != (b), #a " != " #b)
#define ASSERT_LT(a, b) Tester(__FILE__, __LINE__).Is((a) < (b), #a " < " #b)
#define ASSERT_LE(a, b) Tester(__FILE__, __LINE__).Is((a) <= (b), #a " <= " #b)
#define ASSERT_GT(a, b) Tester(__FILE__, __LINE__).Is((a) > (b), #a " > " #b)
#define ASSERT_GE(a, b) Tester(__FILE__, __LINE__).Is((a) >= (b), #a " >= " #b)

#ifdef TEST_ENABLE
#define TEST(base, name)                              \
	struct base##name##_Test                          \
	{                                                 \
		base##name##_Test(){                          \
			RegisterTest(#base "." #name, &Run);      \
		}                                             \
		static void Run();                            \
	};                                                \
	base##name##_Test g_##base##name##_Test;          \
	void base##name##_Test::Run()

void RegisterTest(const std::string &name, void (*test)());

#else
#define TEST(base, name) \
	void base##name##_Test()
	
#endif // TEST_ENABLE

bool RunAllTests();


#endif // __COMMON_TEST_H__