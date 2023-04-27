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


std::vector<void (*)()> tests;
std::vector<std::string> test_names;

#define TEST(base, name)                              \
	struct base##name##_Test                          \
	{                                                 \
		base##name##_Test(){                          \
			test_names.push_back(#base "." #name);    \
			tests.push_back(&base##name##_Test::Run); \
		}                                             \
		static void Run();                            \
	};                                                \
	base##name##_Test g_##base##name##_Test;          \
	void base##name##_Test::Run()

bool RunAllTests()
{
	for (uint i = 0; i < tests.size(); ++i)
	{
		std::cout << "[ RUN      ] " << test_names[i] << std::endl;
		tests[i]();
	}
	std::cout << "[ ALL TESTS PASSED SUCCESS ]" << std::endl;
	return true;
}





#endif // __COMMON_TEST_H__