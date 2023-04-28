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

class RegisterTest
{
public:
	static RegisterTest *GetInstance(){
		static RegisterTest instance;
		return &instance;
	}

	void Register(const std::string &name, void (*test)()){
		if(!m_tests){ m_tests = new std::vector<ContextTest>();}
		m_tests->push_back(ContextTest(name, test));
	}

	bool RunAllTests(){
		std::cout << "[==========] Running " << m_tests->size() << " tests." << std::endl;
		for(auto &test : *m_tests){
			std::cout << "[ RUN      ] " << test.name << std::endl;
			test.test();
			std::cout << "[       OK ] " << test.name << std::endl;
		}
		std::cout << "[ ALL TESTS PASSED SUCCESS ]" << std::endl;
		return true;
	}
private:
	struct ContextTest{
		std::string name;
		void (*test)();
		
		ContextTest(const std::string &name, void (*test)()) : name(name), test(test) {}
	};

	std::vector<ContextTest> *m_tests;
};

#ifdef TEST_ENABLE
#define TEST(base, name)                              											\
	struct base##name##_Test                          											\
	{                                                 											\
		base##name##_Test(){                          											\
			RegisterTest::GetInstance()->Register(#base "." #name, &base##name##_Test::Run); 	\
		}                                             											\
		static void Run();                            											\
	};                                                											\
	base##name##_Test g_##base##name##_Test;          											\
	void base##name##_Test::Run()

#define RunAllTests() RegisterTest::GetInstance()->RunAllTests()

#else
#define TEST(base, name) void base##name##_Test()
#define RunAllTests() false
#endif // TEST_ENABLE


#endif // __COMMON_TEST_H__