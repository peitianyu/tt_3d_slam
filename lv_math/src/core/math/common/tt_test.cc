#include "tt_test.h"

struct ContextTest
{
	std::string name;
	void (*test)();
	
	ContextTest(const std::string &name, void (*test)()) : name(name), test(test) {}
};

static std::vector<ContextTest> *g_tests;

void RegisterTest(const std::string &name, void (*test)())
{
	if(!g_tests){ g_tests = new std::vector<ContextTest>();}
	g_tests->push_back(ContextTest(name, test));
}

#ifdef TEST_ENABLE
bool RunAllTests()
{
	std::cout << "[==========] Running " << g_tests->size() << " tests." << std::endl;
	for(auto &test : *g_tests)
	{
		std::cout << "[ RUN      ] " << test.name << std::endl;
		test.test();
		std::cout << "[       OK ] " << test.name << std::endl;
	}
	std::cout << "[ ALL TESTS PASSED SUCCESS ]" << std::endl;
	return true;
}
#else
bool RunAllTests() { return true;}
#endif // TEST_ENABLE
