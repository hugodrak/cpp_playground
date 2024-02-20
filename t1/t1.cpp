#include <iostream>
#include <iomanip>
void test(float& hej) {
	hej+= 3.12313123123123;
}


int main() {
	float hej = 0;
	test(hej);
	test(hej);
    std::cout << std::fixed << std::setprecision(4) << hej << std::endl;
	return 0;
}