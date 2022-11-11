#include <iostream>




class TestClass
{
    public:
        int printnumber(int number);
    protected:
        float number2;
    private:
};

int TestClass :: printnumber(int number)
{
    number += 10;
    for (int i = 0; i < 10; i++)
    {
        number++;
    }
    return number;
}

int main()
{
    TestClass test;
    int number = test.printnumber(1);
    std::cout << number << std::endl;
}