#include "calculate.hpp"
#include <iostream>
#include "GxIAPI.h"
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>


int main()
{
    int a = 1, b = 2;
    std::cout << calculate::addition(a, b);
    std::cout << calculate::subtract(a, b);

    return 0;
}