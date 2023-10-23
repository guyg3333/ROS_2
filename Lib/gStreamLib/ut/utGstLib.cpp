#include <iostream>
#include "gStreamLib.h"
#include <boost/thread.hpp>



int main(int argc, char *argv[])
{
    std::string input;
    g_print("start\n");
    gStreamLib* hGst = new gStreamLib();
    boost::thread imageCapture = boost::thread(boost::bind(&gStreamLib::initPipline,hGst));

    //hGst->initPipline("cam",640,480);
    std::cout << "Thread waiting for user input: ";
    std::cin >> input;
    std::cout << "Thread received input: " << input << std::endl;

    hGst->closePipline();

    imageCapture.join();
    delete hGst;
    g_print("end\n");


    return 0;
}
