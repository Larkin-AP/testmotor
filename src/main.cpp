#include <aris.hpp>
#include "robot.h"



int main(int argc, char *argv[])
{
 //   std::cout << "cs.controller().xmlString()" << std::endl;

    auto&cs = aris::server::ControlServer::instance();
	
    cs.resetController(robot::createControllerQuadruped().release());
    cs.resetPlanRoot(robot::createPlanQuadruped().release());
//    std::cout << cs.controller().xmlString() << std::endl;
    cs.init();



	//开启WebSocket/socket服务器//
    cs.open(); 


    cs.start();
//    auto &ec = dynamic_cast<aris::control::EthercatController&>(cs.controller());


//    uint32_t data=1;
//    for(int i=1;i<10000000000000;i++)
//    {
//        if(data!=1)
//        {
//            ec.slavePool()[0].readPdo(0x60FD,0,data);
//            std::cout<< data<<std::endl;
//        }
//        else if (data==1)
//                    std::cout<< "2222"<<std::endl;
//        // std::cout<< "1111"<<std::endl;

//    }



	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
    cs.runCmdLine();

	return 0;
}
