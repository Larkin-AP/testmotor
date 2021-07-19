#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "robot.h"
#include"plan.h"

using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;
namespace robot
{



// 单关节正弦往复轨迹 //
struct MoveJSParam
{
    double j1;
    double time;
    uint32_t timenum;
};
auto MoveJS::prepareNrt()->void
{
    MoveJSParam param;

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto &p : cmdParams())
    {
        if (p.first == "j1")
        {
            if (p.second == "current_pos")
            {
                param.j1 = controller()->motionPool()[0].actualPos();
            }
            else
            {
                param.j1 = doubleParam(p.first);
            }

        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;
    std::vector<std::pair<std::string, std::any>> ret_value;
    for (auto &option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|NOT_CHECK_POS_CONTINUOUS;
    ret() = ret_value;
}
auto MoveJS::executeRT()->int
{
    auto &param = std::any_cast<MoveJSParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;
    // 访问主站 //
    auto &cout = controller()->mout();

    if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }

        step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {
        // 获取当前起始点位置 //
        if (count() == totaltime - time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }

    // 打印 //
    if (count() % 100 == 0)
    {
        cout << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
        cout << std::endl;
    }

    // log //
    auto &lout = controller()->lout();
    lout << controller()->motionAtAbs(0).targetPos() << ",";
    lout << std::endl;

    return totaltime - count();
}
auto MoveJS::collectNrt()->void {}
MoveJS::MoveJS(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"moveJS\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}




//轴空间移动
auto DogMoveJoint::prepareNrt()->void
{
    dir_ = doubleParam("direction");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

}
auto DogMoveJoint::executeRT()->int
{
    static double begin_angle[18];

    if (count() == 1)
    {

        begin_angle[0] = controller()->motionPool()[0].targetPos();
//        begin_angle[1] = controller()->motionPool()[1].targetPos();
//        begin_angle[2] = controller()->motionPool()[2].targetPos();
//        begin_angle[3] = controller()->motionPool()[3].targetPos();
//        begin_angle[4] = controller()->motionPool()[4].targetPos();
//        begin_angle[5] = controller()->motionPool()[5].targetPos();
//        begin_angle[6] = controller()->motionPool()[6].targetPos();
//        begin_angle[7] = controller()->motionPool()[7].targetPos();
//        begin_angle[8] = controller()->motionPool()[8].targetPos();
//        begin_angle[9] = controller()->motionPool()[9].targetPos();
//        begin_angle[10] = controller()->motionPool()[10].targetPos();
//        begin_angle[11] = controller()->motionPool()[11].targetPos();
//        begin_angle[12] = controller()->motionPool()[12].targetPos();
//        begin_angle[13] = controller()->motionPool()[13].targetPos();
//        begin_angle[14] = controller()->motionPool()[14].targetPos();
//        begin_angle[15] = controller()->motionPool()[15].targetPos();
//        begin_angle[16] = controller()->motionPool()[16].targetPos();
//        begin_angle[17] = controller()->motionPool()[17].targetPos();
    //    std::cout<<begin_angle[0]<<std::endl;
      //  begin_angle[1] = controller()->motionPool()[1].targetPos();
     //   std::cout<<begin_angle[1]<<std::endl;
        this->master()->logFileRawName("TestMotor");//建立文件夹
    }
//    std::int32_t digits;
//    this->ecController()->motionPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;




//  梯形曲线
    TCurve s1(1,1);
    s1.getCurveParam();
    double angle0 = begin_angle[0] + dir_ * 1 * s1.getTCurve(count());

//    lout() << angle0 << std::endl;//将电机角度输出到文件

//    controller()->motionPool()[0].setTargetPos(angle0);
//    return s1.getTc() * 1000-count();

//    Cos曲线
//    double angle0 = begin_angle[0] + dir_ * (1 - cos(PI * count()/1000.0))/2;
//    controller()->motionPool()[0].setTargetPos(angle0);
//    return 1000-count();

//      Sin曲线
//    double angle0 = begin_angle[0] + dir_ * 10*sin(PI * count()/15000.0);
//    double angle0 = begin_angle[0] + dir_ * count() * 0.001;
//    double angle0 = begin_angle[0] + dir_ * sin(PI * count()/2/1000.0);
//    double angle1 = begin_angle[1] + dir_ * sin(PI * count()/2/1000.0);
//    double angle2 = begin_angle[2] + dir_ * sin(PI * count()/2/1000.0);
//    double angle3 = begin_angle[3] + dir_ * sin(PI * count()/2/1000.0);
//    double angle4 = begin_angle[4] + dir_ * sin(PI * count()/2/1000.0);
//    double angle5 = begin_angle[5] + dir_ * sin(PI * count()/2/1000.0);
//    double angle6 = begin_angle[6] + dir_ * sin(PI * count()/2/1000.0);
//    double angle7 = begin_angle[7] + dir_ * sin(PI * count()/2/1000.0);
//    double angle8 = begin_angle[8] + dir_ * sin(PI * count()/2/1000.0);
//    double angle9 = begin_angle[9] + dir_ * sin(PI * count()/2/1000.0);
//    double angle10 = begin_angle[10] + dir_ * sin(PI * count()/2/1000.0);
//    double angle11 = begin_angle[11] + dir_ * sin(PI * count()/2/1000.0);
//    double angle12 = begin_angle[12] + dir_ * 15*sin(PI * count()/2/1000.0);
//    double angle13 = begin_angle[13] + dir_ * 15*sin(PI * count()/2/1000.0);
//    double angle14 = begin_angle[14] + dir_ * 15*sin(PI * count()/2/1000.0);
//    double angle15 = begin_angle[15] + dir_ * 15*sin(PI * count()/2/1000.0);
//    double angle16 = begin_angle[16] + dir_ * 15*sin(PI * count()/2/1000.0);
//    double angle17 = begin_angle[17] + dir_ * 15*sin(PI * count()/2/1000.0);
//    double angle1 = begin_angle[1] + dir_ * sin(PI * count()/2/1000.0);
    controller()->motionPool()[0].setTargetPos(angle0);
//    controller()->motionPool()[1].setTargetPos(angle1);
//    controller()->motionPool()[2].setTargetPos(angle2);
//    controller()->motionPool()[3].setTargetPos(angle3);
//    controller()->motionPool()[4].setTargetPos(angle4);
//    controller()->motionPool()[5].setTargetPos(angle5);
//    controller()->motionPool()[6].setTargetPos(angle6);
//    controller()->motionPool()[7].setTargetPos(angle7);
//    controller()->motionPool()[8].setTargetPos(angle8);
//    controller()->motionPool()[9].setTargetPos(angle9);
//    controller()->motionPool()[10].setTargetPos(angle10);
//    controller()->motionPool()[11].setTargetPos(angle11);
//    controller()->motionPool()[12].setTargetPos(angle12);
//    controller()->motionPool()[13].setTargetPos(angle13);
//    controller()->motionPool()[14].setTargetPos(angle14);
//    controller()->motionPool()[15].setTargetPos(angle15);
//    controller()->motionPool()[16].setTargetPos(angle16);
//    controller()->motionPool()[17].setTargetPos(angle17);

//    for(int i = 0;i<18;++i){
//        double angle = begin_angle[i] + dir_ *  count() * 0.002;
//        //double angle = begin_angle[i] + dir_ * 5* std::sin(PI * count()/2/1000.0);
//        controller()->motionPool()[i].setTargetPos(angle);

//    }

    //    controller()->motionPool()[1].setTargetPos(angle1);
//    lout() << angle0 << std::endl;
//    mout() << angle0 << std::endl;
    mout() << controller()->motionPool()[0].actualPos() <<std::endl;

    //return 3600000-count();
    return 10000-count();


}
auto DogMoveJoint::collectNrt()->void {}
DogMoveJoint::DogMoveJoint(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"test_mvj\">"
        "	<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
DogMoveJoint::~DogMoveJoint() = default;



auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 1; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[2]
        {
            0,0,0,0,0,0
        };
#else
        double pos_offset[2]
        {
         //  1.900100

        };
#endif
        double pos_factor[18]
        {
//            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
//            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
//            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
//            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
            4000/PI,4000/PI,4000/PI,4000/PI,4000/PI,4000/PI,
            4000/PI,4000/PI,4000/PI,4000/PI,4000/PI,4000/PI,
            4000/PI,4000/PI,4000/PI,4000/PI,4000/PI,4000/PI
        };
        double max_pos[18]
        {
            50000*PI,500*PI,500*PI,500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,500*PI,500*PI,500*PI
        };
        double min_pos[18]
        {
            -50000*PI,-500*PI,-500*PI,-500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,-500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,-500*PI,-500*PI,-500*PI
        };
        double max_vel[18]
        {
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
        };
        double max_acc[18]
        {
            3000,  3000,  3000,3000,  3000,  3000,
            3000,  3000,  3000,3000,  3000,  3000,
            3000,  3000,  3000,3000,  3000,  3000
        };

        int phy_id[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};


        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1600\" is_tx=\"false\">"
            "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
//            "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
            "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
            "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatMotor>";


        auto &s = controller->slavePool().add<aris::control::EthercatMotor>();
        aris::core::fromXmlString(s,xml_str);

#ifdef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif

#ifndef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif

        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setDcAssignActivate(0x300);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setControlWord(0x00);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setModeOfOperation(0x08);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setTargetPos(0.0);
    };
    return controller;
}
auto createPlanQuadruped()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();

    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //自己写的命令
    plan_root->planPool().add<DogMoveJoint>();
    plan_root->planPool().add<MoveJS>();
    return plan_root;
}

}
