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
            this->master()->logFileRawName("moveJS");//建立记录数据的文件夹
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
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motionAtAbs(0).actualVel() << std::endl;
    }

    // log //
//    auto &lout = controller()->lout();
//    lout << controller()->motionAtAbs(0).targetPos() << ",";
//    lout << std::endl;
    lout() << controller()->motionAtAbs(0).actualPos() <<"\t";
    lout() << controller()->motionAtAbs(0).actualVel() <<std::endl;

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




//Tcurve
auto TcurveDrive::prepareNrt()->void
{
    cef_ = doubleParam("coefficient");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto TcurveDrive::executeRT()->int //进入实时线程
{
    static double begin_angle[3];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        this->master()->logFileRawName("TestMvj");//建立记录数据的文件夹
    }

//    std::int32_t digits;
//    this->ecController()->motionPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(5,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * cef_  * s1.getTCurve(count()) ;
    controller()->motionPool()[0].setTargetPos(angle0);
    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motionAtAbs(0).actualVel() << std::endl;
    }
    //log//
    lout() << controller()->motionAtAbs(0).actualPos() <<"\t";
    lout() << controller()->motionAtAbs(0).actualVel() <<std::endl;
    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto TcurveDrive::collectNrt()->void {}
TcurveDrive::TcurveDrive(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"test_mvj\">"
        "	<Param name=\"coefficient\" default=\"1\" abbreviation=\"k\"/>"
        "</Command>");
}
TcurveDrive::~TcurveDrive() = default;  //析构函数



//  速度模式
auto VelDrive::prepareNrt()->void{

    cef_ = doubleParam("coefficient");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto VelDrive::executeRT()->int{
    static double begin_vel[3];

    if (count()==1)
    {
        begin_vel[0] = controller()->motionPool()[0].actualVel();
        this->master()->logFileRawName("TestVel");
    }
    double vel0= begin_vel[0]+cef_*5.0*(1-std::cos(2*PI*count()/2000.0))/2;
    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motionAtAbs(0).actualVel() << std::endl;
    }
    //log//
    lout() << controller()->motionAtAbs(0).actualPos() <<"\t";
    lout() << controller()->motionAtAbs(0).actualVel() <<std::endl;
    controller()->motionPool()[0].setTargetVel(vel0);
    return 2000-count();
}

auto VelDrive::collectNrt()->void{}
VelDrive::VelDrive(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"test_vel\">"
        "	<Param name=\"coefficient\" default=\"1.0\" abbreviation=\"k\"/>"
        "</Command>");
}
VelDrive::~VelDrive() = default;  //析构函数




auto createControllerMotor()->std::unique_ptr<aris::control::Controller>
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
        double pos_factor[3] //偏置系数
        {
            2000/PI,2000/PI,2000/PI
        };
        double max_pos[3] //最大位置
        {
            500*PI,500*PI,500*PI
        };
        double min_pos[3] //最小位置
        {
            -500*PI,-500*PI,-500*PI
        };
        double max_vel[3]  //最大速度
        {
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
        };
        double max_acc[3]  //最大加速度
        {
            3000,  3000,  3000
        };

        int phy_id[3]={0,1,2};


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
            "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
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
            "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
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
auto createPlanMotor()->std::unique_ptr<aris::plan::PlanRoot>
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
    plan_root->planPool().add<TcurveDrive>();
    plan_root->planPool().add<MoveJS>();
    plan_root->planPool().add<VelDrive>();
    return plan_root;
}

}
