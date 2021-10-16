#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{

    class TcurveDrive :public aris::core::CloneObject<TcurveDrive,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~TcurveDrive();
        explicit TcurveDrive(const std::string &name = "motor_drive");

    private:
        double cef_;
    };


    class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
      {
      public:
          auto virtual prepareNrt()->void;
          auto virtual executeRT()->int;
          auto virtual collectNrt()->void;

          explicit MoveJS(const std::string &name = "MoveJS_plan");

      };

    class VelDrive : public aris::core::CloneObject<VelDrive,aris::plan::Plan>
    {
     public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~VelDrive();
        explicit VelDrive(const std::string &name = "vel_drive");

     private:
        double cef_;
    };


    class XYZmove : public aris::core::CloneObject<XYZmove,aris::plan::Plan>
    {
     public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~XYZmove();
        explicit XYZmove(const std::string &name = "xyz_drive");

     private:
        double x_;
        double y_;
        double z_;
    };

    class ReadPos : public aris::core::CloneObject<ReadPos,aris::plan::Plan>
    {
     public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ReadPos();
        explicit ReadPos(const std::string &name = "read_pos");


    };




    auto createControllerMotor()->std::unique_ptr<aris::control::Controller>;
    auto createPlanMotor()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
