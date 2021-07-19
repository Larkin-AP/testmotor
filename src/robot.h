#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{

    class DogMoveJoint :public aris::core::CloneObject<DogMoveJoint,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~DogMoveJoint();
        explicit DogMoveJoint(const std::string &name = "dog_move_joint");

    private:
        double dir_;
    };


    class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
      {
      public:
          auto virtual prepareNrt()->void;
          auto virtual executeRT()->int;
          auto virtual collectNrt()->void;

          explicit MoveJS(const std::string &name = "MoveJS_plan");

      };
    

    auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>;
    auto createPlanQuadruped()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
