#include <Logic/relationalMachine.h>

//===========================================================================

void TEST(RM){
  rai::RelationalMachine RM("machine.g");

  cout <<"symbols = " <<RM.getSymbols() <<endl;
  cout <<"rules = " <<RM.getRules() <<endl;
  cout <<"state = " <<RM.getState() <<endl;

  RM <<"(init)";
  RM.fwdChainRules();

  RM <<"(alignHand conv) (positionHand conv)";  //return msg from the actions
  RM.fwdChainRules();

  RM <<"(lowerHand conv)";  //return msg from the actions
  RM.fwdChainRules();

  cout <<"(homing) condition?" <<RM.queryCondition("(homing)") <<endl;

  RM <<"(controlForce timeout)";  //return msg from the actions
  RM.fwdChainRules();

  cout <<"(homing) condition?" <<RM.queryCondition("(homing)") <<endl;

  RM <<"(homing conv)";  //return msg from the actions
  RM.fwdChainRules();

  RM <<"(undefined)";  //generates error

}

//===========================================================================

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testRM();
}
