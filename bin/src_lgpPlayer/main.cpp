#include <Logic/fol.h>
#include <Logic/folWorld.h>
#include <Kin/kin.h>
#include <LGP/LGP_tree.h>

//===========================================================================

const char *USAGE =
    "\nUsage:  lgpPlay <fol-filename> <g-filename> "
    "\n"
    "\n  -verbose <int>  "
    "\n";

//===========================================================================

void playFOL(const char* file){
  rai::FOL_World fol(file);
  fol.verbose = rai::getParameter<int>("verbose", 2);
  fol.reset_state();

  cout <<"*** initial FOL world:\n";
  cout <<"--- KB graph:\n";
  fol.write(cout);
  cout <<"\n--- PDDL domain:\n";
  fol.writePDDLdomain(cout);
  cout <<"--- PDDL problem:\n";
  fol.writePDDLproblem(cout);
  fol.writePDDLfiles("z");

  for(bool go=true;go;){
    bool terminal = fol.is_terminal_state();
    auto actions = fol.get_actions();
    cout <<"********************" <<endl;
    cout <<"STATE: ";
    fol.get_info(rai::FOL_World::writeState);
    cout <<"\nCHOICES:" <<endl;
    cout <<"(q) quit" <<endl;
    cout <<"(r) reset_state" <<endl;
    cout <<"(m) make_current_initial" <<endl;
    uint c=0;
    if(!terminal) for(auto& a:actions){ cout <<"(" <<c++ <<") DECISION: " <<*a <<endl; }

    rai::String cmd;
    {
      std::string tmp;
      getline(std::cin, tmp);
      cmd=tmp.c_str();
      cout <<"COMMAND: '" <<cmd <<"'" <<endl;
    }

    if(cmd=="q") go=false;
    else if(cmd=="r") fol.reset_state();
    else if(cmd=="m") fol.make_current_state_new_start();
    else {
      int choice=-1;
      cmd >>choice;
      cout <<"CHOICE=" <<choice <<endl;
      if(choice<0 || choice>=(int)actions.size()) {
        LOG(-1) <<"command '" <<c <<"' not known";
      } else {
        auto &a = actions[choice];
        cout <<"executing decision " <<*a <<endl;
        auto res=fol.transition(a);
        cout <<"->  result: obs=" <<*res.observation <<" reward=" <<res.reward <<endl;
      }
    }
  }
}

//===========================================================================

void playLGP(const char* folFile, const char* gFile){
  rai::Configuration C;
  C.addFile(gFile);
  rai::LGP_Tree lgp(C, folFile);

  cout <<"*** initial FOL world:\n";
  cout <<"--- KB graph:\n";
  lgp.fol.write(cout);
  cout <<"\n--- PDDL domain:\n";
  lgp.fol.writePDDLdomain(cout);
  cout <<"--- PDDL problem:\n";
  lgp.fol.writePDDLproblem(cout);
  lgp.fol.writePDDLfiles("z");

  lgp.player();
}


//===========================================================================

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);

  cout <<USAGE <<endl;

  rai::String folFile=rai::getParameter<rai::String>("folFile",STRING("none"));
  rai::String gFile=rai::getParameter<rai::String>("gFile",STRING("none"));
  if(rai::argc>=2 && rai::argv[1][0]!='-'){
    folFile=rai::argv[1];
    if(rai::argc>=3 && rai::argv[2][0]!='-') gFile=rai::argv[2];
  }
  LOG(0) <<"using fol-file '" <<folFile <<"'" <<endl;
  LOG(0) <<"using g-file '" <<gFile <<"'" <<endl;

  if(folFile=="none") return 0;

  rnd.clockSeed();

  if(gFile=="none") playFOL(folFile);
  else playLGP(folFile, gFile);

  cout <<"bye bye" <<endl;
}
