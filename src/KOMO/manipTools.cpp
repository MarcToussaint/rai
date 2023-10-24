#include "manipTools.h"

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName, bool pre) {
    arr xLine, yzPlane;
    FeatureSymbol xyScalarProduct=FS_none, xzScalarProduct=FS_none;
    if(dir==rai::_xAxis){
        xLine = arr{{1,3},{1,0,0}};
        yzPlane = arr{{2,3},{0,1,0,0,0,1}};
        xyScalarProduct = FS_scalarProductXY;
        xzScalarProduct = FS_scalarProductXZ;
    } else if(dir==rai::_yAxis){
        xLine = arr{{1,3},{0,1,0}};
        yzPlane = arr{{2,3},{1,0,0,0,0,1}};
        xyScalarProduct = FS_scalarProductXX;
        xzScalarProduct = FS_scalarProductXZ;
    } else if(dir==rai::_zAxis){
        xLine = arr{{1,3},{0,0,1}};
        yzPlane = arr{{2,3},{1,0,0,0,1,0}};
        xyScalarProduct = FS_scalarProductXX;
        xzScalarProduct = FS_scalarProductXY;
    }

    double margin=.02;

    //position: center in inner target plane; X-specific
    if(!pre){
      komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e1, {});
      komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*1e0, (boxSize/2.-margin));
      komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*(-1e0), -(boxSize/2.-margin));
    }else{
      komo.addObjective({time, time+1.}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e1, {});
    }

    //orientation: grasp axis orthoginal to target plane; X-specific
    komo.addObjective({time-.2,time}, xyScalarProduct, {gripperName, boxName}, OT_eq, {1e0}, {});
    komo.addObjective({time-.2,time}, xzScalarProduct, {gripperName, boxName}, OT_eq, {1e0}, {});

    //no collision with palm
    if(!pre){
      komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.001});
    }else{
      komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_eq, {1e1}, {-.07});
    }

    //approach: only longitudial velocity, min distance before and at grasp
    if(komo.k_order>1) komo.addObjective({time-.3,time}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
    if(komo.k_order>1) komo.addObjective({time-.5,time-.3}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

    //zero vel
    if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

void addBoxPlaceObjectives(KOMO& komo, double time,
                           rai::ArgWord dir, const char* boxName, const arr& boxSize,
                           const char* tableName,
                           const char* gripperName, const char* palmName,
                           double margin, bool pre) {
    double relPos=0.;
    FeatureSymbol zVector = FS_none;
    arr zVectorTarget = {0.,0.,1.};
    if(dir==rai::_xAxis){
        relPos = .5*boxSize(0)+.03;
        zVector = FS_vectorX;
    } else if(dir==rai::_yAxis){
        relPos = .5*boxSize(1)+.03;
        zVector = FS_vectorY;
    } else if(dir==rai::_zAxis){
        relPos = .5*boxSize(2)+.03;
        zVector = FS_vectorZ;
    } else if(dir==rai::_xNegAxis){
        relPos = .5*boxSize(0)+.03;
        zVector = FS_vectorX;
        zVectorTarget *= -1.;
    } else if(dir==rai::_yNegAxis){
        relPos = .5*boxSize(1)+.03;
        zVector = FS_vectorY;
        zVectorTarget *= -1.;
    } else if(dir==rai::_zNegAxis){
        relPos = .5*boxSize(2)+.03;
        zVector = FS_vectorZ;
        zVectorTarget *= -1.;
    }

    //z-position: fixed
    if(!pre){
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({1,3},{0,0,1}), {.0, .0, relPos});
    }else{
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({1,3},{0,0,1}), {.0, .0, relPos+.04});
    }

    //xy-position: above table
    if(!pre){
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({2,3},{1,0,0,0,1,0}));
      //komo.addObjective({time}, FS_aboveBox, {boxName, tableName}, OT_ineq, {3e0}, {margin});
    }else{
      komo.addObjective({time}, FS_positionDiff, {boxName, tableName}, OT_eq, 1e1*arr({2,3},{1,0,0,0,1,0}));
      //komo.addObjective({time, time+1.}, FS_aboveBox, {boxName, tableName}, OT_ineq, {3e0}, {margin});
    }

    //orientation: Y-up
    komo.addObjective({time-.2, time}, zVector, {boxName}, OT_eq, {0.5}, zVectorTarget);

    //retract: only longitudial velocity, min distance after grasp
    if(komo.k_order>1) komo.addObjective({time,time+.3}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
    if(komo.k_order>1) komo.addObjective({time+.3,time+.5}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

    //zero vel
    if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}
