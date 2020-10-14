#include "animation.h"

void rai::Animation::AnimationPart::write(std::ostream& os) const{
    frameIDs.writeTagged(os, "frameIDs");
    X.writeTagged(os, "poses");
}

void rai::Animation::AnimationPart::read(std::istream& is) {
    frameIDs.readTagged(is, "frameIDs");
    X.readTagged(is, "poses");
}

void rai::Animation::read(istream& is) {
  A.readTagged(is, "animation");
}

void rai::Animation::write(ostream& os) const {
  A.writeTagged(os, "animation");
}

