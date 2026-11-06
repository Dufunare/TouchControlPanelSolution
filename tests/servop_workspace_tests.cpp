#include "../shared/ServoPWorkspace.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace touchpanel::teleop;
int main() {
  std::array<double,6> last={10,-349.97,200,0,0,0};
  auto candidate=last;
  candidate[0]=13;candidate[1]=-353;candidate[2]=210;
  const unsigned held=holdOutwardServoPAxes(candidate,last);
  assert(held==2U);
  assert(candidate[0]==13 && candidate[1]==last[1]&&candidate[2]==210);
  assert(checkServoPWorkspace(candidate).code==ServoPWorkspaceCode::Allowed);
  candidate=last;candidate[0]=-351;candidate[1]=-351;candidate[2]=55;
  assert(holdOutwardServoPAxes(candidate,last)==3U);
  assert(candidate[0]==last[0]&&candidate[1]==last[1]&&candidate[2]==55);
  // The helper cannot authorize out-of-range previous targets.
  auto invalidPrevious=last;invalidPrevious[1]=-351;
  candidate=last;candidate[1]=-355;
  assert(holdOutwardServoPAxes(candidate,invalidPrevious)==0U);
  assert(checkServoPWorkspace(candidate).code==ServoPWorkspaceCode::XYZOutside);
  candidate=last;candidate[0]=std::numeric_limits<double>::quiet_NaN();
  assert(holdOutwardServoPAxes(candidate,last)==0U);
  assert(checkServoPWorkspace(candidate).code==ServoPWorkspaceCode::NonFinite);
  std::cout<<"ServoP virtual wall does not expand XYZ workspace\n";
}
