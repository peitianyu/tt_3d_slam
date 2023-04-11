#include<iostream>
#include<cmath>

namespace front_end{
namespace gnss{
namespace UTM{

void LLtoUTM(double lat, double lon, double &x, double &y, int &zone, bool &north);
void UTMtoLL(double x, double y, double &lat, double &lon, int &zone, bool &north);

} // namespace UTM
} // namespace gnss
} // namespace front_end
